// Transparent USB <-> Feetech servo bus bridge for the Waveshare ESP32 servo driver board.
//
// Host (Pi) link: Serial (USB via the board's CP2102), 115200 baud.
// Servo bus:      Serial1 on S_RXD/S_TXD, 1,000,000 baud, half duplex (MAX485_DE selects transmit).
//
// Host -> bus: each packet (FF FF ID LEN ..., LEN + 4 bytes) is forwarded as soon as its last
// byte arrives, in one burst. A partial or malformed packet is forwarded after 2 ms with no
// new byte, so the stream resynchronises.
// Bus -> host: bytes are passed through as they arrive.
// The OLED (status: servo IDs found at boot, byte counts) refreshes only while the host has been
// quiet for 200 ms, because a refresh blocks for several milliseconds of I2C traffic.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define S_RXD 18
#define S_TXD 19
#define MAX485_DE 21

#define HOST_BAUD 115200   // Pi <-> ESP32 (CP2102 supports up to 1 Mbaud ... but this waveshare board doesn't seem to))
#define BUS_BAUD  1000000   // ESP32 <-> servos (STS factory setting)

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

unsigned long hostBytesCount = 0;
unsigned long servoBytesCount = 0;
unsigned long lastOledUpdate = 0;
const unsigned long oledInterval = 1000;        // ms between OLED refreshes
const unsigned long oledQuietUs = 200000;       // host must be quiet this long before a refresh
const unsigned long partialPacketUs = 2000;     // forward an incomplete packet after this silence

String discoveredIDs = "None";

// Host -> bus packet assembly
static uint8_t hostPkt[260];   // largest packet: LEN 255 + 4
static size_t hostLen = 0;
static unsigned long lastHostByteUs = 0;

static void forwardToBus(const uint8_t* buf, size_t n) {
  digitalWrite(MAX485_DE, HIGH);   // transmit
  Serial1.write(buf, n);
  Serial1.flush();                 // wait until the last byte has left the UART
  delayMicroseconds(5);
  digitalWrite(MAX485_DE, LOW);    // back to receive for the servo's reply
  hostBytesCount += n;
}

bool pingServo(uint8_t id) {
  while (Serial1.available()) Serial1.read();
  uint8_t packet[6] = {0xFF, 0xFF, id, 0x02, 0x01, (uint8_t)~(id + 0x02 + 0x01)};

  digitalWrite(MAX485_DE, HIGH);
  Serial1.write(packet, 6);
  Serial1.flush();
  delayMicroseconds(5);
  digitalWrite(MAX485_DE, LOW);

  delay(5);
  if (Serial1.available() >= 6) {
    if (Serial1.read() == 0xFF && Serial1.read() == 0xFF) {
      for (int i = 0; i < 4; i++) { Serial1.read(); }
      return true;
    }
  }
  return false;
}

void updateOled() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("IDs: "); display.println(discoveredIDs);
  display.setCursor(0, 12);
  display.print("USB:115k BUS:1M");
  display.setCursor(0, 24);
  display.print("H:"); display.print(hostBytesCount);
  display.print("  S:"); display.print(servoBytesCount);
  display.display();
}

void setup() {
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, LOW);

  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("ROS2 HYBRID BRIDGE");
    display.println("Scanning bus (1M)...");
    display.display();
  }

  Serial.begin(HOST_BAUD);
  Serial1.begin(BUS_BAUD, SERIAL_8N1, S_RXD, S_TXD);

  delay(2000);  // servo power-up

  String foundIDs = "";
  for (uint8_t id = 1; id <= 6; id++) {
    if (pingServo(id)) {
      if (foundIDs.length() > 0) foundIDs += ",";
      foundIDs += String(id);
    }
  }
  if (foundIDs.length() > 0) discoveredIDs = foundIDs;

  // Drop anything the host sent during the scan, so the first packet starts clean
  while (Serial.available()) Serial.read();
  updateOled();
}

void loop() {
  // 1. Host -> bus: forward each complete packet immediately
  while (Serial.available() > 0) {
    if (hostLen < sizeof(hostPkt)) hostPkt[hostLen++] = Serial.read();
    else Serial.read();  // cannot happen: hostPkt holds the largest possible packet
    lastHostByteUs = micros();
    if (hostLen >= 4 && hostLen == (size_t)hostPkt[3] + 4) {
      forwardToBus(hostPkt, hostLen);
      hostLen = 0;
    }
  }
  // Partial or malformed input: forward what arrived after a short silence (resync)
  if (hostLen > 0 && micros() - lastHostByteUs > partialPacketUs) {
    forwardToBus(hostPkt, hostLen);
    hostLen = 0;
  }

  // 2. Bus -> host: pass replies straight through
  while (Serial1.available() > 0) {
    Serial.write(Serial1.read());
    servoBytesCount++;
  }

  // 3. OLED: only while the host is quiet, so the blocking refresh never delays forwarding
  if (millis() - lastOledUpdate >= oledInterval && micros() - lastHostByteUs > oledQuietUs) {
    updateOled();
    lastOledUpdate = millis();
  }
}
