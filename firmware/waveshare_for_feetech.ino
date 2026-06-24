#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define S_RXD 18
#define S_TXD 19
#define MAX485_DE 21

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

unsigned long hostBytesCount = 0;
unsigned long servoBytesCount = 0;
unsigned long lastOledUpdate = 0;
const unsigned long oledInterval = 1000; 

String discoveredIDs = "None";

bool pingServo(uint8_t id) {
  while(Serial1.available()) Serial1.read();
  uint8_t packet[6] = {0xFF, 0xFF, id, 0x02, 0x01, (uint8_t)~(id + 0x02 + 0x01)};
  
  digitalWrite(MAX485_DE, HIGH);
  Serial1.write(packet, 6);
  Serial1.flush();
  delayMicroseconds(5);
  digitalWrite(MAX485_DE, LOW);

  delay(5); 
  if (Serial1.available() >= 6) {
    if (Serial1.read() == 0xFF && Serial1.read() == 0xFF) {
      for(int i = 0; i < 4; i++) { Serial1.read(); }
      return true;
    }
  }
  return false;
}

void setup() {
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, LOW);

  if(display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("ROS2 HYBRID BRIDGE");
    display.println("Scanning bus (1M)...");
    display.display();
  }

  // --- HARDWARE DECOUPLED CLOCK SPEEDS ---
  Serial.begin(115200);                                 // Pi 5 USB Link: Stable 115200
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);     // Servo Bus: High-Speed 1M Native
  
  delay(2000); // Complete servo initialization guard window

  String foundIDs = "";
  for (uint8_t id = 1; id <= 6; id++) {
    if (pingServo(id)) {
      if (foundIDs.length() > 0) foundIDs += ",";
      foundIDs += String(id);
    }
  }
  if (foundIDs.length() > 0) discoveredIDs = foundIDs;

  Serial.setTimeout(5); // 5ms timeout padding for stable block ingestion
}

void loop() {
  // 1. Ingest pristine frames from the Pi at 115,200 baud
  if (Serial.available() > 0) {
    uint8_t buffer[64];
    int bytesRead = Serial.readBytes(buffer, sizeof(buffer));
    
    if (bytesRead > 0) {
      digitalWrite(MAX485_DE, HIGH); // Flip to transmit
      
      // Blast them locally down the copper bus lines at 1,000,000 baud
      Serial1.write(buffer, bytesRead);
      Serial1.flush();               
      
      delayMicroseconds(5);          
      digitalWrite(MAX485_DE, LOW);   // Drop back to listen mode immediately
      
      hostBytesCount += bytesRead;
    }
  }
  
  // 2. Return response frames up to the Pi at 115,200 baud
  while (Serial1.available() > 0) {
    Serial.write(Serial1.read());
    servoBytesCount++;             
  }

  if (millis() - lastOledUpdate >= oledInterval) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("IDs: "); display.println(discoveredIDs);
    display.setCursor(0, 12);
    display.print("USB:115k BUS:1M");
    display.setCursor(0, 24);
    display.print("H:"); display.print(hostBytesCount);
    display.print("  S:"); display.print(servoBytesCount);
    display.display();
    lastOledUpdate = millis();
  }
}
