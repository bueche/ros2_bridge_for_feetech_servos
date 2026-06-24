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

// ==========================================
// CONFIGURATION: Set your desired Servo ID here
// ==========================================
const uint8_t TARGET_ID = 6; 

// Low-level function to verify if the newly assigned ID answers on the bus
bool verifyNewID(uint8_t id) {
  // Clear any leftover bytes in the buffer
  while(Serial1.available()) Serial1.read();
  
  // Format standard native PING packet for the new target ID
  uint8_t packet[6] = {0xFF, 0xFF, id, 0x02, 0x01, (uint8_t)~(id + 0x02 + 0x01)};
  
  digitalWrite(MAX485_DE, HIGH); // Transmit Mode
  Serial1.write(packet, 6);
  Serial1.flush();
  delayMicroseconds(5);
  digitalWrite(MAX485_DE, LOW);  // Listen Mode

  // Wait briefly (up to 10ms) for the 6-byte response frame to return
  delay(10); 
  
  if (Serial1.available() >= 6) {
    if (Serial1.read() == 0xFF && Serial1.read() == 0xFF) {
      while(Serial1.available()) Serial1.read(); // Flush out the remaining status payload
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
    display.println("ID PROGRAMMER + PING");
    display.print("Target ID: "); display.println(TARGET_ID);
    display.println("Writing to bus...");
    display.display();
  }

  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);     
  delay(1000); // Allow hardware lines to settle

  // --- STEP 1: UNLOCK EEPROM WRITE PROTECTION ---
  uint8_t unlock_packet[8] = {0xFF, 0xFF, 0xFE, 0x04, 0x03, 55, 0, 0};
  unlock_packet[7] = (uint8_t)~(0xFE + 0x04 + 0x03 + 55 + 0);

  digitalWrite(MAX485_DE, HIGH);
  Serial1.write(unlock_packet, 8);
  Serial1.flush();
  delayMicroseconds(5);
  digitalWrite(MAX485_DE, LOW);
  delay(100);

  // --- STEP 2: WRITE NEW SERVO ID (Register 5) ---
  uint8_t id_packet[8] = {0xFF, 0xFF, 0xFE, 0x04, 0x03, 5, TARGET_ID, 0};
  id_packet[7] = (uint8_t)~(0xFE + 0x04 + 0x03 + 5 + TARGET_ID);

  digitalWrite(MAX485_DE, HIGH);
  Serial1.write(id_packet, 8);
  Serial1.flush();
  delayMicroseconds(5);
  digitalWrite(MAX485_DE, LOW);
  delay(100);

  // --- STEP 3: RELOCK EEPROM FOR SAFETY ---
  uint8_t lock_packet[8] = {0xFF, 0xFF, TARGET_ID, 0x04, 0x03, 55, 1, 0};
  lock_packet[7] = (uint8_t)~(TARGET_ID + 0x04 + 0x03 + 55 + 1);

  digitalWrite(MAX485_DE, HIGH);
  Serial1.write(lock_packet, 8);
  Serial1.flush();
  delayMicroseconds(5);
  digitalWrite(MAX485_DE, LOW);
  delay(200); // Give the EEPROM a window to finish flashing internally

  // --- STEP 4: AUTOMATED VERIFICATION PING ---
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("VERIFYING WRITE...");
  display.display();
  
  bool pingSuccess = verifyNewID(TARGET_ID);

  // --- STEP 5: FINAL TELEMETRY SCREEN UPDATE ---
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Servo is now ID: "); display.println(TARGET_ID);
  display.setCursor(0, 12);
  
  if (pingSuccess) {
    display.println("PING STATUS: SUCCESS");
    display.setCursor(0, 24);
    display.println("[Safe to Unplug]");
  } else {
    display.println("PING STATUS: FAILED!");
    display.setCursor(0, 24);
    display.println("[Check Wires / Retry]");
  }
  display.display();
}

void loop() {
  // Execution complete. Holds state until the board reboots.
}
