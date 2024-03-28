#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// This is the trolley

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#define TROLLEY_ID 5

#define RF95_FREQ 923.0

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

typedef struct {
  uint8_t trolleyId;
  uint8_t isLock;
} serverMessage;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void (*resetFunc)(void) = 0;  //declare reset function at address 0

typedef struct {
  uint8_t trolleyId;
} packet;

void setup() {

  Serial.begin(19200);
  delay(100);

  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println(F("LoRa radio init failed"));

    while (1)
      ;
  }

  // Defaults after init are 915.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(915)) {
    Serial.println(F("setFrequency failed"));
    while (1)
      ;
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address or 0x3D for
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) {
      delay(1000);
    }
  }
  // Setup oled display
  display.setTextSize(1);       // Normal 1:1 pixel scale
  display.setTextColor(WHITE);  // Draw white text
  display.setCursor(0, 0);      // Start at top-left corner

  // Simple text
  display.clearDisplay();
  display.println("Powered on");
  display.display();

  rf95.setTxPower(13, false);
  rf95.setFrequency(923.0);

  Serial.println("Done");
}

void loop() {
  if (rf95.waitAvailableTimeout(1000))
    Serial.println("rf95");
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.recv(buf, &len)) {
    serverMessage packet;
    memcpy(&packet, buf, sizeof(serverMessage));
    Serial.println(packet.trolleyId);
    if (packet.trolleyId == TROLLEY_ID) {
      if (packet.isLock == 1) {
        display.clearDisplay();
        display.println("Lock Wheels!");
        Serial.println("Lock Wheels");
        // Set cursor position
        display.setCursor(0, 0);
        display.display();
      } else {
        display.clearDisplay();
        display.println("Unlock Wheels!");
        // Set cursor position
        Serial.println("Unlock Wheels");
        display.setCursor(0, 0);
        display.display();
      }
    } else {
      Serial.print("Trolley ID not found: ");
      Serial.println(packet.trolleyId);
    }
    // delay(15000);
  } else {
    packet message;
    message.trolleyId = TROLLEY_ID;
    // Send the trolley id to the beacon
    rf95.send((uint8_t*)&message, sizeof(message));
    rf95.waitPacketSent();
    Serial.println("packet sent");
  }
}
