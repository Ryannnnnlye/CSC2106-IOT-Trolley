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

#define RF95_FREQ 915.0

#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void (*resetFunc)(void) = 0;  //declare reset function at address 0

typedef struct {
  uint8_t trolleyId;
  uint8_t data;
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
    Serial.println("LoRa radio init failed");

    while (1)
      ;
  }

  // Defaults after init are 915.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }

  rf95.setTxPower(13, false);
}

void loop() {
  Serial.println("Sending to all beacons");

  // Define the package details
  packet message;
  message.trolleyId = TROLLEY_ID;
  message.data = 1;

  rf95.send((uint8_t*)&message, sizeof(message));

  // If send is successful, clear it
  if (rf95.waitPacketSent()) {
    Serial.println("Send successful!");
    Serial.println("");
  }

  delay(1000);
}