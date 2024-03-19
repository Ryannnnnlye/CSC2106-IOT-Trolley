#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

// This is beacon A

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#define BEACON_A 2
#define TROLLEY_ID 5

#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define OLED_RESET -1

struct LoRaMessage {
  uint8_t trolleyId;
  uint8_t data[32];
};

void (*resetFunc)(void) = 0;

uint8_t numReceived = 0;
int16_t totalRSSI = 0;

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

  // The default transmitter power is 13dBm, using PA_BOOST.
  rf95.setTxPower(13, false);
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    // Receive broadcast from trolley
    if (rf95.recv(buf, &len)) {
      LoRaMessage* receivedMessage = (LoRaMessage*)buf;

      if (receivedMessage->trolleyId == TROLLEY_ID) {
        numReceived++;
        totalRSSI += rf95.lastRssi();

        Serial.print("Received from trolley (");
        Serial.print(numReceived);
        Serial.print(" times) The RSSI is: ");
        Serial.println(rf95.lastRssi(), DEC);

        // Check if 10 messages have been received
        if (numReceived == 10) {
          // Calculate the mean RSSI
          float meanRssi = (totalRSSI) / numReceived;

          Serial.print("Mean RSSI (2 DP): ");
          Serial.println(meanRssi, 2);
          Serial.println("");

          // Reset the variables
          numReceived = 0;
          totalRSSI = 0;
        }
      }
    } else {
      Serial.println("Receive failed");
    }
  }
}