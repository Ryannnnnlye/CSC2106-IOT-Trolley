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
#define BEACON_ID 5
#define TROLLEY_ID 10
// #define SERVER_ID 5

#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define OLED_RESET -1

typedef struct {
  uint8_t trolleyId;
  uint8_t beaconId;
  float meanRSSI;
} LoRaMessage;

void (*resetFunc)(void) = 0;

uint8_t numReceived = 0;
int16_t totalRSSI = 0;

// one time function
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
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("setFrequency failed"));
    while (1)
      ;
  }

  // The default transmitter power is 13dBm, using PA_BOOST.
  rf95.setTxPower(13, false);
  rf95.setFrequency(923.0);
}

void loop() {
  if (rf95.available()) {
    Serial.println("Available");
    // Set back 
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    // Receive broadcast from trolley
    if (rf95.recv(buf, &len)) {
      LoRaMessage packet;
      memcpy(&packet, buf, sizeof(LoRaMessage));

      Serial.println(packet.trolleyId);
      if (packet.trolleyId == TROLLEY_ID) {
        numReceived++;
        totalRSSI += rf95.lastRssi();

        Serial.print(F("Received from trolley ("));
        Serial.print(numReceived);
        Serial.print(F(" times) The RSSI is: "));
        Serial.println(rf95.lastRssi());

        // Check if 10 messages have been received
        if (numReceived == 10) {
          // Calculate the mean RSSI
          float meanRSSI = static_cast<float>(totalRSSI) / static_cast<float>(numReceived);
          
          packet.meanRSSI = meanRSSI;
          packet.beaconId = BEACON_ID;

          Serial.print(F("Mean RSSI (2 DP): "));
          Serial.println(packet.meanRSSI);
          Serial.print(F("Beacon ID: "));
          Serial.println(packet.beaconId);
          Serial.println(F(""));

          // Change frequncy to prevent potential congestion
          rf95.setFrequency(920.0);
          delay(100);
          rf95.send((uint8_t*)&packet, sizeof(packet));
          
          rf95.setFrequency(923.0);

          // Reset the variables
          numReceived = 0;
          totalRSSI = 0;

          delay(1000);
        }
      }
    } else {
      Serial.println(F("Receive failed"));
    }
  }
  // } else {
  //   // Serial.println(F("RF95 not avaialble"));
  // }
  delay(500);
}
