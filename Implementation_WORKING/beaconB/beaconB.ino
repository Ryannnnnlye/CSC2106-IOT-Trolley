#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

// This is beacon B

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#define BEACON_ID 3
#define TROLLEY_ID 5
#define SERVER_ID 1
#define numberOfTrollies 10 

#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define OLED_RESET -1

typedef struct {
  uint8_t trolleyId;
  uint8_t beaconId;
  float meanRSSI;
} LoRaMessage;


uint8_t numReceivedIndex[numberOfTrollies] = {0};
int16_t rssitotal[numberOfTrollies] = {0};

void (*resetFunc)(void) = 0;

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

    // Set back 
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    // Receive broadcast from trolley
    if (rf95.recv(buf, &len)) {

      LoRaMessage packet;
      memcpy(&packet, buf, sizeof(LoRaMessage));

      // increase the packet count for that trolley
      numReceivedIndex[packet.trolleyId]++; 

      // increase rssi for that trolley
      rssitotal[packet.trolleyId] += rf95.lastRssi();

      Serial.print("The trolley id is ");
      Serial.print(packet.trolleyId);
      Serial.println();
      Serial.print(F("Received from trolley ("));
      Serial.print(numReceivedIndex[packet.trolleyId]);
      Serial.print(F(" times) The total RSSI is: "));
      Serial.println(rssitotal[packet.trolleyId]);

        // Check if 10 messages have been received
        if (numReceivedIndex[packet.trolleyId] == 10) {
          float meanRSSI = static_cast<float>(rssitotal[packet.trolleyId]) / static_cast<float>(numReceivedIndex[packet.trolleyId]);
         
          packet.meanRSSI = meanRSSI;
          packet.beaconId = BEACON_ID;

          Serial.print(F("Mean RSSI (2 DP): "));
          Serial.println(packet.meanRSSI);
          Serial.print(F("Trolley ID: "));
          Serial.println(packet.trolleyId);
          Serial.println(F(""));

          // Change frequncy to prevent potential congestion
          rf95.setFrequency(920.0);
          delay(100);
          rf95.send((uint8_t*)&packet, sizeof(packet));
          rf95.setFrequency(923.0);

          // Reset the variables
          numReceivedIndex[packet.trolleyId] = 0;
          rssitotal[packet.trolleyId] = 0; // O(1)
          delay(1000);
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
