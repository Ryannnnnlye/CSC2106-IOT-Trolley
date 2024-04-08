#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

// This is lora server

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

#define SERVER_ID 1
#define STARTING_BEACON_ID 2

// Used to determine the source to minus from to get the index
#define STARTING_TROLLEY_ID 10
#define TROLLEY_ID 10
#define NO_OF_BEACONS 4
#define NO_BEACONS_NEEDED_FOR_TRILATERATION 3
#define NO_OF_TROLLEY 1

#define RF95_FREQ 920.0
// #define SENDING_FREQUENCY 920.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define OLED_RESET -1

typedef struct {
  uint8_t trolleyId;
  uint8_t beaconId;
  float meanRSSI;
} LoRaMessage;

typedef struct {
  uint8_t trolleyId;
  uint8_t isLock;
} serverMessage;

struct TrolleyTable {
  float rssi[NO_OF_BEACONS]; // that index of the rssi is the index of the beacon
};

const uint8_t sharedKey[] = {0xAB, 0xCD, 0xEF, 0x12, 0x34}; // Example key, replace with your own

TrolleyTable trolleyTable[NO_OF_TROLLEY];

void (*resetFunc)(void) = 0;

uint8_t numReceived = 0;
int16_t totalRSSI = 0;

// A B C D
uint8_t location[NO_OF_BEACONS][2] = {
  {12,15},
  {1,16},
  {1,1},
  {12,0}
};
double target[3]; //Array for target node RSSI or distance
double target_1m[4] = {-58.5, -54.0, -55.9, -56.0}; //Navigation node RSSI_1m constant for each beacon
double target_Cpl[4] = {2.2, 2.2, 2.2, 2.2}; //Target node path loss constant for each beacon

void xorEncryptDecrypt(uint8_t *data, size_t len, const uint8_t *key, size_t keyLen) {
  for (size_t i = 0; i < len; ++i) {
    data[i] ^= key[i % keyLen];
  }
}

//Function to convert RSSI to distance
double to_distance(double input_rssi, int one_m_power, int path_loss_constant){ //RSSI to convert, 1m Power, Path Loss Constant
  double output = pow(10,(one_m_power - input_rssi)/(10*path_loss_constant));
  return output;
}

//Function to trilaterate the 3 distances into the node x,y position
bool trilaterate(uint8_t serverId1, double dist1, uint8_t serverId2, double dist2, uint8_t serverId3, double dist3){
  uint8_t x1 = location[serverId1][0];
  uint8_t y1 = location[serverId1][1];
  uint8_t x2 = location[serverId2][0];
  uint8_t y2 = location[serverId2][1];
  uint8_t x3 = location[serverId3][0];
  uint8_t y3 = location[serverId3][1];
  float r1 = dist1;
  float r2 = dist2;
  float r3 = dist3;
  float AB = sqrt(pow((x1-x2),2)+pow((y1-y2),2))* 60 /100;
  float BC = sqrt(pow((x3-x2),2)+pow((y3-y2),2))* 60 /100;
  float AC = sqrt(pow((x1-x3),2)+pow((y1-y3),2))* 60 /100;

  // Check if (x, y) is inside the triangle ABC
    float sp = (AB + BC + AC)/2; // semi perimeter
    float areaABC = sqrt((sp)*(sp-AB)*(sp-BC)*(sp-AC));

    float spABP = (AB + r1 + r2)/2; // semi perimeter
    float areaABP = sqrt((spABP)*(spABP-AB)*(spABP-r1)*(spABP-r2));
    Serial.println(areaABP);

    float spBCP = (BC + r3 + r2)/2; // semi perimeter
    float areaBCP = sqrt((spBCP)*(spBCP-BC)*(spBCP-r3)*(spBCP-r2));
    Serial.println(areaBCP);
    
    float spACP = (AC + r1 + r3)/2; // semi perimeter
    float areaACP = sqrt((spACP)*(spACP-AC)*(spACP-r1)*(spACP-r3));
    Serial.println(areaACP);

    if (areaABP > areaABC  areaACP > areaABC  areaBCP > areaABC){
      return 0;
    }

    int checklimit = 0;
    // Check for NaN and replace with average
    if (areaABP == 0.0) {
        areaABP = (areaBCP + areaACP) / 2.0;
        checklimit++;
    }
    if (areaBCP == 0.0) {
        areaBCP = (areaABP + areaACP) / 2.0;
        checklimit++;
    }
    if (areaACP == 0.0) {
        areaACP = (areaABP + areaBCP) / 2.0;
        checklimit++;
    }
    if (checklimit >= 2)
      return 0;
    Serial.print("total area: ");
    Serial.print((areaABP + areaACP + areaBCP));
    
    Serial.print("areaABC: ");
    Serial.println(areaABC);

    Serial.println("\n");

    float tolerance = 2.0; // Tolerance for area difference
    bool insideTriangle = abs(areaABP+ areaACP + areaBCP - areaABC) <= tolerance;
    return insideTriangle;
}

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
  Serial.println("Setup complete");
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    // Receive broadcast from trolley
    if (rf95.recv(buf, &len)) {
      // Decrypt the received message
      xorEncryptDecrypt(buf, len, sharedKey, sizeof(sharedKey));
      LoRaMessage packet;
      memcpy(&packet, buf, sizeof(LoRaMessage));
      Serial.print("Update beacon ID: ");
      Serial.println(packet.beaconId);

      // Decrypt the received message
      int8_t receivedTrolleyId = packet.trolleyId;
      int8_t receivedTrolleyIndex = receivedTrolleyId - STARTING_TROLLEY_ID;
      // If trolley found
      if (receivedTrolleyIndex >= 0 && receivedTrolleyIndex <= NO_OF_TROLLEY) {
        // Starting beacon index is 2, so we minus to get index [0] for beacon A, [1] for beacon B and so on
        int beaconIndex = packet.beaconId - STARTING_BEACON_ID;
        trolleyTable[receivedTrolleyIndex].rssi[beaconIndex] = packet.meanRSSI;
      }

      // Print trolleyTable data
      for (int i = 0; i < NO_OF_TROLLEY; i++) {
        Serial.println("Trolley ID: " + String(STARTING_TROLLEY_ID + i));
        for (int j = 0; j < NO_OF_BEACONS; j++) {
          Serial.print("Beacon ");
          Serial.print((char)('A' + j));  // Convert index to beacon letter
          Serial.print(" RSSI: ");
          Serial.println(trolleyTable[i].rssi[j]);
        }
        Serial.println();
      }

      // Find when all 3 beacon's RSSI value are filled
      uint8_t allowableZeroForTrilateration = NO_OF_BEACONS - NO_BEACONS_NEEDED_FOR_TRILATERATION + 1;
      for (int i = 0; i < NO_OF_BEACONS; i++) {
        if (trolleyTable[receivedTrolleyIndex].rssi[i] == 0.0) { // is not filled
          allowableZeroForTrilateration--;
          if (allowableZeroForTrilateration == 0) {
            Serial.println("All RSSI values are not filled");
            break;
          }
        }
      }

      if (allowableZeroForTrilateration != 0) {
        uint8_t recordedIndexBeacons[NO_BEACONS_NEEDED_FOR_TRILATERATION];
        Serial.println("All RSSI values filled");
        // Calculate trilateration
        for(int i=0; i<NO_OF_BEACONS; i++){
          if (trolleyTable[receivedTrolleyIndex].rssi[i] == 0.0){
            continue; // Skip if RSSI value is 0
          }
          recordedIndexBeacons[i] = i;
          target[i] = to_distance(trolleyTable[receivedTrolleyIndex].rssi[i], target_1m[i], target_Cpl[i]);

          Serial.print("target");
          Serial.print(i);
          Serial.print(": ");
          Serial.println(target[i]);
        } //Convert target RSSI to distances

        serverMessage msgOut;
        msgOut.trolleyId = packet.trolleyId;

        Serial.print("lock?");
        if (trilaterate(target[recordedIndexBeacons[0]], target[0], target[recordedIndexBeacons[1]], target[1], target[recordedIndexBeacons[2]], target[2])){
          Serial.println("------------------------------------Unlock");
          msgOut.isLock = 0;
        } else {
          Serial.println("------------------------------------Lock");
          msgOut.isLock = 1;
        }

        rf95.setFrequency(923.0); // to trolley
        delay(100);
        xorEncryptDecrypt((uint8_t*)&msgOut, sizeof(msgOut), sharedKey, sizeof(sharedKey));
        for (int i=0; i<5; i++){rf95.send((uint8_t*)&msgOut, sizeof(msgOut));
          delay(100);
        }
        // rf95.setFrequency(newFrequency); 
        rf95.setFrequency(920.0); 

        // Reset RSSI vaues for the trolley
        for (int i = 0; i < NO_OF_BEACONS; i++) {
          trolleyTable[receivedTrolleyIndex].rssi[i] = 0.0;
        }
      }

      // Reset packet data
      packet.trolleyId = 0;
      packet.beaconId = 0;
      packet.meanRSSI = 0;

    } else {
      Serial.println(F("Receive failed"));
    }
  } else {
    // Serial.println(F("RF95 not avaialble"));
  }
}