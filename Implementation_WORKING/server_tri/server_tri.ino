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
#define BEACON_A 2
#define BEACON_B 3
#define BEACON_C 4
#define STARTING_BEACON_ID 2

// Used to determine the source to minus from to get the index
#define STARTING_TROLLEY_ID 5
#define TROLLEY_ID 5
#define NO_OF_BEACONS 3
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
  float rssi[NO_OF_BEACONS];
};

TrolleyTable trolleyTable[NO_OF_TROLLEY];

void (*resetFunc)(void) = 0;

uint8_t numReceived = 0;
int16_t totalRSSI = 0;

// A B C
double locationA[2] = {6,10}; //Location of beacon 1
double locationB[2] = {5,1}; //Location of beacon 2
double locationC[2] = {1,5}; //Location of beacon 3
double target[3]; //Array for target node RSSI or distance
double target_1m[3] = {-56.0, -60.4, -56.5}; //Navigation node RSSI_1m constant for each beacon
double target_Cpl[3] = {2.2, 2.2, 2.2}; //Target node path loss constant for each beacon

//Function to convert RSSI to distance
double to_distance(double input_rssi, int one_m_power, int path_loss_constant){ //RSSI to convert, 1m Power, Path Loss Constant
  double output = pow(10,(one_m_power - input_rssi)/(10*path_loss_constant));
  return output;
}

//Function to trilaterate the 3 distances into the node x,y position
bool trilaterate(double dist1, double dist2, double dist3){
  float x1 = locationA[0];
  float y1 = locationA[1];
  float x2 = locationB[0];
  float y2 = locationB[1];
  float x3 = locationC[0];
  float y3 = locationC[1];
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
    float areaABP = sqrt((spABP)*(spABP-AB)*abs(spABP-r1)*abs(spABP-r2));
    Serial.println(areaABP);

    float spBCP = (BC + r3 + r2)/2; // semi perimeter
    float areaBCP = sqrt((spBCP)*(spBCP-BC)*abs(spBCP-r3)*abs(spBCP-r2));
    Serial.println(areaBCP);
    
    float spACP = (AC + r1 + r3)/2; // semi perimeter
    float areaACP = sqrt((spACP)*(spACP-AC)*abs(spACP-r1)*abs(spACP-r3));
    Serial.println(areaACP);

    if (areaABP > areaABC || areaACP > areaABC || areaBCP > areaABC){
      return 0;
    }

    int checklimit = 0;
    // Check for NaN and replace with average
    if (areaABP == 0) {
        areaABP = (areaBCP + areaACP) / 2.0;
        checklimit++;
    }
    if (areaBCP == 0) {
        areaBCP = (areaABP + areaACP) / 2.0;
        checklimit++;
    }
    if (areaACP == 0) {
        areaACP = (areaABP + areaBCP) / 2.0;
        checklimit++;
    }
    if (checklimit >= 2)
      return 0;
    Serial.println("total area: ");
    Serial.print((areaABP + areaACP + areaBCP));
    
    Serial.println("areaABC: ");
    Serial.print(areaABC);

    Serial.println("\n");

    float tolerance = 2.0; // Tolerance for area difference
    bool insideTriangle = abs(areaABP + areaACP + areaBCP - areaABC) <= tolerance;
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
      LoRaMessage packet;
      memcpy(&packet, buf, sizeof(LoRaMessage));

      // Serial.println("received trolley id: ");
      // Serial.println(packet.trolleyId);
      Serial.print("Update beacon ID: ");
      Serial.println(packet.beaconId);
      // Serial.print("Received rssi : ");
      // Serial.println(packet.meanRSSI);

      int8_t receivedTrolleyId = packet.trolleyId;
      int8_t receivedTrolleyIndex = receivedTrolleyId - STARTING_TROLLEY_ID;
      // If trolley found
      if (receivedTrolleyIndex >= 0 && receivedTrolleyIndex <= NO_OF_TROLLEY) {
        // Starting beacon index is 2, so we minus to get index [0] for beacon A, [1] for beacon B and so on
        int beaconIndex = packet.beaconId - STARTING_BEACON_ID;
        trolleyTable[receivedTrolleyIndex].rssi[beaconIndex] = packet.meanRSSI;
      }

      // Reset packet data
      packet.trolleyId = 0;
      packet.beaconId = 0;
      packet.meanRSSI = 0;

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
      bool allRSSIFilled = true;
      for (int i = 0; i < NO_OF_BEACONS; i++) {
        if (trolleyTable[receivedTrolleyIndex].rssi[i] == 0.0) {
          allRSSIFilled = false;
          break;
        }
      }

      if (allRSSIFilled) {
        Serial.println("All RSSI values filled");
        // Calculate trilateration
        for(int i=0; i<3; i++){target[i] = to_distance(trolleyTable[receivedTrolleyIndex].rssi[i],target_1m[i], target_Cpl[i]);} //Convert target RSSI to distances
        for(int i=0; i<3; i++){
          Serial.print("target");
          Serial.print(i);
          Serial.print(": ");
          Serial.println(target[i]);
        }

        serverMessage msgOut;
        msgOut.trolleyId = packet.trolleyId;

        Serial.print("lock?");
        if (trilaterate(target[0], target[1], target[2])){
          Serial.print("Unlock");
          msgOut.isLock = 0;
        } else {
          Serial.print("Lock");
          msgOut.isLock = 1;
        }

        rf95.setFrequency(923.0);
        delay(100);
        rf95.send((uint8_t*)&msgOut, sizeof(msgOut));
        rf95.setFrequency(920.0);

        // Reset RSSI vaues for the trolley
        for (int i = 0; i < 3; i++) {
          trolleyTable[receivedTrolleyIndex].rssi[i] = 0.0;
        }
      }


    } else {
      Serial.println(F("Receive failed"));
    }
  } else {
    // Serial.println(F("RF95 not avaialble"));
  }
}