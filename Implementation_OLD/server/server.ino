
#include <RHReliableDatagram.h>  //get it here http://lowpowerlab.com/RadioHead_LowPowerLab.zip
#include <RH_RF95.h>  //get it here http://lowpowerlab.com/RadioHead_LowPowerLab.zip
#include <SPI.h> //get it here: https://www.github.com/lowpowerlab/spiflash

#define SERVER_ADDRESS 1 // address number of the Server
#define CLIENT_ADDRESS_2 2 // address number of the Client 
#define CLIENT_ADDRESS_3 3 // address number of the Client 
#define CLIENT_ADDRESS_4 4 // address number of the Client 
#define TROLLEY_ADDRESS 5 // address number of the Client 
#define RETRIES 4 // Number of times the sendtoWait() will try to send a message. Default is 3
#define TIMEOUT 2000 // Timeout before sendWait() tries again to send a message 

// Match frequency to the hardware version of the radio on your Moteino
#define FREQUENCY 925

// Singleton instance of the radio driver
RH_RF95 driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

uint8_t key_table = 0;
uint8_t value = 0;

// create a struct data with a parent_id and a list of child_id without the need to declare the size of the list
struct data_received {
  uint8_t beacon_id;
  uint8_t trolley_id;
  float rssi; // average rssi value
};

struct trolley_to_beacon {
  uint8_t beacon_id = 0;
  float rssi = 0; // average rssi value
};

// struct for x,y coordinates of the trolley
struct Point {
    float x;
    float y;
};

// create array of struct
// trolley_to_beacon trolley_to_beacon_data[];

struct trolley_table {
  uint8_t trolley_id = 5;
  trolley_to_beacon data[3];
};

trolley_table trolley_table_data[2];

void add_if_exist(data_received* data){
  for (int i = 0; i < 2; i++){
    // print type and value
    Serial.println(trolley_table_data[i].trolley_id);
    Serial.println(data->trolley_id);

    if (trolley_table_data[i].trolley_id == data->trolley_id){
      for (int j = 0; j < 3; j++){
        // if = 0 means no data insert yet. thus replace it with the new data
        if (trolley_table_data[i].data[j].beacon_id == data->beacon_id){ // if same beacon id, replace the rssi value
          trolley_table_data[i].data[j].rssi = data->rssi;
          key_table = j; // if return j is 2, means last data added.
          value = i;
          return;
        }
        if (trolley_table_data[i].data[j].beacon_id == 0){
          trolley_table_data[i].data[j].beacon_id = data->beacon_id;
          trolley_table_data[i].data[j].rssi = data->rssi;
          key_table = j; // if return j is 2, means last data added.
          value = i;
          return; 
        }
      }
    }
  }
};

Point trilateration(float distance1, float distance2, float distance3){
  //do the formula here
  //return x and y coordinates
  float x2 = 1.5;
  float x3 = 1;
  float y3 = 1.5;

  float xPos = (pow(distance1, 2) - pow(distance2, 2) + pow(x2, 2)) / (2 * x2);
  float yPos = (pow(distance1, 2) - pow(distance3, 2) + pow(x3, 2) + pow(y3, 2) - (2 * x3 * xPos)) / (2 * y3);

  Point point;
  point.x = xPos;
  point.y = yPos; 
  return point;
}


bool check_trili(float distance1, float distance2, float distance3, float pathloss){
  // if (distance1 < 0 || distance2 < 0 || distance3 < 0 || pathloss < 0){
  //   return false;
  // }

  // calculate trilateration of the trolley
  trilateration(distance1, distance2, distance3);

  // include geofencing stuff here also
  
  
  return true;
};

float calculate_distance(float rssi, uint8_t beacon_id, float pathloss){
  //Measured power: RSSI value of beacon to trolley at 1 meter
  int measured_power = 0;
  if (beacon_id == 2) {
    measured_power = -44  ;
  }
  else if (beacon_id == 3) {
    measured_power = -46;
  }
  else if (beacon_id == 4) {
    measured_power = -45;
  }

  return pow(10, ((measured_power - rssi) / (10 * pathloss)));
};

float estPathloss() {
    // int destination = 3;  // reference anchor node address (B)
    // float avgRSSI = 0.0;

    // float A0 = -59;  // TODO: tune this
    // float d = 1.5;   // TODO: tune this (in meters)
    

    // // Simulated RF module settings
    // int ack_retries = 1;
    // float ack_wait = 0.1;
    // float ack_delay = 0;

    // std::vector<float> pathLossArr;

    // for (int i = 0; i < 5; ++i) {
    //     // Simulated send_with_ack function
    //     bool sendSuccess = send_with_ack("pLoss");
    //     if (!sendSuccess) {
    //         std::cout << "pLoss failed" << std::endl;
    //     }

    //     // Simulated receive function
    //     float last_rssi = receive_ack();
    //     if (last_rssi != 0) {
    //         pathLossArr.push_back(last_rssi);
    //         std::cout << "pathloss ping: " << last_rssi << std::endl;
    //     }

    //     // Simulated delay
    //     delay(500)
    // }

    // destination = 1;  // reset destination
    // ack_retries = 2;
    // ack_wait = 0.2;
    // delay(500)

    // if (!pathLossArr.empty()) {
    //     float sum = 0;
    //     for (auto &value : pathLossArr) {
    //         sum += value;
    //     }
    //     avgRSSI = sum / pathLossArr.size();
    // }

    // std::cout << "num " << -avgRSSI + A0 << std::endl;
    // std::cout << "denom " << -10 * log10(d) << std::endl;
    // float pathLoss = (-avgRSSI + A0) / (-10 * log10(d));

    // return pathLoss;
    return 2;
};

void setup() 
{
   // connect to the serial monitor at 115200 baud
  Serial.begin(115200);
  Serial.println("DATAGRAM TEST - CLIENT");
  Serial.println("");
  
  //Initialize the radio
  if (!manager.init())
  {
    Serial.println("Init failed!");
  delay(500);
    setup();
  }
    else
  {
    Serial.print("Init OK - ");
    Serial.println(FREQUENCY); Serial.print("mhz");
    driver.setFrequency(FREQUENCY);
  }

  manager.setRetries(RETRIES);
  manager.setTimeout(TIMEOUT);

};

// declare data struct with own id added inside
// struct data data_struct_temp;

// create error data msg back to sender
uint8_t data[] = "ACK";

int beacon_priority[3] = {2, 3, 4};

void cycleQueue() {
  int first_element = beacon_priority[0];

  for (int i = 0; i < 2; i++) {
    beacon_priority[i] = beacon_priority[i + 1];
  }

  beacon_priority[2] = first_element;
}

void loop()
{
  // Serial.println("working");
  // uint8_t len = sizeof(buf);
  // uint8_t from;
  if (manager.sendtoWait(data, sizeof(data), beacon_priority[0])){
      // Wait for a message addressed to us from the client

    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAckTimeout(buf, &len, 10000, &from))
    {
      Serial.println("testing");
      data_received* receivedMessage = (data_received*)buf;
      
      // Serial.println(receivedMessage->trolley_id);
      // Serial.println(receivedMessage->beacon_id);
      // Serial.println(receivedMessage->rssi);

      add_if_exist(receivedMessage);
      // print out trolley_table_data [0] see data
      Serial.println("data shown:");
      Serial.println(trolley_table_data[0].trolley_id);
      Serial.println(trolley_table_data[0].data[0].beacon_id);
      Serial.println(trolley_table_data[0].data[0].rssi);
      Serial.println(trolley_table_data[0].data[1].beacon_id);
      Serial.println(trolley_table_data[0].data[1].rssi);
      Serial.println(trolley_table_data[0].data[2].beacon_id);
      Serial.println(trolley_table_data[0].data[2].rssi);

      if(key_table == 2){
        Serial.println("last data added"); // full data obtained

        // float pathloss = estPathloss();
        float pathloss = 2.25;

        float dist1 = calculate_distance(trolley_table_data[0].data[0].rssi, trolley_table_data[0].data[0].beacon_id, pathloss);
        float dist2 = calculate_distance(trolley_table_data[0].data[1].rssi, trolley_table_data[0].data[1].beacon_id, pathloss);
        float dist3 = calculate_distance(trolley_table_data[0].data[2].rssi, trolley_table_data[0].data[2].beacon_id, pathloss);
        Serial.println("dist");
        Serial.println(dist1);
        Serial.println(dist2);
        Serial.println(dist3);
        
        // true means to lock
        // false dont do anything
        if (check_trili(dist1, dist2, dist3, pathloss)){ 
          // send ack to trolley
          if (!manager.sendtoWait((uint8_t*)&data, sizeof(data), TROLLEY_ADDRESS)){
            Serial.println("sendtoWait failed");
            // need delete the data in the datatable so that it can be added again
            // for (int i = 0; i < sizeof(trolley_table_data) / sizeof(trolley_table_data[0]); i++) {
            //   if (trolley_table_data[i].trolley_id == value) {
            //     trolley_table_data[i].trolley_id = 0;
            //     trolley_table_data[i].data[0].beacon_id = 0;
            //     trolley_table_data[i].data[1].beacon_id = 0;
            //     trolley_table_data[i].data[2].beacon_id = 0;
            //     trolley_table_data[i].data[0].rssi = 0;
            //     trolley_table_data[i].data[1].rssi = 0;
            //     trolley_table_data[i].data[2].rssi = 0;
            //     break;
            //   }
            // }
            
          } else {
            key_table = 0;
            value = 0;
            trolley_table_data[value].trolley_id = 5;
            trolley_table_data[value].data[0].beacon_id = 0;
            trolley_table_data[value].data[1].beacon_id = 0;
            trolley_table_data[value].data[2].beacon_id = 0;
            trolley_table_data[value].data[0].rssi = 0.00;
            trolley_table_data[value].data[1].rssi = 0.00;
            trolley_table_data[value].data[2].rssi = 0.00;
            Serial.println("data cleared:");
            Serial.println(trolley_table_data[0].trolley_id);
            Serial.println(trolley_table_data[0].data[0].beacon_id);
            Serial.println(trolley_table_data[0].data[0].rssi);
            Serial.println(trolley_table_data[0].data[1].beacon_id);
            Serial.println(trolley_table_data[0].data[1].rssi);
            Serial.println(trolley_table_data[0].data[2].beacon_id);
            Serial.println(trolley_table_data[0].data[2].rssi);
          }

        }
      }
    } 
    cycleQueue();
  }
  else {
    Serial.println("failed");
  }

  // if (manager.available())
  // {
  //   // Wait for a message addressed to us from the client
  //   uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

  //   uint8_t len = sizeof(buf);
  //   uint8_t from;
  //   if (manager.recvfromAck(buf, &len, &from))
  //   {
  //     if (!manager.sendtoWait(data, sizeof(data), from)){
  //       Serial.println("sendtoWAit failed");
  //     }
  //     data_received* receivedMessage = (data_received*)buf;
      
  //     // Serial.println(receivedMessage->trolley_id);
  //     // Serial.println(receivedMessage->beacon_id);
  //     // Serial.println(receivedMessage->rssi);

  //     add_if_exist(receivedMessage);
  //     // print out trolley_table_data [0] see data
  //     Serial.println("data shown:");
  //     Serial.println(trolley_table_data[0].trolley_id);
  //     Serial.println(trolley_table_data[0].data[0].beacon_id);
  //     Serial.println(trolley_table_data[0].data[0].rssi);
  //     Serial.println(trolley_table_data[0].data[1].beacon_id);
  //     Serial.println(trolley_table_data[0].data[1].rssi);
  //     Serial.println(trolley_table_data[0].data[2].beacon_id);
  //     Serial.println(trolley_table_data[0].data[2].rssi);

  //     if(key_table == 2){
  //       Serial.println("last data added"); // full data obtained

  //       // float pathloss = estPathloss();
  //       float pathloss = 2.25;

  //       float dist1 = calculate_distance(trolley_table_data[0].data[0].rssi, trolley_table_data[0].data[0].beacon_id, pathloss);
  //       float dist2 = calculate_distance(trolley_table_data[0].data[1].rssi, trolley_table_data[0].data[1].beacon_id, pathloss);
  //       float dist3 = calculate_distance(trolley_table_data[0].data[2].rssi, trolley_table_data[0].data[2].beacon_id, pathloss);
  //       Serial.println("dist");
  //       Serial.println(dist1);
  //       Serial.println(dist2);
  //       Serial.println(dist3);
        

  //       // true means to lock
  //       // false dont do anything
  //       if (check_trili(dist1, dist2, dist3, pathloss)){ 
  //         // send ack to trolley
  //         if (!manager.sendtoWait((uint8_t*)&data, sizeof(data), TROLLEY_ADDRESS)){
  //           Serial.println("sendtoWait failed");
  //           // need delete the data in the datatable so that it can be added again
  //           // for (int i = 0; i < sizeof(trolley_table_data) / sizeof(trolley_table_data[0]); i++) {
  //           //   if (trolley_table_data[i].trolley_id == value) {
  //           //     trolley_table_data[i].trolley_id = 0;
  //           //     trolley_table_data[i].data[0].beacon_id = 0;
  //           //     trolley_table_data[i].data[1].beacon_id = 0;
  //           //     trolley_table_data[i].data[2].beacon_id = 0;
  //           //     trolley_table_data[i].data[0].rssi = 0;
  //           //     trolley_table_data[i].data[1].rssi = 0;
  //           //     trolley_table_data[i].data[2].rssi = 0;
  //           //     break;
  //           //   }
  //           // }
            
  //         } else {
  //           key_table = 0;
  //           value = 0;
  //           trolley_table_data[value].trolley_id = 5;
  //           trolley_table_data[value].data[0].beacon_id = 0;
  //           trolley_table_data[value].data[1].beacon_id = 0;
  //           trolley_table_data[value].data[2].beacon_id = 0;
  //           trolley_table_data[value].data[0].rssi = 0.00;
  //           trolley_table_data[value].data[1].rssi = 0.00;
  //           trolley_table_data[value].data[2].rssi = 0.00;
  //           Serial.println("data cleared:");
  //           Serial.println(trolley_table_data[0].trolley_id);
  //           Serial.println(trolley_table_data[0].data[0].beacon_id);
  //           Serial.println(trolley_table_data[0].data[0].rssi);
  //           Serial.println(trolley_table_data[0].data[1].beacon_id);
  //           Serial.println(trolley_table_data[0].data[1].rssi);
  //           Serial.println(trolley_table_data[0].data[2].beacon_id);
  //           Serial.println(trolley_table_data[0].data[2].rssi);
  //         }

  //       }
  //     }
  //   }
  // }

  // Serial.println("Sending a message to the SERVER... ");
    
  // // Send a message to manager_server
  // if (manager.sendtoWait((uint8_t*)&data, sizeof(data), SERVER_ADDRESS))
  // {
  //   Serial.println("Message sent to SERVER");
  //   uint8_t len = sizeof(buf);
  //   uint8_t from;   
  //   // Now wait for a reply from the server
  //   if (manager.recvfromAckTimeout(buf, &len, 500, &from))
  //   {
  //     Serial.print("Got reply from server ID ");
  //     Serial.print(from);
  //     Serial.print(": ");
  //     Serial.println((char*)buf);
  //     Serial.println("");
  //   }
  //   else
  //   {
  //     Serial.println("No reply, is SERVER running?");
  //   }
  // }
  // else{
  //   Serial.println("Transmission of data failed!");
  // }

  // Serial.print("End of Loop");
  delay(500);
};