
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
#define FREQUENCY 915

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
          return
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


bool check_trili(float distance1, float distance2, float distance3, float pathloss){
  // if (distance1 < 0 || distance2 < 0 || distance3 < 0 || pathloss < 0){
  //   return false;
  // }
  return true;
};

float calculate_distance(float rssi){
  // create txpower 23
  uint8_t txpower = 23;
  uint8_t n = 2.4;

  return pow(10, ((txpower - rssi) / (10 * n)));
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

void loop()
{
  // Serial.println("working");
  // uint8_t len = sizeof(buf);
  // uint8_t from;

  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      data_received* receivedMessage = (data_received*)buf;
      
      Serial.println(receivedMessage->trolley_id);
      Serial.println(receivedMessage->beacon_id);
      Serial.println(receivedMessage->rssi);

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
        float dist1 = calculate_distance(trolley_table_data[0].data[0].rssi);
        float dist2 = calculate_distance(trolley_table_data[0].data[1].rssi);
        float dist3 = calculate_distance(trolley_table_data[0].data[2].rssi);
        float pathloss = estPathloss();

        // true means to lock
        // false dont do anything
        if (check_trili(dist1, dist2, dist3, pathloss)){ 
          // send ack to trolley
          if (!manager.sendtoWait((uint8_t*)&data, sizeof(data), TROLLEY_ADDRESS)){
            Serial.println("sendtoWait failed");
            key_table = 0;
            value = 0;
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
            
            // trolley_table_data[value].trolley_id = 0;
            // trolley_table_data[value].data[0].beacon_id = 0;
            // trolley_table_data[value].data[1].beacon_id = 0;
            // trolley_table_data[value].data[2].beacon_id = 0;
            // trolley_table_data[value].data[0].rssi = 0;
            // trolley_table_data[value].data[1].rssi = 0;
            // trolley_table_data[value].data[2].rssi = 0;
          }

        }
      }
    }
  }

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


