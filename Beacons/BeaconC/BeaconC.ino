#include <RHReliableDatagram.h>  //get it here http://lowpowerlab.com/RadioHead_LowPowerLab.zip
#include <RH_RF95.h>  //get it here http://lowpowerlab.com/RadioHead_LowPowerLab.zip
#include <SPI.h> //get it here: https://www.github.com/lowpowerlab/spiflash


/*
1. me and jason will act as beacon
2. ping the shopping cart for 10 seconds
3. collate the RSI values
4. send to the raspberry pi to calculate distance and perform trilateration
*/


#define CART_A_ADDRESS 5 // address number cart
#define BEACON_C_ADDRESS 4 // address number of the Client
#define SERVER_ADDRESS 1 // address number of the Server
#define RETRIES 4 // Number of times the sendtoWait() will try to send a message. Default is 3
#define TIMEOUT 2000 // Timeout before sendWait() tries again to send a message 

// Match frequency to the hardware version of the radio on your Moteino
#define FREQUENCY   915 

// Singleton instance of the radio driver
RH_RF95 driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, BEACON_C_ADDRESS);

void setup() 
{
   // connect to the serial monitor at 115200 baud
  Serial.begin(115200);
  Serial.println("Beacon C");
  Serial.println("");
  
  //Initialize the radio
  if (!manager.init())
    Serial.println("Init failed!");
  else
  {
    Serial.print("Init OK - ");
    driver.setFrequency(FREQUENCY);
  }

  manager.setRetries(RETRIES);
  manager.setTimeout(TIMEOUT);

}

uint8_t data[] = "ping";
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
uint8_t from;

typedef struct {
    uint8_t beacon_id;
    uint8_t trolley_id;
    float average_rssi;
} packet;


packet message;

// his example he set in rtos
void loop()
{
  Serial.println("[Beacon C] sending to cart... ");

  bool trolley_id_set = false;
  message.beacon_id = BEACON_C_ADDRESS;
  int16_t totalRssi = 0;


  for (int i = 0; i < 10; i++)
  {
    if (manager.sendtoWait(data, sizeof(data), CART_A_ADDRESS))
    {
      if (manager.recvfromAckTimeout(buf, &len, 500, &from))
      {
        if(!trolley_id_set)
        {
          message.trolley_id = from;
          trolley_id_set = true;
        }
        Serial.println("The rsi value is " + String(driver.lastRssi()));
        totalRssi += driver.lastRssi();
      }
    }
    else
    {
      Serial.println("Packet Loss Cart A!");
    }
  }

  Serial.println("The total rsi is " + String(totalRssi));
  message.average_rssi = totalRssi / 10.0;
  Serial.println("The average value is " + String(message.average_rssi));

  
  if (manager.sendtoWait((uint8_t*)&message, sizeof(message), SERVER_ADDRESS))
    {
      if (manager.recvfromAckTimeout(buf, &len, 4000, &from))
      {
        Serial.println("Transmission successful");
      }
    }
}