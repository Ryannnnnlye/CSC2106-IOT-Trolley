
#include <RHReliableDatagram.h>  //get it here http://lowpowerlab.com/RadioHead_LowPowerLab.zip
#include <RH_RF95.h>  //get it here http://lowpowerlab.com/RadioHead_LowPowerLab.zip
#include <SPI.h> //get it here: https://www.github.com/lowpowerlab/spiflash

#define SERVER_ADDRESS 1 // address number of the Server
#define CLIENT_ADDRESS 2 // address number of the Client 
#define RETRIES 4 // Number of times the sendtoWait() will try to send a message. Default is 3
#define TIMEOUT 2000 // Timeout before sendWait() tries again to send a message 

// Match frequency to the hardware version of the radio on your Moteino
#define FREQUENCY   433 

// Singleton instance of the radio driver
RH_RF95 driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

void setup() 
{
   // connect to the serial monitor at 115200 baud
  Serial.begin(115200);
  Serial.println("DATAGRAM TEST - CLIENT");
  Serial.println("");
  
  //Initialize the radio
  if (!manager.init())
    Serial.println("Init failed!");
  else
  {
    Serial.print("Init OK - ");
    Serial.println(FREQUENCY); Serial.print("mhz");
    driver.setFrequency(FREQUENCY);
  }

  manager.setRetries(RETRIES);
  manager.setTimeout(TIMEOUT);

}

uint8_t data[] = "Hello there!";
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void loop()
{
  Serial.println("Sending a message to the SERVER... ");
    
  // Send a message to manager_server
  if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS))
  {
    uint8_t len = sizeof(buf);
    uint8_t from;   
    // Now wait for a reply from the server
    if (manager.recvfromAckTimeout(buf, &len, 500, &from))
    {
      Serial.print("Got reply from server ID ");
      Serial.print(from);
      Serial.print(": ");
      Serial.println((char*)buf);
      Serial.println("");
    }
    else
    {
      Serial.println("No reply, is SERVER running?");
    }
  }
  else
    Serial.println("Transmission of data failed!");
  delay(500);
}