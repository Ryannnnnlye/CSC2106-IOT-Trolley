#include <RHReliableDatagram.h>
// #include <RHDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

#define SERVER_ADDRESS 1
#define BEACON_A_ADDRESS 2
#define BEACON_B_ADDRESS 3
#define BEACON_C_ADDRESS 4
#define TROLLEY_ADDRESS 5 // self

#define FREQUENCY 915

#define NSS 5
#define DIO0 26
// RH_RF95 driver(NSS, DIO0);
RH_RF95 driver;

RHReliableDatagram manager(driver, TROLLEY_ADDRESS);


void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for serial port to be available
  if (!manager.init())
    Serial.println("init failed");
  else
    Serial.println("init success");

  driver.setFrequency(FREQUENCY);
  driver.setTxPower(23, false);
  manager.setRetries(1);
  manager.setTimeout(150);
}

uint8_t data[] = "a";
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void loop()
{
  // If data is bein gsent
  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      Serial.print("got request from address: ");
      Serial.println(from, DEC);
      Serial.print("Message: ");
      // Message sent
      Serial.println((char *)buf);
      // Check if address is from server
      if (from == BEACON_A_ADDRESS || from == BEACON_B_ADDRESS || from == BEACON_C_ADDRESS) {
        Serial.print("Received from beacon: ");
        Serial.println(from);
        // Send a reply back to the originator client
        if (!manager.sendtoWait(data, sizeof(data), from))
          Serial.println("sendtoWait failed");
    }
      }
      if (from == SERVER_ADDRESS) {
        Serial.println("Received from server.");
        bool result = ((char *)buf == '1');
        if (result) {
          Serial.println("Lock wheels.");
        }
        else {
          Serial.println("Do not lock wheels.");
        }

      }


  }
}