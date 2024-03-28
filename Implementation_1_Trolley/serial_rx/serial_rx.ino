#include<SoftwareSerial.h>
SoftwareSerial SUART(7, 8); //SRX = 7, STX = 8

void setup()
{
  Serial.begin(19200);
  SUART.begin(19200);
}

void loop()
{
  byte n = SUART.available();
  if (n != 0)
  {
    char ch = SUART.read();
    Serial.print(ch);
  }
}