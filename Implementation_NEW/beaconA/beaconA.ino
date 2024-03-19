#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <lmic.h>
#include <hal/hal.h>

static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui(u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

static const u1_t PROGMEM DEVEUI[8] = { 0x02, 0x60, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

static const u1_t PROGMEM APPKEY[16] = { 0x5E, 0xDD, 0x15, 0xD2, 0xA9, 0x37, 0xEE, 0x31, 0xED, 0x84, 0x59, 0x42, 0xDD, 0xF8, 0xBE, 0x6C };
void os_getDevKey(u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static osjob_t sendjob;

const unsigned TX_INTERVAL = 60;

int fPort = 1;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 7,
  .dio = { 2, 5, 6 },
};

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#define BEACON_A 2
#define TROLLEY_ID 5

#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint8_t numReceived = 0;
int16_t totalRSSI = 0;

struct LoRaMessage {
  uint8_t trolleyId;
  uint8_t data[8];
};

void (*resetFunc)(void) = 0;

void onEvent(ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
      }
      LMIC_setLinkCheckMode(0);
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));

      if (LMIC.dataLen) {
        Serial.print(F("Received "));

        //------ Added ----------------
        fPort = LMIC.frame[LMIC.dataBeg - 1];
        Serial.print(F("fPort "));
        Serial.println(fPort);
        //-----------------------------
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    default:
      Serial.print(F("Unknown event: "));
      break;
  }
}

void setup() {
  Serial.begin(19200);
  delay(100);
  Serial.println(F("Starting"));

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

  // LORAWAN STUFF
  os_init();
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  do_send(&sendjob, 0);
}


void do_send(osjob_t* j, int meanRssi) {
  

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    LMIC_setTxData2(fPort, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
  }
}

void loop() {
  uint8_t mydata[1];
  os_runloop_once();
  // Serial.print(F("loopenis\n"));
  if (rf95.available()) {


    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    // Receive broadcast from trolley
    if (rf95.recv(buf, &len)) {
      Serial.print(F("Hello bitches\n"));
      LoRaMessage* receivedMessage = (LoRaMessage*)buf;
      receivedMessage->trolleyId = 5;

      if (receivedMessage->trolleyId == TROLLEY_ID) {
        Serial.print(F("Hello bitches2"));
        numReceived++;
        totalRSSI += rf95.lastRssi();

        // Serial.print(F("Received from trolley ("));
        // Serial.print(numReceived);
        // Serial.print(F(" times) The RSSI is: "));
        // Serial.println(rf95.lastRssi(), DEC);

        // Check if 10 messages have been received
        if (numReceived == 10) {
          Serial.print(F("Hello bitches3\n"));

          mydata[0] = int((totalRSSI) / numReceived);
          // Serial.println("the mean rssi is " + mydata[0]);
          do_send(&sendjob, meanRssi);

          Serial.println(F("Packet queued"));

          // Reset the variables
          numReceived = 0;
          totalRSSI = 0;
        }
      }
    } else {
      Serial.println(F("Receive failed"));
    }
  } else {
    Serial.print(F("Fail fish\n"));
  }
}
