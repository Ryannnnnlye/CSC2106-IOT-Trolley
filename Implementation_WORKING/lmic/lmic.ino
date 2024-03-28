#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>

SoftwareSerial SUART(7, 8);  //SRX = 7, STX = 8

#define SCK 13
#define MISO 12
#define MOSI 11
#define SS 10
#define RST 7
#define DI0 2
#define BAND 923E6


static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui(u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

static const u1_t PROGMEM DEVEUI[8] = { 0xCD, 0x60, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

static const u1_t PROGMEM APPKEY[16] = { 0xD0, 0x22, 0x37, 0xDC, 0xB3, 0x1C, 0x99, 0x43, 0x89, 0x19, 0xD9, 0x32, 0xA4, 0x1F, 0xB6, 0xA6 };
void os_getDevKey(u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}


uint8_t mydata[255] = "Default data";
static osjob_t sendjob;
bool loraData = true;
int i = 0;

int fPort = 1;

const unsigned TX_INTERVAL = 5;

const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 7,
  .dio = { 2, 5, 6 },
};
void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print("0");
  Serial.print(v, HEX);
}

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

      Serial.println("ev tx");
      // Serial.println(i);

      // do_send(&sendjob);

      // if (i == 1) {
      //   setupLoRa();

      //   while (loraData) {

      //     loraSend();
      //   }
      //   setup();
      // }
      // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      i++;
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


void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

// Configuración LoRa
void setupLoRa() {
  Serial.println("set lora");
  LoRa.setPins(SS, RST, DI0);
  delay(3000);
  if (!LoRa.begin(BAND)) {
    Serial.println("Iniciando LoRa fallido !");
    while (1)
      ;
  }
}

void loraSend() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet ");
    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    // print RSSI of packet
    Serial.print("with RSSI ");
    Serial.println(LoRa.packetRssi());
    loraData = false;
  }
}
void setup() {

  // while (! Serial);
  Serial.begin(19200);
  Serial.println(F("Starting"));

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setClockError(MAX_CLOCK_ERROR * 7 / 100);
  LMIC_setLinkCheckMode(1);
  LMIC_setDrTxpow(DR_SF7, 14);

  LMIC_selectSubBand(1);

  // Start job (sending automatically starts OTAA too)

  do_send(&sendjob);
}

void loop() {
  os_runloop_once();

  byte n = SUART.available();
  if (n != 0) {
    char ch = SUART.read();
    data = SUART.read();
    Serial.print(ch);
    do_send(&sendjob);
  }
}