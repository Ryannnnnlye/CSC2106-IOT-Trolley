#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
// #include <DHT.h>
// #include <DHT_U.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>

#ifdef COMPILE_REGRESSION_TEST
#define FILLMEIN 0
#else
#warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
#define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

SoftwareSerial SUART(7, 8);  //SRX = 7, STX = 8

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui(u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x02, 0x60, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x5E, 0xDD, 0x15, 0xD2, 0xA9, 0x37, 0xEE, 0x31, 0xED, 0x84, 0x59, 0x42, 0xDD, 0xF8, 0xBE, 0x6C };
void os_getDevKey(u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static uint8_t btn_activated[1] = { 0x01 };
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 7,
  .dio = { 2, 5, 6 },
};

void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}


#define DHT_PIN 8

// DHT11 or DHT22
#define DHTTYPE DHT22

// Initialize dht
// DHT dht(DHT_PIN, DHTTYPE);

int fPort = 1;            // fPort usage: 1=dht11, 2=button, 3=led
int buttonState = 0;      // current state of the button
int lastButtonState = 0;  // previous state of the button
//-----------------------------


void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
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
      // {
      //   u4_t netid = 0;
      //   devaddr_t devaddr = 0;
      //   u1_t nwkKey[16];
      //   u1_t artKey[16];
      //   LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
      //   Serial.print("netid: ");
      //   Serial.println(netid, DEC);
      //   Serial.print("devaddr: ");
      //   Serial.println(devaddr, HEX);
      //   Serial.print("AppSKey: ");
      //   for (size_t i = 0; i < sizeof(artKey); ++i) {
      //     if (i != 0)
      //       Serial.print("-");
      //     printHex2(artKey[i]);
      //   }
      //   Serial.println("");
      //   Serial.print("NwkSKey: ");
      //   for (size_t i = 0; i < sizeof(nwkKey); ++i) {
      //     if (i != 0)
      //       Serial.print("-");
      //     printHex2(nwkKey[i]);
      //   }
      //   Serial.println();
      // }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      // if (LMIC.txrxFlags & TXRX_ACK)
      //   Serial.println(F("Received ack"));

      // if (LMIC.dataLen) {
      //   Serial.print(F("Received "));
      //   Serial.print(LMIC.dataLen);
      //   Serial.println(F(" bytes of payload"));


      //   //------ Added ----------------
      //   fPort = LMIC.frame[LMIC.dataBeg - 1];
      //   Serial.print(F("fPort "));
      //   Serial.println(fPort);

      //   Serial.println();
        //-----------------------------
      // }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
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
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;

    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // uint32_t humidity = dht.readHumidity(false) * 100;
    // uint32_t temperature = dht.readTemperature(false) * 100;

    // Serial.println("Humidity: " + String(humidity));
    // Serial.println("Temperature: " + String(temperature));

    // uint8_t payload[] = "Hello";
    // payload[0] = highByte(humidity);
    // payload[1] = lowByte(humidity);
    // payload[2] = highByte(temperature);
    // payload[3] = lowByte(temperature);
    uint8_t mydata[] = "Hello world!";
    fPort = 1;
    // Serial.println(mydata);
    //Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(fPort, mydata, sizeof(mydata) - 1, 0);
    // fPort = 1;

    //Prepare upstream data transmission at the next possible time.
    // LMIC_setTxData2(fPort, payload, sizeof(payload), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(19200);
  Serial.println(F("Starting"));
  // SUART.begin(19200);

  // dht.begin();
  //-----------------------------

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Use with Arduino Pro Mini ATmega328P 3.3V 8 MHz
  // Let LMIC compensate for +/- 1% clock error
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // Disable link check validation – this is used to periodically verify
  LMIC_setLinkCheckMode(0);

  // Set data rate to Spreading Factor 7 and transmit power to 14 dBi for
// uplinks
 LMIC_setDrTxpow(DR_SF7, 14);

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {

  os_runloop_once();
  byte n = SUART.available();
  if (n != 0) {
    uint8_t ch = SUART.read();
    Serial.print(ch);
    do_send(&sendjob);
  }
}
