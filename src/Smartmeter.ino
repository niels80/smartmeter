/*******************************************************************************
    Send Smart Meter Readings via Lora

 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <smartmeter.h>
#include <LowPower.h>
#include <Arduino.h>

#define MY_DEBUG 0


// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
// DEVEUI und AppEUI LSB
static const PROGMEM u1_t NWKSKEY[16] = { 0x7E, 0x10, ... *ENTER TEXT HERE* };
 
 
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
// MSB
static const u1_t PROGMEM APPSKEY[16] = { 0xF9, 0xCF, ... *ENTER TEXT HERE* };
 
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x2....... *ENTER TEXT HERE*;
 
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

/* 
 *  Ende Variante mit vordefiniertem Session Key 
 */


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 56;  // 56 Seconds wait
 
static uint8_t mydata[14] = {0};
static osjob_t sendjob;

// Pin mapping (as in https://things4u.github.io/HardwareGuide/Arduino/Mini-Sensor-HTU21/mini-lowpower.html)
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 6,
  .dio = {4, 5, 7},
};


// Smart Meter Input
#define IR_RX_PIN  8
#define IR_TX_PIN  A3 // not used, but required for SoftwareSerial
#define IR_INVERTED false // 

// Voltage Input
#define PIN_VOLTAGE A2  // Connect voltage divider to A2

#define PIN_VCC_IR 2  // Pin to supply voltage to IR diode
#define PIN_VCC_INVERTER 3 // Pin to supply voltage to Inverter

//  Smart Meter Interface
SoftwareSerial irSerial(IR_RX_PIN, IR_TX_PIN, IR_INVERTED); // RX, TX  (8 ist aktuell unbenutzt)

// Smart Meter Data
SML sml;


/*
 * Debug
 */


#if MY_DEBUG > 0
#define debugPrint(a) (Serial.print(a))
#define debugPrintln(a) (Serial.println(a))
#else
#define debugPrint(a) 
#define debugPrintln(a) 
#endif
/**
*
*  Hole Zaehlerdaten / Read IR data
*/
void holeIRdaten() {
 
  // remove a pending overflow flag (might have been left over from a previous run)
  // and ensure that the SoftwareSerial library is listening
  unsigned long lastReadTime = 0;
  irSerial.overflow();
  irSerial.listen();
  sml.reset();
  
  // wait until actual data is available
  while (!irSerial.available());

  debugPrintln(F("Empfange IR-Daten..."));

  // keep reading data until either the message buffer is filled or no more data was
  // received for SERIAL_READ_TIMEOUT_MS ms
  lastReadTime =    millis();
  while (millis() - lastReadTime < SERIAL_READ_TIMEOUT_MS) {
    if (irSerial.available()) {
      sml.add(irSerial.read());
      lastReadTime = millis();
    }
  }

}


/**
*
*  Generate testdata (can be commented out if unused)
*/
void holeTestdaten() {  

 uint8_t tmp[15];
 uint8_t len;
 
 sml.reset();
 sml.add(0x1b);
 sml.add(0x1b);
 sml.add(0x1b);
 sml.add(0x1b);
 sml.add(0x01);
 sml.add(0x01);
 sml.add(0x01);
 sml.add(0x01);

 for (int i=0; i<100; i++) { sml.add(random(0,254));  }
 len = sizeof(SML_CODE_START);
 memcpy_P(tmp, SML_CODE_START, len);
 for (int i=0; i<len; i++) {sml.add(tmp[i]); }

 len = sizeof(SML_CODE_BEZUG_TARIF1);
 memcpy_P(tmp, SML_CODE_BEZUG_TARIF1, len);
 for (int i=0; i<len; i++) {sml.add(tmp[i]); }
 for (int i=0; i<8; i++) { sml.add(1);  }
 for (int i=0; i<100; i++) { sml.add(random(0,254));  }
}


void holeZaehlerdaten() {

   // Power on
  digitalWrite(PIN_VCC_INVERTER, HIGH);
  digitalWrite(PIN_VCC_IR, HIGH);
  delay(10);

  // set the pin configuration
  irSerial.begin(9600);
  

  // Here you can choose if you want real data or test data to test LoRa connection
  holeIRdaten();  // Real data
  //holeTestdaten(); // Test data
   
  // Power off
  digitalWrite(PIN_VCC_INVERTER, LOW);
  digitalWrite(PIN_VCC_IR, LOW);
  
  if (!sml.isValidSMLHeader()) {
    debugPrintln(F("Invalid header, 2nd try"));
    holeIRdaten();
  }

  // report an error if an overflow condition was encountered - the data received is useless
  // in this case :-(
  if (irSerial.overflow()) {
    SML::printError(ERR_BUFFER_OVERFLOW);
  } else {
    // check the header

    if (!sml.isValidSMLHeader()) {
      // not a valid header - notify the user...
      SML::printError(ERR_INVALID_HEADER);
      sml.printBuffer();
      // ...and empty the receiver buffer (wait for the end of the current data stream
      
    } else {
      debugPrintln(F("Valid  Header"));
      sml.parseSML();
           
      debugPrint(F("B:"));
      debugPrint(sml.gib_bezug());
      debugPrintln(F(STR_KWh));

      debugPrint(F("B1:"));
      debugPrint(sml.gib_bezug1());
      debugPrintln(STR_KWh);
      
      debugPrint(F("B2:"));
      debugPrint(sml.gib_bezug2());
      debugPrintln(F(STR_KWh));

      debugPrint(F("Pwirk:  "));
      debugPrint(sml.gib_wirkleistung());
      debugPrintln(F(" kW"));
    }
  }
  sml.updatePayload(mydata);
  irSerial.end();

   
   // Read battery data
   // Source: https://github.com/rocketscream/MiniUltraPro/blob/master/ttn-otaa-sleep.ino
   // ***** Battery monitor connection
   //
   // VBAT-----1M-----3M3-----GND
   //              | 
   //              ---0.1uF---GND
   //              |
   //              A2   
  int iV = analogRead(PIN_VOLTAGE);
  iV = (int) ((float) iV / 1023 * 4.3 * 1000);
  debugPrint(F("Spannung:  "));
  debugPrint(iV);
  debugPrintln(F(" mV"));

  mydata[12] = iV >> 8; 
  mydata[13] = iV & 0xFF; 
}



void onEvent (ev_t ev) {
  debugPrint(os_getTime());
  debugPrint(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      debugPrintln(F("EV_SCAN_TO"));
      break;
    case EV_BEACON_FOUND:
      debugPrintln(F("EV_BF"));
      break;
    case EV_BEACON_MISSED:
      debugPrintln(F("EV_BM"));
      break;
    case EV_BEACON_TRACKED:
      debugPrintln(F("EV_BT"));
      break;
    case EV_JOINING:
      debugPrintln(F("EV_JOINING"));
      break;
    case EV_JOINED:
      debugPrintln(F("EV_JOINED"));

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      debugPrintln(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      debugPrintln(F("EV_JOIN_FAIL"));
      break;
    case EV_REJOIN_FAILED:
      debugPrintln(F("EV_REJOIN_FAIL"));
      break;
      break;
    case EV_TXCOMPLETE:
      debugPrintln(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        debugPrintln(F("Rcvd ack"));
      if (LMIC.dataLen) {
        debugPrintln(F("Received "));
        debugPrintln(LMIC.dataLen);
        debugPrintln(F(" bytes of payload"));
      }
      
      debugPrintln(F("Sleep"));
      for (int i=0; i<int(TX_INTERVAL/8); i++) {
           // Use library from https://github.com/rocketscream/Low-Power
            debugPrint(F("Sleeping"));
            debugPrintln(i);
            LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      }
      debugPrintln(F("Wakeup"));
      
     // Get new data
      holeZaehlerdaten();
	  
	  // Send out new data
      do_send(&sendjob);
      

      break;
    case EV_LOST_TSYNC:
      debugPrintln(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      debugPrintln(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      debugPrintln(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      debugPrintln(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      debugPrintln(F("EV_LINK_ALIVE"));
      break;
    default:
      debugPrintln(F("Unknwn event"));
      break;
  }
}

void do_send(osjob_t* j) {
  
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    debugPrintln(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    int resp = LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    debugPrintln(F("Packet queued"));
    debugPrint(F("Return code:"));
    debugPrintln(resp);
  }
}


/**
 * SETUP
 * 
 * 
*/

void setup() {
  
  Serial.begin(57600); //9600
  debugPrintln(F("Starting"));

  // Init Voltage Measurement
  pinMode(PIN_VOLTAGE,INPUT);
  analogReference(DEFAULT); 

  // Init IR sensor
  pinMode(IR_RX_PIN, INPUT);

  // Voltage supply to sensors (they eat up 20mA so they must be put to sleep when not used).
  pinMode(PIN_VCC_INVERTER, OUTPUT);
  pinMode(PIN_VCC_IR, OUTPUT);
  digitalWrite(PIN_VCC_INVERTER, LOW);
  digitalWrite(PIN_VCC_IR, LOW);
  
  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Bugfix for RFM95
  LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);


#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

   
#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
      LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
      LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
//  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(1, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7B),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(2, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(3, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(4, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(5, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(6, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(7, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
//  LMIC_setupChannel(8, 868100000, DR_RANGE_MAP(DR_FSK, DR_FSK),  BAND_MILLI);      // g2-
// 
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif
 
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
 
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
 
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);


  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
