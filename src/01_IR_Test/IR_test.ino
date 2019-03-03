#include <SoftwareSerial.h>


#define IR_RX_PIN  8
#define IR_TX_PIN  2 // not used, but required for SoftwareSerial
#define IR_INVERTED false // falls der Pegel invertiert ist (Verdrahtung Niels)

#define PIN_VCC_IR 2  // Pin um den IR-Sensor  mit Spannung zu versorgen
#define PIN_VCC_INVERTER 3 // Pin um den Inverter mit Spannung zu versorgen


/**
    SML Suchstrings
*/

static const byte SML_CODE_START[4]           = {0x77, 0x07, 0x01, 0x00};
static const byte SML_CODE_BEZUG_TOTAL[15]    = {0x01, 0x08, 0x00, 0xFF, 0x65, 0x00, 0x00, 0x01, 0x82, 0x01, 0x62, 0x1E, 0x52, 0xFF, 0x59};
static const byte SML_CODE_BEZUG_TARIF1[11]   = {0x01, 0x08, 0x01, 0xFF, 0x01, 0x01, 0x62, 0x1E, 0x52, 0xFF, 0x59};
static const byte SML_CODE_BEZUG_TARIF2[11]   = {0x01, 0x08, 0x02, 0xFF, 0x01, 0x01, 0x62, 0x1E, 0x52, 0xFF, 0x59};
static const byte SML_CODE_WIRKLEISTUNG[11]   = {0x10, 0x07, 0x00, 0xFF, 0x01, 0x01, 0x62, 0x1B, 0x52, 0x00, 0x55};


/**
   error codes (= numbers of error beeps)
*/
#define ERR_INVALID_HEADER       1
#define ERR_HEADER_TIMEOUT       2
#define ERR_SERIAL_OVERFLOW      3
#define ERR_BUFFER_OVERFLOW      4

static const char *ERR_TEXT[] = {
  "No error",
  "Invalid Header",
  "Serial Overflow",
  "Buffer Overflow"
};

/**
   size of the SML message buffer = maximum number of bytes that can be received
*/
#define SML_MSG_BUFFER_SIZE    500



/**
   maximum time to wait for the end of a data packet received via IR
*/
#define SERIAL_READ_TIMEOUT_MS 100

#define SEPARATOR_TEXT "<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>"

/***************************************************************************************************
   GLOBAL VARIABLES (YUCK!)
 ***************************************************************************************************/


//  Smart Meter Daten auslesen
SoftwareSerial irSerial(IR_RX_PIN, IR_TX_PIN, IR_INVERTED); // RX, TX  (8 ist aktuell unbenutzt)

/**
   the global buffer to store the SML message currently being read
*/
unsigned char buffer[SML_MSG_BUFFER_SIZE];

struct zaehlwerte {
  unsigned long wirkleistung = 0; // 4 Byte
  unsigned long long bezug  = 0;   // 8 Byte
  unsigned long long bezug1 = 0;   // 8 Byte
  unsigned long long bezug2 = 0;   // 8 Byte
  boolean hat_wirkleistung = false;
  boolean hat_bezug  = false;
  boolean hat_bezug1 = false;
  boolean hat_bezug2 = false;
};

/***************************************************************************************************
   SUBROUTINES Smart Meter
 ***************************************************************************************************/
struct zaehlwerte parseSML(int len) {
  struct zaehlwerte zw;
  boolean found;
  boolean finished;
  byte offset = 0;

  for (int i = 0; i < len; i++) {
    found = true;
    finished = false;
    offset   = 0;

    // Suche SML_Start;

    for (int j = 0; j < 4; j++) {
      if (i + j >= len || buffer[i + j] != SML_CODE_START[j]) {
        found = false;
        break;
      }
    }

    // Start gefunden
    if (found) {
      offset = 4;

      //Prüfe auf Kennzahl BEZUG (8 Byte)
      found = true;
      for (int j = 0; j < 15; j++) {
        if (i + j + offset >= len || buffer[i + j + offset] != SML_CODE_BEZUG_TOTAL[j]) {
          found = false;
          break;
        }
      }
      if (found) {
        offset += 15;
        for (int j = 0; j < 8; j++) {
          zw.bezug += (unsigned long long) buffer[i + j + offset] << (7 - j) * 8; // bitweise verschieben
        }
        zw.hat_bezug = true;
        i = i + offset + 8;
        finished = true;
      }

      //Prüfe auf Kennzahl BEZUG1 (8 Byte)
      if (!finished) {
      found = true;
      for (int j = 0; j < 11; j++) {
        if (i + j + offset >= len || buffer[i + j + offset] != SML_CODE_BEZUG_TARIF1[j]) {
          found = false;
          break;
        }
      }
      if (found) {
        offset += 11;
        for (int j = 0; j < 8; j++) {
          zw.bezug1 += (unsigned long long) buffer[i + j + offset] << (7 - j) * 8; // bitweise verschieben
        }
        zw.hat_bezug1 = true;
        finished = true;
        i = i + offset + 8;
      }

      //Prüfe auf Kennzahl BEZUG2 (8 Byte)
      if (!finished)
      found = true;
      for (int j = 0; j < 11; j++) {
        if (i + j + offset >= len || buffer[i + j + offset] != SML_CODE_BEZUG_TARIF2[j]) {
          found = false;
          break;
        }
      }
      if (found) {
        offset += 11;
        for (int j = 0; j < 8; j++) {
          zw.bezug2 += (unsigned long long) buffer[i + j + offset] << (7 - j) * 8; // bitweise verschieben
        }
        zw.hat_bezug2 = true;
        finished = true;
        i = i + offset + 8;
      }


      //Prüfe auf Kennzahl WIRKLEISTUNG (4 Byte)
      if (!finished) {

        found = true;
        for (int j = 0; j < 10; j++) {
          if (i + j + offset >= len || buffer[i + j + offset] != SML_CODE_WIRKLEISTUNG[j]) {
            found = false;
            break;
          }
        }
        if (found) {
          offset += 11;
          for (int j = 0; j < 4; j++) {
            zw.wirkleistung += (unsigned long long) buffer[i + j + offset] << (3 - j) * 8;
          }
          zw.hat_wirkleistung = true;
          i = i + offset + 11 + 4;
        }
      }
      }
      if (!finished) {
        // Unbekannte Kennzahl
        i = i + offset;
      }
    }
  }
}

/**
   isValidHeader - returns true if the global message buffer begins with a valid SML escape sequence.
*/
inline boolean isValidHeader() {
  return ((buffer[0] == 0x1b) ||
          (buffer[1] == 0x1b) ||
          (buffer[2] == 0x1b) ||
          (buffer[3] == 0x1b) ||
          (buffer[4] == 0x01) ||
          (buffer[5] == 0x01) ||
          (buffer[6] == 0x01) ||
          (buffer[7] == 0x01));
}




/**
   print buffer to serial
*/
void printBuffer(int len) {
  Serial.println(SEPARATOR_TEXT);
  for (int i = 0; i < len; i++) {
    if (i > 5 && i % 8 == 0) Serial.println();
    Serial.print(buffer[i] > 16  ? "" : "0");
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println(SEPARATOR_TEXT);
}


/**
   printError - prints an error to serial console
*/
void printError(int messageNumber, ...) {
  Serial.print("Error: ");
  Serial.print(ERR_TEXT[messageNumber]);
  Serial.println();
}


/**
   SETUP
*/


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  // set the pin configuration
  pinMode(IR_RX_PIN, INPUT);

  pinMode(PIN_VCC_INVERTER, OUTPUT);
  pinMode(PIN_VCC_IR, OUTPUT);
  digitalWrite(PIN_VCC_INVERTER, HIGH);
  digitalWrite(PIN_VCC_IR, HIGH);


  //
  irSerial.begin(9600);

  Serial.println("Init complete");
}

/**
   LOOP
*/


void loop() {
  unsigned int nextBufferPosition = 0;
  unsigned long lastReadTime = 0;

  // clear the message buffer
  memset(buffer, 0, sizeof(buffer));

  // remove a pending overflow flag (might have been left over from a previous run)
  // and ensure that the SoftwareSerial library is listening
  irSerial.overflow();
  irSerial.listen();



  // wait until actual data is available
  while (!irSerial.available());

  Serial.println("Da kommen Daten.. auf geht's");

  // keep reading data until either the message buffer is filled or no more data was
  // received for SERIAL_READ_TIMEOUT_MS ms
  lastReadTime = millis();
  while (millis() - lastReadTime < SERIAL_READ_TIMEOUT_MS) {
    if (irSerial.available()) {
      buffer[nextBufferPosition] = irSerial.read();
      lastReadTime = millis();
      if (nextBufferPosition >= SML_MSG_BUFFER_SIZE) {
        printError(ERR_BUFFER_OVERFLOW);
        return;
      }
      nextBufferPosition += 1;
    }
  }

  Serial.println("Empfang beendet... dann werte ich mal aus.");

  // report an error if an overflow condition was encountered - the data received is useless
  // in this case :-(
  if (irSerial.overflow()) {
    printError(ERR_BUFFER_OVERFLOW);
  } else {
    // check the header
    if (!isValidHeader()) {
      // not a valid header - notify the user...
      printError(ERR_INVALID_HEADER);
      printBuffer(nextBufferPosition);
      // ...and empty the receiver buffer (wait for the end of the current data stream
      while (irSerial.available() > 0) {
        irSerial.read();
      }
    } else {
      Serial.println("Valider Header....");
      zaehlwerte zw = parseSML(nextBufferPosition);
      if (zw.hat_bezug)         {
        Serial.print("Bezug Total:     ");
        Serial.print((long) zw.bezug / 10000);
        Serial.println(" kWh");
      }
      if (zw.hat_bezug1) {
        Serial.print("Bezug (Tarif 1): ");
        Serial.print((long) zw.bezug1 / 10000);
        Serial.println(" kWh");
      }
      if (zw.hat_bezug2) {
        Serial.print("Bezug (Tarif 2): ");
        Serial.print((long) zw.bezug2 / 10000);
        Serial.println(" kWh");
      }
      if (zw.hat_wirkleistung)  {
        Serial.print("Wirkleistung:  ");
        Serial.print((double) zw.wirkleistung / 1000);
        Serial.println(" kW");
      }

      //printBuffer(nextBufferPosition);

    }
  }
}
