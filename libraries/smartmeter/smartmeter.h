/*******************************************************************************
 * Niels 2018 - No copyright
 * Read SML data from an EHC Meter
 *******************************************************************************/

#ifndef _smartmeter_h_
#define _smartmeter_h_

#include "Arduino.h"

/**
   error codes (= numbers of error beeps)
*/
#define ERR_INVALID_HEADER       1
#define ERR_HEADER_TIMEOUT       2
#define ERR_SERIAL_OVERFLOW      3
#define ERR_BUFFER_OVERFLOW      4

static const char *ERR_TEXT[] = {
  "OK", // No error
  "Inv.Head", //Invalid header
  "Header TO", //Timeout
  "Ser Ovf",  //Serial overflow
  "Buf Ovf"   //Buffer overflow
};

/**
   size of the SML message buffer = maximum number of bytes that can be received
*/
#define SML_MSG_BUFFER_SIZE    350

/**
   maximum time to wait for the end of a data packet received via IR
*/
#define SERIAL_READ_TIMEOUT_MS 100

#define SEPARATOR_TEXT "-----"
#define STR_KWh	       " kWh"

/**
   the global buffer to store the SML message currently being read
*/

static const uint8_t PROGMEM  SML_CODE_START[4]           = {0x77, 0x07, 0x01, 0x00};
static const uint8_t PROGMEM  SML_CODE_BEZUG_TOTAL[15]    = {0x01, 0x08, 0x00, 0xFF, 0x65, 0x00, 0x00, 0x01, 0x82, 0x01, 0x62, 0x1E, 0x52, 0xFF, 0x59};
static const uint8_t PROGMEM  SML_CODE_BEZUG_TARIF1[11]   = {0x01, 0x08, 0x01, 0xFF, 0x01, 0x01, 0x62, 0x1E, 0x52, 0xFF, 0x59};
static const uint8_t PROGMEM  SML_CODE_BEZUG_TARIF2[11]   = {0x01, 0x08, 0x02, 0xFF, 0x01, 0x01, 0x62, 0x1E, 0x52, 0xFF, 0x59};  
static const uint8_t PROGMEM  SML_CODE_WIRKLEISTUNG[11]   = {0x10, 0x07, 0x00, 0xFF, 0x01, 0x01, 0x62, 0x1B, 0x52, 0x00, 0x55};

class SML 
{
	/**
		SML Suchstrings
	*/

	private:

		uint8_t bufferSML[SML_MSG_BUFFER_SIZE];
		int  bufferlen; 
		unsigned int idx_wirkleistung; // 4 Byte
		unsigned int idx_bezug;   // 8 Byte
		unsigned int idx_bezug1;   // 8 Byte
		unsigned int idx_bezug2;   // 8 Byte
		
	public:
	
		SML();
		static void SML::printError(int messageNumber, ...);
		void parseSML();
		void add(uint8_t byte);
		void reset();
		double gib_wirkleistung();
		double gib_bezug();
		double gib_bezug1();
		double gib_bezug2();
		void printBuffer();
		bool isValidSMLHeader();
		void updatePayload(uint8_t data[]);
};
		


#endif // _smartmeter_h_

