#include "smartmeter.h"
#include "Arduino.h"


SML::SML() {   // Constructor
	
	bufferSML[SML_MSG_BUFFER_SIZE] = {0};
	bufferlen  = 0;
	
	memset(bufferSML, 0, sizeof(bufferSML));
	
	  idx_wirkleistung = 0; // 4 Byte
	  idx_bezug  = 0;   // 8 Byte
	  idx_bezug1 = 0;   // 8 Byte
	  idx_bezug2 = 0;   // 8 Byte
		
}

void SML::add(uint8_t byte) {
	if (bufferlen < SML_MSG_BUFFER_SIZE-1) {
		bufferSML[bufferlen] = byte;
		bufferlen++;
	} else {
		printError(ERR_BUFFER_OVERFLOW);
    }
}

void SML::reset() {
  // clear the message buffer
  SML();
  memset(bufferSML, 0, sizeof(bufferSML));
  bufferlen = 0;

}

double SML::gib_bezug() {
	double wert = 0;
	return idx_bezug;
	if(idx_bezug2>0) {
		for (int j = 0; j < 8; j++) {
			wert += bufferSML[idx_bezug + j] * pow(2,(7 - j) * 8) / 10000;
			wert += bufferSML[idx_bezug + j] * pow(2,(7 - j) * 8) / 10000;
		}
		return wert;
	}
	return NULL;
}

double SML::gib_bezug1() {
	double wert = 0;
	if(idx_bezug1>0) {
		for (int j = 0; j < 8; j++) {
			wert += bufferSML[idx_bezug1 + j] * pow(2,(7 - j) * 8) / 10000;
		}
		return wert;
	}
	return NULL;
}

double SML::gib_bezug2() {
	double wert = 0;
	if(idx_bezug2>0) {
		for (int j = 0; j < 8; j++) {
			wert += bufferSML[idx_bezug2 + j] * pow(2,(7 - j) * 8) / 10000;
		}
		return wert;
	}
	return NULL;
}

double SML::gib_wirkleistung() {
	double wert = 0;
	if(idx_wirkleistung>0) {
		for (int j = 0; j < 4; j++) {
			wert += bufferSML[idx_wirkleistung + j] * pow(2,(3 - j) * 8) / 10000;
		}
		return wert;
	}
	return NULL;
}

void SML::updatePayload(uint8_t data[]) {
   for (int j = 0; j < 8; j++) {
	  if (idx_bezug1>0) {data[j] = bufferSML[idx_bezug1+j];}
	  else {data[j] = 0;}
   }
   for (int j = 0; j < 4; j++) {
	  if (idx_wirkleistung>0) {data[8+j] = bufferSML[idx_wirkleistung+j];}
	  else {data[8+j] = 0;}
   }
   
}

void SML::parseSML() {
  
  bool found;
  bool finished;
  unsigned char offset = 0;
  unsigned char tarif  = 0;
  
  idx_wirkleistung = 0; // 4 Byte
  idx_bezug  = 0;   // 8 Byte
  idx_bezug1 = 0;   // 8 Byte
  idx_bezug2 = 0;   // 8 Byte

  uint8_t suchstring[15];
  uint8_t suchlaenge;

  for (int i = 0; i < bufferlen; i++) {
    found = true;
    finished = false;
    offset   = 0;

    // Suche SML_Start;
	suchlaenge = sizeof(SML_CODE_START);
	memcpy_P(suchstring, SML_CODE_START, sizeof(SML_CODE_START));
  
    for (int j = 0; j < suchlaenge; j++) {
	  if (i + j >= bufferlen || bufferSML[i + j] != suchstring[j]) {
        found = false;
        break;
      }
    }

    // Start gefunden
    if (found) {
      offset = suchlaenge;
		
      //Prüfe auf Kennzahl BEZUG (8 Byte)
	  suchlaenge = sizeof(SML_CODE_BEZUG_TOTAL);
	  memcpy_P(suchstring, SML_CODE_BEZUG_TOTAL, sizeof(SML_CODE_BEZUG_TOTAL));
      found = true;
      for (int j = 0; j < suchlaenge; j++) {
        if (i + j + offset >= bufferlen || bufferSML[i + j + offset] != suchstring[j]) {
          found = false;
          break;
        }
      }
      if (found) {
        offset += suchlaenge;
        idx_bezug = i+offset;
        i = i + offset + 8;
        finished = true;
      }

      //Prüfe auf Kennzahl BEZUG1 
      if (!finished) {
		  found = true;
		  suchlaenge = sizeof(SML_CODE_BEZUG_TARIF1);
		  memcpy_P(suchstring, SML_CODE_BEZUG_TARIF1, sizeof(SML_CODE_BEZUG_TARIF1));
		  for (int j = 0; j < suchlaenge; j++) {
			if (i + j + offset >= bufferlen || bufferSML[i + j + offset] != suchstring[j]) {
			  found = false;
			  break;
			}
		  }
		  if (found) {
			offset += suchlaenge;
			idx_bezug1 = i+offset;
			finished = true;
			i = i + offset + 7;
		  }
	  }
	  //Prüfe auf Kennzahl BEZUG2 
      if (!finished) {
		  found = true;
		  suchlaenge = sizeof(SML_CODE_BEZUG_TARIF2);
		  memcpy_P(suchstring, SML_CODE_BEZUG_TARIF2, sizeof(SML_CODE_BEZUG_TARIF2));
		  for (int j = 0; j < suchlaenge; j++) {
			if (i + j + offset >= bufferlen || bufferSML[i + j + offset] != suchstring[j]) {
			  found = false;
			  break;
			}
		  }
		  if (found) {
			offset += suchlaenge;
			idx_bezug2 = i+offset;
			finished = true;
			i = i + offset + 8;
		  }
      }

      //Prüfe auf Kennzahl WIRKLEISTUNG (4 Byte)
      if (!finished) {

        found = true;
		suchlaenge = sizeof(SML_CODE_WIRKLEISTUNG);
		memcpy_P(suchstring, SML_CODE_WIRKLEISTUNG, sizeof(SML_CODE_WIRKLEISTUNG));
        for (int j = 0; j < suchlaenge; j++) {
          if (i + j + offset >= bufferlen || bufferSML[i + j + offset] != suchstring[j]) {
            found = false;
            break;
          }
        }
        if (found) {
          offset += suchlaenge;
          idx_wirkleistung = i+offset;
          i = i + offset + suchlaenge + 4;
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
   isValidHeader - returns true if the global message bufferSML begins with a valid SML escape sequence.
*/
bool SML::isValidSMLHeader() {
  return ((bufferSML[0] == 0x1b) &&
          (bufferSML[1] == 0x1b) &&
          (bufferSML[2] == 0x1b) &&
          (bufferSML[3] == 0x1b) &&
          (bufferSML[4] == 0x01) &&
          (bufferSML[5] == 0x01) &&
          (bufferSML[6] == 0x01) &&
          (bufferSML[7] == 0x01));
}



/**
   print bufferSML to serial
*/
void SML::printBuffer() {
  Serial.println(SEPARATOR_TEXT);
  for (int i = 0; i < bufferlen; i++) {
    if (i > 5 && i % 8 == 0) Serial.println();
    Serial.print(bufferSML[i] > 16  ? "" : "0");
    Serial.print(bufferSML[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println(SEPARATOR_TEXT);
}


/**
   printError - prints an error to serial console
*/
static void SML::printError(int messageNumber, ...) {
  Serial.print(F("Error: "));
  Serial.print(ERR_TEXT[messageNumber]);
  Serial.println();
}
