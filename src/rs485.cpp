#include "rs485.h"
#include "ui.h"
uint8_t serialBuffer[332];
uint16_t bufferPos = 0;

// ===== CRC-16 Modbus Calculation =====
uint16_t calculateChecksum(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)data[pos];
    for (uint8_t i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// ===== Frame Validation =====
bool validateFrame(uint8_t* frame, uint16_t len) {
  if (len < 15 || frame[0] != STX1 || frame[1] != STX2) {
    return false;
  }

  uint16_t declaredLength = (frame[2] << 8) | frame[3];
  uint16_t expectedLength = declaredLength + 6;
  
  if (len != expectedLength) {
    return false;
  }

  uint16_t etxPos = 4 + declaredLength - 1;
  if (frame[etxPos] != ETX) {
    return false;
  }

  uint16_t calculatedChecksum = calculateChecksum(&frame[2], declaredLength + 2);
  uint16_t receivedChecksum = (frame[expectedLength-2] << 8) | frame[expectedLength-1];
  
  if (receivedChecksum != calculatedChecksum) {
    return false;
  }

  return true;
}

void read_rs485_frames() {
  static uint16_t expectedFrameLength = 0;
  static bool frameStarted = false;
  
  while (Serial1.available()) {
    uint8_t incomingByte = Serial1.read();
    
    // ===== STATE 1: Looking for STX1 (0x5D) =====
    if (!frameStarted && bufferPos == 0) {
      if (incomingByte == STX1) {
        serialBuffer[bufferPos++] = incomingByte;
      }
      // Discard all other bytes
    }
    
    // ===== STATE 2: Looking for STX2 (0x47) =====
    else if (!frameStarted && bufferPos == 1) {
      if (incomingByte == STX2) {
        serialBuffer[bufferPos++] = incomingByte;
        frameStarted = true;
        // Removed Serial.println() for speed
      } else {
        // False start, reset
        bufferPos = 0;
        if (incomingByte == STX1) {
          serialBuffer[bufferPos++] = incomingByte;
        }
      }
    }
    
    // ===== STATE 3: Capturing frame data =====
    else if (frameStarted) {
      if (bufferPos < sizeof(serialBuffer)) {
        serialBuffer[bufferPos++] = incomingByte;
        
        // Read length field (once, at position 4)
        if (bufferPos == 4 && expectedFrameLength == 0) {
          uint16_t declaredLength = (serialBuffer[2] << 8) | serialBuffer[3];
          expectedFrameLength = declaredLength + 6; // Total = LEN + STX(2) + LEN(2) + CRC(2)
          
          // Sanity check
          if (expectedFrameLength > sizeof(serialBuffer) || expectedFrameLength < 15) {
            // Invalid length, reset
            bufferPos = 0;
            frameStarted = false;
            expectedFrameLength = 0;
          }
        }
        
        // Check if we have complete frame
        if (expectedFrameLength > 0 && bufferPos == expectedFrameLength) {
          // Complete frame received!
          
          // Quick validation and process
          if (quickValidateFrame(serialBuffer, expectedFrameLength)) {
            processCompleteFrame();
          }
          
          // Reset for next frame
          bufferPos = 0;
          frameStarted = false;
          expectedFrameLength = 0;
        }
        // Safety: exceeded expected length
        else if (expectedFrameLength > 0 && bufferPos > expectedFrameLength) {
          bufferPos = 0;
          frameStarted = false;
          expectedFrameLength = 0;
        }
      } else {
        // Buffer overflow
        bufferPos = 0;
        frameStarted = false;
        expectedFrameLength = 0;
      }
    }
  }
}

/* Fast frame validation - No debug prints */
bool quickValidateFrame(uint8_t* frame, uint16_t len) {
  // Check minimum length and STX
  if (len < 15 || frame[0] != STX1 || frame[1] != STX2) {
    return false;
  }

  // Verify length field matches
  uint16_t declaredLength = (frame[2] << 8) | frame[3];
  uint16_t expectedLength = declaredLength + 6;
  
  if (len != expectedLength) {
    return false;
  }

  // Check ETX at correct position (3 bytes before end)
  uint16_t etxPos = expectedLength - 3;
  if (frame[etxPos] != ETX) {
    return false;
  }

  // Validate CRC16
  uint16_t calculatedCRC = calculateChecksum(&frame[2], declaredLength + 2);
  uint16_t receivedCRC = (frame[expectedLength - 2] << 8) | frame[expectedLength - 1];
  
  return (calculatedCRC == receivedCRC);
}

/* Process validated frame - Fast, no prints */
void processCompleteFrame() {
  uint16_t declaredLength = (serialBuffer[2] << 8) | serialBuffer[3];
  uint16_t expectedFrameLength = declaredLength + 6;
  uint8_t etxPos = expectedFrameLength - 3;
  
  // Data section: from byte 4 to ETX (byte 20 in your case)
  // Skip fixed header (7 bytes typically: src, dest, cmd, subcmd, reserved x3)
  uint8_t dataStart = 11;  // After 7-byte header (starting at position 4)
  uint8_t dataEnd = etxPos;
  
  // Track which UI elements to update
  uint8_t updatedIDs[20];
  uint8_t updateCount = 0;
  
  // Parse data fields
  for (uint8_t j = dataStart; j < dataEnd;) {
    if (j >= dataEnd) break;
    
    uint8_t id = serialBuffer[j++];
    
    switch (id) {
      case ID_SOC:
        if (j < dataEnd) {
          dashData.soc = serialBuffer[j++];
          updatedIDs[updateCount++] = id;
        }
        break;
        
      case ID_VOLTAGE:
        if (j + 1 < dataEnd) {
          uint16_t v = (serialBuffer[j] << 8) | serialBuffer[j + 1];
          dashData.voltage = v * 0.01f;
          j += 2;
          updatedIDs[updateCount++] = id;
        }
        break;
      
      case ID_CURRENT:
        if (j + 1 < dataEnd) {
          uint16_t c = (serialBuffer[j] << 8) | serialBuffer[j + 1];
          dashData.current = (c & 0x8000) ? -(c & 0x7FFF) * 0.01f : c * 0.01f;
          j += 2;
          updatedIDs[updateCount++] = id;
        }
        break;
      
      case ID_TEMP:
        if (j + 1 < dataEnd) {
          uint16_t t = (serialBuffer[j] << 8) | serialBuffer[j + 1];
          dashData.battery_temp = (int)(t * 0.1f);
          j += 2;
          updatedIDs[updateCount++] = id;
        }
        break;
      
      case ID_SPEED:
        if (j + 1 < dataEnd) {
          uint16_t s = (serialBuffer[j] << 8) | serialBuffer[j + 1];
          dashData.speed = (int)(s * 0.1f);
          j += 2;
          updatedIDs[updateCount++] = id;
        }
        break;
      
      case ID_MODE:
        if (j < dataEnd) {
          uint8_t m = serialBuffer[j++];
          if (m == MODE_ECO) dashData.mode = "Eco";
          else if (m == MODE_CITY) dashData.mode = "City";
          else if (m == MODE_SPORT) dashData.mode = "Sport";
          updatedIDs[updateCount++] = id;
        }
        break;
      
      case ID_ARMED:
        if (j < dataEnd) {
          uint8_t a = serialBuffer[j++];
          dashData.status = a ? "ARMED" : "DISARMED";
          updatedIDs[updateCount++] = id;
        }
        break;
      
      case ID_RANGE:
        if (j + 1 < dataEnd) {
          uint16_t r = (serialBuffer[j] << 8) | serialBuffer[j + 1];
          dashData.range = (int)(r * 0.1f);
          j += 2;
          updatedIDs[updateCount++] = id;
        }
        break;
      
      case ID_CONSUMPTION:
        if (j + 1 < dataEnd) {
          uint16_t c = (serialBuffer[j] << 8) | serialBuffer[j + 1];
          dashData.avg_wkm = (int)(c * 0.1f);
          j += 2;
          updatedIDs[updateCount++] = id;
        }
        break;
      
      case ID_AMBIENT_TEMP:
        if (j + 1 < dataEnd) {
          uint16_t t = (serialBuffer[j] << 8) | serialBuffer[j + 1];
          dashData.motor_temp = (int)(t * 0.1f);
          j += 2;
          updatedIDs[updateCount++] = id;
        }
        break;
      
      case ID_TRIP:
        if (j + 1 < dataEnd) {
          uint16_t t = (serialBuffer[j] << 8) | serialBuffer[j + 1];
          dashData.trip = (int)(t * 0.1f);
          j += 2;
          updatedIDs[updateCount++] = id;
        }
        break;
      
      case ID_ODOMETER:
        if (j + 3 < dataEnd) {
          uint32_t o = (serialBuffer[j] << 24) | (serialBuffer[j + 1] << 16) | 
                      (serialBuffer[j + 2] << 8) | serialBuffer[j + 3];
          dashData.odo = (int)(o * 0.1f);
          j += 4;
          updatedIDs[updateCount++] = id;
        }
        break;
      
      case ID_AVG_SPEED:
        if (j + 1 < dataEnd) {
          uint16_t as = (serialBuffer[j] << 8) | serialBuffer[j + 1];
          dashData.avg_kmh = (int)(as * 0.1f);
          j += 2;
          updatedIDs[updateCount++] = id;
        }
        break;

      default:
        // Unknown ID - skip safely
        if (id == ID_ODOMETER) {
          j += 4;
        } else if (id >= 0x80 && id <= 0x8F) {
          j += 2;
        } else {
          j++;
        }
        break;
    }
  }
  
  // Update only changed UI elements
  for (uint8_t k = 0; k < updateCount; k++) {
    update_ui_element(updatedIDs[k]);
  }
  
  // Single display refresh
  lv_refr_now(disp);
}