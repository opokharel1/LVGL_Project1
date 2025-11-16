#pragma once
// rs485.h - RS485 parsing API

#include "shared.h"
#include <Arduino.h>
#include "ui.h"

#ifdef __cplusplus
extern "C" {
#endif
// ===== Buffer for receiving data =====




// Call from loop() (or from an RS485 task)
void read_rs485_frames(void);
uint16_t calculateChecksum(const uint8_t *data, uint16_t length);
bool validateFrame(uint8_t* frame, uint16_t len);


bool quickValidateFrame(uint8_t* frame, uint16_t len);
void processCompleteFrame(void); // current implementation reads serialBuffer global

#ifdef __cplusplus
}
#endif
