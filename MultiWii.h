#ifndef MULTIWII_H_
#define MULTIWII_H_

#include "types.h"

extern uint32_t currentTime;
extern uint16_t previousTime;
extern uint16_t cycleTime;

extern analog_t analog;
extern int16_t debug[4];

extern int16_t rcData[RC_CHANS];
extern int16_t rcSerial[8];
extern uint8_t rcSerialCount;

#endif /* MULTIWII_H_ */
