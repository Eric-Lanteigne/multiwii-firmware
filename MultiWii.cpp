/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
November  2013     V2.3
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include <avr/io.h>

#include "Arduino.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "RX.h"
#include "Serial.h"
#include "Protocol.h"

#include <avr/pgmspace.h>

uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop

analog_t analog;
int16_t  debug[4];

int16_t rcData[RC_CHANS] = {1500, 1500,1500, 1500,1500, 1500,1500, 1500};    // interval [1000;2000]
int16_t rcSerial[8];         // interval [1000;2000] - is rcData coming from MSP
uint8_t rcSerialCount = 0;   // a counter to select legacy RX when there is no more MSP rc serial data

void setup() {
  SerialOpen(0,SERIAL_COM_SPEED);
  SerialOpen(1,SERIAL_COM_SPEED);
  
  previousTime = micros();
}

// ******** Main Loop *********
void loop () {
  static uint16_t rcTime  = 0;

  if ((int16_t)(currentTime-rcTime) >0 ) { // 50Hz
    rcTime = currentTime + 20000;
    computeRC();
  }
 
  serialCom();

  // Measure loop rate
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;
}
