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
#include "Servo.h"

#include <avr/pgmspace.h>

uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop

analog_t analog;
int16_t  debug[4];

int16_t rcData[RC_CHANS] = {0, 0, 0, 0, 0, 0, 0, 0};    // interval [1000;2000]
int16_t rcSerial[8];         // interval [1000;2000] - is rcData coming from MSP
uint8_t rcSerialCount = 0;   // a counter to select legacy RX when there is no more MSP rc serial data

//Servo definitions
Servo MotorL,MotorR,Roll,Pitch,Yaw,Rail;
#define MotorL_pin 10 //Controlled by throttle
#define MotorR_pin 13 //Controlled by throttle
#define Roll_pin 6 //Controlled by roll
#define Pitch_pin 11 //Controlled by pitch
#define Yaw_pin 5 //Controlled by yaw
#define Rail_pin 9 //Controlled by AUX1

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

	//Throttle
	//Experimentally, the left motor starts at 1181, right motor at 1138 (diff 43) 
	if(rcData[THROTTLE]==0 ){
		MotorL.detach();
		MotorR.detach();
	}else{

		if(!MotorL.attached())MotorL.attach(MotorL_pin);
		MotorL.writeMicroseconds(rcData[THROTTLE]+43+100);
		if(!MotorR.attached())MotorR.attach(MotorR_pin);
		MotorR.writeMicroseconds(rcData[THROTTLE]+100);
	}

	//Roll
	if(rcData[ROLL]==0){
		Roll.detach();
	}else{
		if(!Roll.attached())Roll.attach(Roll_pin);
		Roll.writeMicroseconds(rcData[ROLL]);
	}

	//Pitch
	if(rcData[PITCH]==0){
		Pitch.detach();
	}else{
		if(!Pitch.attached())Pitch.attach(Pitch_pin);
		Pitch.writeMicroseconds(rcData[PITCH]);
	}

	//Yaw
	if(rcData[YAW]==0){
		Yaw.detach();
	}else{
		if(!Yaw.attached())Yaw.attach(Yaw_pin);
		Yaw.writeMicroseconds(rcData[YAW]);
	}

	//Rail (disabled also when centered, so it doesn't drift)
	//Experimentally centered at 1377
	if(rcData[AUX1]==0 || rcData[AUX1]==1500){
		Rail.detach();
	}else{
		if(!Rail.attached())Rail.attach(Rail_pin);
		Rail.writeMicroseconds(rcData[AUX1]-1500+1377);
	}
  }
 
  serialCom();

  // Measure loop rate
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;
}
