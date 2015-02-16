#ifndef TYPES_H_
#define TYPES_H_

enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2,
  AUX3,
  AUX4,
  AUX5,
  AUX6,
  AUX7,
  AUX8
};

typedef struct {
  uint8_t  vbat;               // battery voltage in 0.1V steps
  uint16_t intPowerMeterSum;
  uint16_t rssi;              // range: [0;1023]
  uint16_t amperage;          // 1unit == 100mA
  uint16_t watts;             // 1unit == 1W
} analog_t;

#endif /* TYPES_H_ */
