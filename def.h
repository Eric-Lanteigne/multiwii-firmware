#ifndef DEF_H_
#define DEF_H_

/**************************************************************************************/
/***************             Proc specific definitions             ********************/
/**************************************************************************************/
// Proc auto detection
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
#define PROMINI
#endif
#if defined(__AVR_ATmega32U4__) || defined(TEENSY20)
#define PROMICRO
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
#define MEGA
#endif

#define SERIAL_COM_SPEED 115200

#if !defined(RX_SERIAL_PORT)
#define RX_SERIAL_PORT           1
#endif
#define USB_CDC_TX               3
#define USB_CDC_RX               2
#define PCINT_PIN_COUNT          4
#define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4)

#define PCINT_RX_PORT                PORTB
#define PCINT_RX_MASK                PCMSK0
#define PCIR_PORT_BIT                (1<<0)
#define RX_PC_INTERRUPT              PCINT0_vect
#define RX_PCINT_PIN_PORT            PINB

#define RC_CHANS 8

#endif /* DEF_H_ */
