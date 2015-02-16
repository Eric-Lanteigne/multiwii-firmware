#include "Arduino.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Serial.h"
#include "Protocol.h"
#include "RX.h"

/************************************** MultiWii Serial Protocol *******************************************************/
// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
//range id [50-99] won't be assigned and can therefore be used for any custom multiwii fork without further MSP id conflict

#define MSP_PRIVATE              1     //in+out message      to be used for a generic framework : MSP + function code (LIST/GET/SET) + data. no code yet

#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX

#define MSP_SET_RAW_RC           200   //in message          8 rc chan

#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

static uint8_t CURRENTPORT=0;

#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];
static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

void evaluateOtherData(uint8_t sr);
#ifndef SUPPRESS_ALL_SERIAL_MSP
void evaluateCommand();
#endif

#define BIND_CAPABLE 0;  //Used for Spektrum today; can be used in the future for any RX type that needs a bind and has a MultiWii module. 
#if defined(SPEK_BIND)
  #define BIND_CAPABLE 1;
#endif
// Capability is bit flags; next defines should be 2, 4, 8...

const uint32_t capability = 0+BIND_CAPABLE;

uint8_t read8()  {
  return inBuf[indRX[CURRENTPORT]++][CURRENTPORT]&0xff;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}

void serialize8(uint8_t a) {
  SerialSerialize(CURRENTPORT,a);
  checksum[CURRENTPORT] ^= a;
}
void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}
void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum[CURRENTPORT] = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP[CURRENTPORT]);
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) {
  headSerialResponse(1, s);
}

void tailSerialReply() {
  serialize8(checksum[CURRENTPORT]);UartSendData(CURRENTPORT);
}

void serializeNames(PGM_P s) {
  headSerialReply(strlen_P(s));
  for (PGM_P c = s; pgm_read_byte(c); c++) {
    serialize8(pgm_read_byte(c));
  }
}

enum MSP_protocol_bytes {
  IDLE,
  HEADER_START,
  HEADER_M,
  HEADER_ARROW,
  HEADER_SIZE,
  HEADER_CMD
};

void serialCom() {
  uint8_t c,cc,port,state,bytesTXBuff;
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static uint8_t c_state[UART_NUMBER];

  for(port=0;port<UART_NUMBER;port++) {
    CURRENTPORT=port;
    #define GPS_COND
    #if defined(GPS_SERIAL) && (UART_NUMBER > 1)
      #define GPS_COND  && (GPS_SERIAL != port)
    #endif
    #define RX_COND
    #if (defined(SPEKTRUM) || defined(SBUS) || defined(SUMD)) && (UART_NUMBER > 1)
      #define RX_COND && (RX_SERIAL_PORT != port)
    #endif
    cc = SerialAvailable(port);
    while (cc-- GPS_COND RX_COND) {
      bytesTXBuff = SerialUsedTXBuff(port); // indicates the number of occupied bytes in TX buffer
      if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
      c = SerialRead(port);
      #ifdef SUPPRESS_ALL_SERIAL_MSP
        // no MSP handling, so go directly
        evaluateOtherData(c);
      #else
        state = c_state[port];
        // regular data handling to detect and handle MSP and other data
        if (state == IDLE) {
          if (c=='$') state = HEADER_START;
          else evaluateOtherData(c); // evaluate all other incoming serial data
        } else if (state == HEADER_START) {
          state = (c=='M') ? HEADER_M : IDLE;
        } else if (state == HEADER_M) {
          state = (c=='<') ? HEADER_ARROW : IDLE;
        } else if (state == HEADER_ARROW) {
          if (c > INBUF_SIZE) {  // now we are expecting the payload size
            state = IDLE;
            continue;
          }
          dataSize[port] = c;
          offset[port] = 0;
          checksum[port] = 0;
          indRX[port] = 0;
          checksum[port] ^= c;
          state = HEADER_SIZE;  // the command is to follow
        } else if (state == HEADER_SIZE) {
          cmdMSP[port] = c;
          checksum[port] ^= c;
          state = HEADER_CMD;
        } else if (state == HEADER_CMD) {
          if (offset[port] < dataSize[port]) {
            checksum[port] ^= c;
            inBuf[offset[port]++][port] = c;
          } else {
            if (checksum[port] == c) {  // compare calculated and transferred checksum
              evaluateCommand();  // we got a valid packet, evaluate it
            }
            state = IDLE;
            cc = 0; // no more than one MSP per port and per cycle
          }
        }
        c_state[port] = state;
      #endif // SUPPRESS_ALL_SERIAL_MSP
    }
  }
}

void  s_struct(uint8_t *cb,uint8_t siz) {
  headSerialReply(siz);
  while(siz--) serialize8(*cb++);
}

void s_struct_w(uint8_t *cb,uint8_t siz) {
  headSerialReply(0);
  while(siz--) *cb++ = read8();
}

#ifndef SUPPRESS_ALL_SERIAL_MSP
void evaluateCommand() {
  uint32_t tmp=0; 

  switch(cmdMSP[CURRENTPORT]) {
   case MSP_PRIVATE:
     headSerialError(0); // we don't have any custom msp currently, so tell the gui we do not use that
     break;
   case MSP_SET_RAW_RC:
     s_struct_w((uint8_t*)&rcSerial,16);
     rcSerialCount = 200; // 1s transition 
     break;
   case MSP_STATUS:
     struct {
       uint16_t cycleTime,i2c_errors_count,sensor;
       uint32_t flag;
       uint8_t set;
     } st;
     st.cycleTime        = cycleTime;
     s_struct((uint8_t*)&st,11);
     break;
   case MSP_RC:
     s_struct((uint8_t*)&rcData,RC_CHANS*2);
     break;
   case MSP_ANALOG:
     s_struct((uint8_t*)&analog,7);
     break;
   case MSP_DEBUG:
     s_struct((uint8_t*)&debug,8);
     break;
   default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
     headSerialError(0);
     break;
  }
  tailSerialReply();
}
#endif // SUPPRESS_ALL_SERIAL_MSP

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr) {
  #ifndef SUPPRESS_OTHER_SERIAL_COMMANDS
    switch (sr) {
    }
  #endif // SUPPRESS_OTHER_SERIAL_COMMANDS
}

void SerialWrite16(uint8_t port, int16_t val)
{
  CURRENTPORT=port;
  serialize16(val);UartSendData(port);
}
