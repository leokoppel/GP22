/*
 * Comm protocol definitions between PC and Arduino
 * (not to TDC)
 */
#ifndef TDCTALK_H
#define TDCTALK_H

#include "GP22.h"
#include <OneWire.h>

/* Serial read/write convenience functions */
inline int16_t read_int16()
{
  /* Read short int from 2 bytes of serial */
  char buf[2];
  Serial.readBytes(buf, 2);
  return *((int16_t *)buf);
}

inline int8_t read_int8()
{
  return Serial.read();
}

inline uint8_t read_uint8()
{
  return (uint8_t)Serial.read();
}

inline void write_int8(const int8_t b)
{
  Serial.write(b);
}

inline void write_uint8(const uint8_t B)
{
  Serial.write(B);
}

inline void write_uint16(const uint16_t h)
{
  Serial.write((byte*)(&h), 2);
}

inline void write_float(const float f)
{
  Serial.write((byte*)(&f), 4);
}

enum ECommand {
  CMD_GET_ARDUINO_STATUS = 0x00,
  CMD_GET_CYCLE_TIME_NS = 0x01,
  CMD_GET_UNCAL_RESULTS = 0x02,
  CMD_CALIBRATE = 0x03,
};

// Status codes to return to PC / Labview program
// TODO: Note for calibration failures we just send the negative eCalibrationResult from GP22.h
// This is quick and lazy compared to duplicating the codes in both places or some combined system    
enum ECommStatus {
  E_OK  = 0,
  E_ARDUINO_OVERFLOW = 1,
  E_UNKNOWN_ERROR = 2,
  E_FAILED_SPI = 3,
};

void respondToCommand(ECommand cmd);

float DS_update_temp_celcius();

/* use globals */
extern GP22 tdc;
extern OneWire onewire;

const uint16_t MEAS_BUF_LEN = 600;
extern volatile uint16_t measbuf[];
extern volatile uint16_t meas_index;
extern const bool DEBUG;
extern float cycleFactor_ns;
extern ECommStatus ardStatus;

const int DIV_CLKHS = 0;
const int CLOCK_FACTOR = DIV_CLKHS < 3 ? (1 << DIV_CLKHS) : (1 << 2);


#endif //TDCTALK_H
