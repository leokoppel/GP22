#include "GP22.h"
#include <SPI.h>

volatile bool _intFlag = 0; // whether interrupt pin is LOW

/* Initialize SPI connection (using default hardware SPI pins) and reset the TDC.
 * INT_Pin is the input pin connected to the TDC's INT (intrrupt flag) output */
GP22::GP22(int pinInt, bool debug, Print& outStream) :
  _pinInt(pinInt),
  _bHardwareInterrupt(0),
  _ISRfunc(pinChangeISR),
  _ISRmode(CHANGE),
  _bDebug(debug),
  _serial(outStream),
  _configRegistersTemp()
{
  for (int i = 0; i < CFG_REGISTER_MAX; ++i) {
    _configRegisters[i] = CFG_DEFAULT_BITMASKS[i];
  }
}

GP22::~GP22()
{
  SPI.end();
  if (_bHardwareInterrupt) {
    detachInterrupt(digitalPinToInterrupt(_pinInt));
  }
}

/* Setup the interrupt and SPI pins and reset the device */
void GP22::init()
{
  pinMode(_pinInt, INPUT_PULLUP);
  //digitalWrite(_pinInt, HIGH);

  attachDefaultInterruptFunc();

  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1); //CPOL=0,CPHA=1
  SPI.setClockDivider(SPI_CLOCK_DIV2); // 16Mhz clock / 2 = 8Mhz
  digitalWrite(SS, HIGH);
  SPI.begin();

  /* Send "power on reset" command */
  sendOpcode(OPCODE_POWER_ON_RESET);

}

/* Interrupt Service Routine attached to INT0 vector */
void GP22::pinChangeISR()
{
  _intFlag = !_intFlag;
}

/* Debug ISR with serial prints */
void GP22::pinChangeISR_debug()
{
  _intFlag = !_intFlag;
  if (_intFlag) Serial.print("\\");
  else Serial.print("/");
}

/* Check whether interrupt pin is set, using hardware or fake */
inline bool GP22::checkInterrupt()
{
  if (_bHardwareInterrupt) {
    return _intFlag;
  }
  else {
    return (digitalRead(_pinInt) == LOW);
  }
}

void GP22::attachDefaultInterruptFunc()
{
  /* Choose between hardware interrupts or "faking it" */
  const int intNum = digitalPinToInterrupt(_pinInt);

  detachInterrupt(intNum);
  

  if (intNum == NOT_AN_INTERRUPT) {
    _bHardwareInterrupt = 0;
    _intFlag = !digitalRead(_pinInt);
  } else {
    // clear any pending interrupts
    EIFR = bit(intNum);

    _intFlag = !digitalRead(_pinInt);
    _bHardwareInterrupt = 1;

    if (_bDebug) {
      attachInterrupt(intNum, pinChangeISR_debug, CHANGE);
    } else {
      attachInterrupt(intNum, pinChangeISR, CHANGE);
    }
  }

  if (_bDebug) {
    _serial.println(F("attached pinChangeISR_debug"));

  }

}

void GP22::attachPreviousInterruptFunc()
{
  const int intNum = digitalPinToInterrupt(_pinInt);

  //TODO: check for intNum == NOT_AN_INTERRUPT

  // clear any pending interrupts
  EIFR = bit(intNum);
    
  attachInterrupt(intNum, _ISRfunc, _ISRmode);

  if (_bDebug) {
    _serial.print(F("attached ISR at 0x"));
    _serial.println((uint32_t)_ISRfunc, HEX);
  }
}
void GP22::attachInterruptFunc(void (*func)(void), int mode)
{
  _ISRfunc = func;
  _ISRmode = mode;

  attachPreviousInterruptFunc();
}


/* Test SPI communication to chip by writing to register 1 and reading read
 * register 5. Return true on success, false on error */
bool GP22::testCommunication()
{
  bool ok = 1;
  unsigned long start_us = micros();

  // Save highest 8 bytes of register 1
  byte initialReg1 = readRegister(READ_REG_1) >> 24;

  // For test data use a scramble of the original
  byte testInput = initialReg1 ^ 0xff + 123;

  // Write the test data, and see if it changes
  writeRegister(1, (uint32_t)(testInput) << 24);
  byte testResult = readRegister(READ_REG_1);

  if (testResult != testInput) ok = 0;

  // Restore the original value of register 1 (and check again)
  writeRegister(1, (uint32_t)initialReg1 << 24);
  byte testResult2 = readRegister(READ_REG_1);

  if (testResult2 != initialReg1) ok = 0;

  unsigned long duration_us = micros() - start_us;

  if (_bDebug) {
    _serial.print(F("Test took "));
    _serial.print(duration_us);
    _serial.println(F(" us"));
  }

  return ok;
}

/* Print IDs from registers */
void GP22::printIDs()
{
  uint64_t ids = readNBytes(OPCODE_READ_ID, 7);
  _serial.print(F("IDs = "));
  _serial.print((uint32_t)(ids >> 32), HEX);
  _serial.println((uint32_t)ids, HEX);
}

/* Print status from status register */
void GP22::printStatus()
{
  uint16_t stat = readRegister(READ_STAT);
  _serial.print(F("Status = 0x"));
  _serial.println(stat, HEX);

  _serial.print(F("ALU pointer "));
  _serial.println((stat & STAT_ALU_OP_PTR * 0x7) / STAT_ALU_OP_PTR);

  _serial.print(F("Hits (ch1, ch2) = "));
  _serial.print((stat & STAT_HITS_CH1 * 0x7) / STAT_HITS_CH1);
  _serial.print(F(","));
  _serial.println((stat & STAT_HITS_CH2 * 0x7) / STAT_HITS_CH2);

  if (stat & STAT_TIMEOUT_TDC) {
    _serial.println(F("Timeout TDC"));
  }
  if (stat & STATUS_TIMEOUT_PRECOUNTER) {
    _serial.println(F("Timeout Precounter"));
  }
  if (stat & STAT_ERROR_OPEN) {
    _serial.println(F("Error open"));
  }
  if (stat & STAT_ERROR_SHORT) {
    _serial.println(F("Error short"));
  }
  if (stat & STAT_EEPROM_ERROR) {
    _serial.println(F("EEPROM error (corrected)"));
  }
  if (stat & STAT_EEPROM_DED) {
    _serial.println(F("EEPROM double error (not corrected)"));
  }
  if (stat & STAT_EEPROM_EQ_CREG) {
    _serial.println(F("EEPROM matches config registers"));
  }
}

/* Print 8-bit data as hex */
void GP22::printHexNum(uint8_t *data, size_t len)
{
  char tmp[2 * len + 1];
  byte first;
  int j = 0;
  for (int8_t i = len - 1; i >= 0; --i)
  {
    first = (data[i] >> 4) | 48;
    if (first > 57) tmp[j] = first + (byte)39;
    else tmp[j] = first ;
    j++;

    first = (data[i] & 0x0F) | 48;
    if (first > 57) tmp[j] = first + (byte)39;
    else tmp[j] = first;
    j++;
  }

  tmp[2 * len] = 0;
  _serial.print(tmp);
}


/* Print (cached) contents of config registers */
void GP22::printConfigRegisters()
{
  _serial.println();
  for (int i = 0; i < CFG_REGISTER_MAX; ++i) {
    _serial.print(F("Register "));
    _serial.print(i);
    _serial.print(F(":\t0x"));
    uint8_t buf[16];
    printHexNum((uint8_t*) & (_configRegisters[i]), 4);
    _serial.println();
  }
  _serial.println();

}

/* Print all 4 output registers */
void GP22::printOutputRegisters()
{

  /* Read output registers */
  uint32_t rawResults[4];
  float floatResults[4];

  _serial.println(F("Read registers: "));
  for (int i = 0; i < 4; ++i) {
    uint32_t rawRes = readRegister((eReadRegister)i);
    _serial.print(i);
    _serial.print(F(": "));
    _serial.print(rawRes, HEX);
    _serial.print(F("\t"));
    _serial.println(fixedPoint16ToFloat(rawRes), 6);
  }

  _serial.println();


}


/* Send a stand-alone opcode
 * (can't be followed by reading bytes, as SS is set low then high) */
void GP22::sendOpcode(eOpcode opcode)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

    digitalWriteFast(SS, LOW);
    SPI.transfer(opcode);
    digitalWriteFast(SS, HIGH);

  }
}

/* Writes the opcode, then the n lowest bytes in data, over SPI.
 * n can be 0 to 4 */
void GP22::writeNBytes(byte opcode, uint32_t data, uint8_t n)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    digitalWriteFast(SS, LOW);

    //transfer the opcode
    SPI.transfer(opcode);

    //read the remaning byte
    for (int shift = (n - 1) * 8; shift >= 0; shift -= 8)
    {
      SPI.transfer((byte)(data >> shift));
    }

    digitalWriteFast(SS, HIGH);
  }
}

/* Writes the opcode, then reads and returns n bytes
 * n can be 1 to 4
 * Note that this doesn't work for reading IDs (7 bytes)
 *  -- only the lowest 4 bytes are returned.
 */
uint32_t GP22::readNBytes(byte opcode, uint8_t n)
{
  byte buf;
  uint32_t res = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    digitalWriteFast(SS, LOW);

    //transfer the opcode
    SPI.transfer(opcode);

    //read the remaning bytes
    for (int shift = (n - 1) * 8; shift >= 0; shift -= 8)
    {
      buf = SPI.transfer(0x00);
      res |= ((uint32_t)buf << shift);
    }

    digitalWriteFast(SS, HIGH);
  }

  if (0 && _bDebug) {
    _serial.print(F("Read opcode "));
    _serial.print(opcode);
    _serial.print(F(": "));
    printHexNum((uint8_t*)&res, 4);
    _serial.println();
  }

  return res;
}

/* Write data into the register at address */
void GP22::writeRegister(byte address, reg_t data)
{
  writeNBytes(OPCODE_WRITE_ADDRESS + address, data, 4);

  /* Save local copy of register */
  _configRegisters[address] = data;

  if (0 && _bDebug) {
    _serial.print(F("Wrote to reg "));
    _serial.print(address);
    _serial.print(F(": "));
    printHexNum((uint8_t*)&data, 4);
    _serial.println();
  }
}

/* Read the register at address */
uint32_t GP22::readRegister(eReadRegister reg)
{
  return readNBytes(OPCODE_READ_ADDRESS | READ_REGISTER_ADDRS[reg],
                    READ_REGISTER_LENGTHS[reg] / 8);
}

/* Convenience function
 * Read the register & convert to float from 16:16 fixed-point format */
float GP22::readResult(int address)
{
  return fixedPoint16ToFloat(readRegister((eReadRegister)address));
}

/* Read only 2 bytes, for an uncalibrated result in meas. mode 1
 * Return the float representation
 */
int16_t GP22::readUncalibratedResult(int address)
{
  return readNBytes(OPCODE_READ_ADDRESS | address, 2);
}


/* Wait for the INT pin to go low.
   Return the number of microseconds passed (0 if low on first read),
   or -1 if the timeout reached. timeout_us=0 --> no timeout*/
long GP22::waitForInterrupt(long timeout_us)
{
  unsigned long start_time = micros();
  unsigned long waited_us = 0;
  while (!checkInterrupt()) {
    waited_us = micros() - start_time;
    if (timeout_us && waited_us > timeout_us) {
      return -1;
    }
  }
  return waited_us;
}

/* Read the status register for the ALU pointer, ALU_OP_PTR.
 *
 * Note ALU_OP_PTR points to the next "empty" register, where the following
 * calculation will be stored. 0 means no measurements yet performed.
 * "After a measurement ALU_OP_PTR minus 1 will point to the ALU result,"
 */
int GP22::getALUPointer()
{
  return ((readRegister(READ_STAT) & STAT_ALU_OP_PTR * 0x7) / STAT_ALU_OP_PTR);
}

/* Get CLOCK_FACTOR (1, 2, or 3) based on the local register cache */
int8_t GP22::getClockFactor()
{
  int DIV_CLKHS = (_configRegisters[0] & CFG0_DIV_CLKHS_0 * 0x3) / CFG0_DIV_CLKHS_0;
  return (DIV_CLKHS < 3 ? (1 << DIV_CLKHS) : (1 << 2));
}


/* Return the expected number of cycles during CAL_RESONATOR procedure
 * This depends on DIV_CLKHS which is obtained from local register cache
 * It also depends on ANZ_PER_CALRES which is assumed constant, as set in getResonatorCycles()
 */
float GP22::getResonatorCyclesTheoretical()
{
  // Need DIV_CLKHS for theoretical result.
  const int CLOCK_FACTOR = getClockFactor();

  // Theoretical result
  return 2.0 / REF_CLK_FREQ_HZ * (1 << DEFAULT_ANZ_PER_CALRES) * HS_CLK_FREQ_HZ / CLOCK_FACTOR;

}

/* For calibrating the ceramic resonator.
 * Return the measured cycles, or 0 for failure
 */
float GP22::getResonatorCycles()
{
  float res_meas = 0;

  // Need EN_AUTOCALC=0 for this. Save a temp copy of the register and disable
  // Also need SEL_TIMO_MB2=3 (discovered by experimentation, not in datasheet)

  writeRegister(3, _configRegistersTemp[3] & ~CFG3_EN_AUTOCALC_MB2 | CFG3_SEL_TIMO_MB2_0 * 0x3);

  // Measure actual value
  sendOpcode(OPCODE_INIT);
  sendOpcode(OPCODE_START_CAL_RESONATOR);
  
  int w = waitForInterrupt(1000000);

  // dirty debug: this just helps it run,
  // for some reason, while measurements are already running
  delay(10);

  if (w == -1) {
    if (_bDebug) {
      _serial.println(F("Failed to capture measurement. Resonator calibration failed.\n"));
      printStatus();
    }
  } else {
    // Read measured value (convert from 16:16 fixed point format)
    res_meas = readResult(READ_RES_0);
    if (_bDebug) {
      _serial.print(F("Resonator: "));
      _serial.println(res_meas, 4);
    }
  }

  return res_meas;
}

float GP22::getResonatorCorrectionFactor() const
{
  return float(_calibration.resonator_theor_cycles) / _calibration.resonator_meas_cycles;
}

float GP22::getCycleTime_ns() const
{
  return (_calibration.Tref_theor_ns * _calibration.clock_factor
          * _calibration.resonator_theor_cycles / _calibration.resonator_meas_cycles
          / _calibration.tdc_cal_cycles);
}

/* Get the TDC calibration Cal2-Cal1
 * Note a regular measurement is needed for this to work
 * i.e. something must be happening on the Start & Stop pins
 * On success, return the positive calibration result
 * On failure, return a negative eCalibrationResult error code
 */
int16_t GP22::getCalCycles()
{
  int16_t res = E_CAL_OK;

  sendOpcode(OPCODE_INIT);

  // Turn off NO_CAL_AUTO and EN_FAST_INIT
  writeRegister(0, _configRegisters[0] &~(CFG0_NO_CAL_AUTO));
  writeRegister(1, CFG_KEEP_DEFAULT_BITMASKS[1]
                | CFG1_HITIN1_0 * 1   // Wait for 1 hit on channel 1
                | CFG1_HITIN2_0 * 0   // Wait for 0 hits on channel 2
                | CFG1_HIT1_0 * 1   // Calculate 1st Stop Ch1 - Start
                | CFG1_HIT2_0 * 0);

  // Set INT on hits
  writeRegister(2, CFG2_EN_INT_HITS);

  if (_bDebug) {
    _serial.println(F("\n\nCalibrating TDC."));

  }

  // Set INT on hits
  writeRegister(2, CFG_KEEP_DEFAULT_BITMASKS[2] | CFG2_EN_INT_HITS);

  sendOpcode(OPCODE_INIT);
  sendOpcode(OPCODE_START_CAL_TDC);

  // Calibration data are addressed only after the next regular measurement
  // Wait for interrupt meaning hits arrived
  int w = waitForInterrupt(1000000);
  if (w == -1) {
    if (_bDebug) {
      _serial.println(F("Failed to capture hits. Calibration failed.\n"));
      printStatus();
    }
    res = E_CAL_FAIL_NO_HITS;
  }

  if (res == E_CAL_OK) {

    // Set INT on timeout or ALU
    writeRegister(2, CFG_KEEP_DEFAULT_BITMASKS[2] | CFG2_EN_INT_ALU | CFG2_EN_INT_TDC_TIMEOUT);

    sendOpcode(OPCODE_INIT);

    int w2 = waitForInterrupt(1000000);
    if (w2 == -1) {
      res = E_CAL_FAIL_NO_MEASUREMENT;
      if (_bDebug) {
        _serial.println(F("Failed to wait for measurement. Calibration failed.\n"));
        printStatus();
      }
    }
  }

  if (readRegister(READ_STAT) & STAT_TIMEOUT_TDC) {
    res = E_CAL_FAIL_TDC_TIMEOUT;
    if (_bDebug) {
      _serial.println(F("TDC Timeout on cal measurement attempt."));
      printOutputRegisters();
      printStatus();
    }
  }

  if (res == E_CAL_OK) {
    if (_bDebug) {
      // Read regular measurement
      float res0 = readUncalibratedResult(0);
      _serial.print(F("Regular measurement ("));
      _serial.print(getALUPointer());
      _serial.print(F("): "));
      _serial.println(res0, 4);
    }

    /* Read calibration (Cal2-Cal1), and calculate real time  */
    writeRegister(1, CFG_KEEP_DEFAULT_BITMASKS[1]
                  | CFG1_HIT1_0 * 7 // Cal2
                  | CFG1_HIT2_0 * 6 // Cal1
                 );

    int w2 = waitForInterrupt(500000);
    if (w2 == -1) {
      if (_bDebug) {
        _serial.println(F("Failed to wait for calibration. Calibration failed.\n"));
        printStatus();
      }
      res = E_CAL_FAIL_WAIT;
    } else {
      if (_bDebug) {
        _serial.print(F("\n"));
        printStatus();
        _serial.print(F("\n"));
      }
      
      /* Looks like success */
      res = readUncalibratedResult(getALUPointer() - 1);
      
      /* Check that value is not garbage to avoid conflict with error codes */
      if(res <=0) {
          res = E_CAL_FAIL_GARBAGE;
      }

  }
  
      
      
  }

  if (_bDebug) {
    _serial.print(F("Cal: "));
    _serial.println(res);
  }

  return res;

}

/* Updates the calibration structure
 * Returns eCalibrationResult with E_OK for success or an error code for failure
 * The _calibration struct is only updated if E_OK returned
 */
eCalibrationResult GP22::updateCalibration()
{
  eCalibrationResult res = E_CAL_OK;

  tempSaveRegisters();

  attachDefaultInterruptFunc();

  // Turn off NO_CAL_AUTO and EN_FAST_INIT
  writeRegister(0, (_configRegisters[0] &~(CFG0_NO_CAL_AUTO)
                    &~(CFG0_ANZ_PER_CALRES_0 * 3))
                | CFG0_ANZ_PER_CALRES_0 * DEFAULT_ANZ_PER_CALRES
               );

  writeRegister(1, CFG_KEEP_DEFAULT_BITMASKS[1]
                | CFG1_HITIN1_0 * 1   // Wait for 1 hit on channel 1
                | CFG1_HITIN2_0 * 0   // Wait for 0 hits on channel 2
                | CFG1_HIT1_0 * 1   // Calculate 1st Stop Ch1 - Start
                | CFG1_HIT2_0 * 0);

  // Calibrate resonator to 32 kHz clock
  const float res_theor = getResonatorCyclesTheoretical();

  if (_bDebug) {
    _serial.print(F("\nCalibrating resonator. Theoretical: "));
    _serial.println(res_theor, 4);
  }

  const float res_meas = getResonatorCycles();

  // Check failure
  if (res_theor == 0 || res_meas == 0)  {
    res = E_CAL_FAIL_RESONATOR;
  }

  // Calculate correction factor
  float corrFact = res_theor / res_meas;

  // Check reasonable correction factor
  if (corrFact < 0.5 || corrFact > 1.5)  {
    res = E_CAL_FAIL_RESONATOR;
  }

  // Get TDC calibration cycles
  const int16_t cal = getCalCycles();
  // Use of negative return value for error codes (TODO)
  if(cal < 0) {
      res = (eCalibrationResult)cal;
  }

  if (res == E_CAL_OK) {
    _calibration.Tref_theor_ns = HS_CLK_PERIOD_NS;
    _calibration.clock_factor = getClockFactor();
    _calibration.resonator_theor_cycles = res_theor;
    _calibration.resonator_meas_cycles = res_meas;
    _calibration.tdc_cal_cycles = cal;
  }


  if (_bDebug) {
    _serial.print(F("Corr: "));
    _serial.println(corrFact, 5);
    _serial.print(F("Cal freq: "));
    _serial.println(HS_CLK_FREQ_HZ * corrFact);
  }

  // Write back original register values
  attachPreviousInterruptFunc();
  tempRestoreRegisters();
  sendOpcode(OPCODE_INIT);

  return res;
}

void GP22::tempSaveRegisters()
{
  for (int i = 0; i < CFG_REGISTER_MAX; ++i) {
    _configRegistersTemp[i] = _configRegisters[i];
  }
}

void GP22::tempRestoreRegisters()
{
  for (int i = 0; i < CFG_REGISTER_MAX; ++i) {
    writeRegister(i, _configRegistersTemp[i]);
  }
}
