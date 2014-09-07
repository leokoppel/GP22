#include "TDCtalk.h"
#include "GP22.h"
#include <OneWire.h>
#include <SPI.h>
#include <avr/io.h>

/* Talk the to TDC-GP22
 *
 * Configure TDC for time-correlated single photon counting,
 * and send time measurements and calibration back to the PC
 * using a custom serial protocol.
 *
 * Test setup (Arduino Uno)
 *     10 -> SS
 *     11 -> MOSI
 *     12 -> MISO
 *     13 -> SCK
 *     2  -> INT
 *     5V -> VCC
 *     GND -> GND
 *     4 -> DS18B20 temperature sensor (optional)
 */

const bool DEBUG = 0;
const int FAST_MODE = 1;

const int PIN_INT = 2;
const int PIN_LED = 5;
const int PIN_TEMP_SENSOR = 4;

GP22 tdc(PIN_INT, DEBUG);
OneWire onewire(PIN_TEMP_SENSOR);

float Tref_ns;
float cycleFactor_ns = 1<< -2;
ECommStatus ardStatus = E_OK;


volatile uint16_t measbuf[MEAS_BUF_LEN];
volatile uint16_t meas_index = 0;


void setup() {

  Serial.begin(DEBUG ? 115200 : 1000000);

  tdc.init();
  
  DS_update_temp_celcius();

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);


  if (DEBUG) {
    Serial.println(F("Starting up"));
  }

  tdc.sendOpcode(OPCODE_INIT);

  // Test SPI read/write to the TDC
  bool res = tdc.testCommunication();
  if (!res) {
    if (DEBUG) {
      Serial.println(F("Read/write test failed!"));
    }
    ardStatus =  E_FAILED_SPI;
  } else {
      if (DEBUG) {
          Serial.println(F("Read/write test succeeded."));
    }
  }
      
      


  if (ardStatus == E_OK) {
    /* Write configuration */
    tdc.writeRegister(0, CFG_KEEP_DEFAULT_BITMASKS[0]
                      | CFG0_MESSB2 * 0                // Measurement mode 1
                      | CFG0_DIV_CLKHS_0 * DIV_CLKHS           // Clock divided by 1
                      | CFG0_START_CLKHS_START_0 * 1   // Oscillator continuously on
                      | CFG0_CALIBRATE * 0               // Calibration on //off
                      | CFG0_NEG_STOP1 * 1                // Stop sensitive to falling edge
                      | CFG0_NO_CAL_AUTO * 1
                      | 0x23
                     );
    tdc.writeRegister(1, CFG_KEEP_DEFAULT_BITMASKS[1]
                      | CFG1_HITIN1_0 * 1   // Wait for 1 hit on channel 1
                      | CFG1_HITIN2_0 * 0   // Wait for 0 hits on channel 2
                      | CFG1_HIT1_0 * 1   // Calculate 1st Stop Ch1 - Start
                      | CFG1_HIT2_0 * 0
                      | CFG1_SEL_TSTO2_0 * 0
                      | CFG1_EN_FAST_INIT * 0
                      | 0x0f
                     );

    tdc.writeRegister(2, CFG_KEEP_DEFAULT_BITMASKS[2]
                      | CFG2_RFEDGE1 * 0      // Ch 1 sensitive to one edge
                      | CFG2_EN_INT_ALU   // Enable interrupt on ALU ready
                      | CFG2_EN_INT_TDC_TIMEOUT   // Enable interrupt on TDC timeout
                      | 0x00
                     );

    tdc.writeRegister(3, CFG_KEEP_DEFAULT_BITMASKS[3]
                      | CFG3_EN_ERR_VAL * 1     // timeout - write 0xffffffff to output register
                      | 0x00
                     );

    tdc.writeRegister(4, CFG_KEEP_DEFAULT_BITMASKS[4]
                      | 0x00
                     );
    tdc.writeRegister(5, CFG_KEEP_DEFAULT_BITMASKS[5]
                      //                      | 0x41
                     );
    tdc.writeRegister(6, CFG_KEEP_DEFAULT_BITMASKS[6]
                      | CFG6_DOUBLE_RES * 0      // Double res. (limits stop inputs to 1)
                      | CFG6_EN_INT_END // interrupt on EEPROM end
                      //                      | 0x3c
                     );
  }

  if (ardStatus == E_OK) {



    /* Get a calibrated Tref */
    if (DEBUG) {
      tdc.printConfigRegisters();
      Serial.println(F("Calibrating..."));
    }
    
    ardStatus = ardStatusFromCalibration();
    
    if(ardStatus == E_OK) {
      cycleFactor_ns = tdc.getCycleTime_ns();


      if (DEBUG) {
        Serial.print(F("1 cycle = "));
        Serial.print(cycleFactor_ns, 4);
        Serial.println(F(" ns"));
      }

    }
  }
  
  if (ardStatus == E_OK) {
      tdc.sendOpcode(OPCODE_INIT);
    
      /* Keep measuring the waveform time */
      tdc.writeRegister(0, CFG_KEEP_DEFAULT_BITMASKS[0]
                        | CFG0_DIV_CLKHS_0 * DIV_CLKHS           // Clock divided by 1
                        | CFG0_START_CLKHS_START_0 * 1   // Oscillator continuously on
                        | CFG0_NO_CAL_AUTO * 1
                        | CFG0_NEG_STOP1 * 0                // Stop sensitive to _  edge
                       );
      tdc.writeRegister(1, CFG_KEEP_DEFAULT_BITMASKS[1]
                        | CFG1_HITIN1_0 * 1   // Wait for 1 hit on channel 1
                        | CFG1_HITIN2_0 * 0   // Wait for 0 hits on channel 2
                        | CFG1_HIT1_0 * 1   // Calculate 1st Stop Ch1 - Start
                        | CFG1_HIT2_0 * 0
                        | CFG1_EN_FAST_INIT * FAST_MODE
                       );
    
      tdc.attachInterruptFunc(tdc_ISR, FALLING);
    
      tdc.sendOpcode(OPCODE_INIT);
  }

}

bool state = 0;
unsigned long time_ms = millis();
unsigned long frameTime = 0;
unsigned long frameCount = 0;

int k = 0;
void loop()
{

  /* Respond to serial commands */
  if (Serial.available()) {
    byte r = Serial.read();
    respondToCommand((ECommand)r);
  }


  if (DEBUG && ardStatus == E_OK) {
    unsigned long curr_ms = millis();

    if (curr_ms - time_ms > 1000) {
        if(!FAST_MODE) {
          tdc.sendOpcode(OPCODE_INIT);
        }

      if (meas_index > 0) {
        Serial.print(F("Last result ["));
        Serial.print(meas_index - 1);
        Serial.println(F("]: "));
        Serial.print(measbuf[meas_index - 1]);
        Serial.print(F(" = "));
        Serial.print(measbuf[meas_index - 1] * cycleFactor_ns, 4);
        Serial.println(F(" ns"));
      }

      if (meas_index > 2) {
        Serial.println(measbuf[meas_index - 2]);
        Serial.println(measbuf[meas_index - 3]);
        Serial.println(measbuf[meas_index - 4]);
      }
      if(k%2==0) { tdc.updateCalibration(); }
      tdc.printStatus();
      tdc.printOutputRegisters();

      Serial.print("Temp = ");
      Serial.print(DS_update_temp_celcius());
      Serial.println(" C");
      time_ms = curr_ms;
    }
    k++;
  }

}

void tdc_ISR()
{
  if (meas_index >= MEAS_BUF_LEN) {
    tdc.sendOpcode(OPCODE_INIT);
  }
  else {
    // when calibrating during measurements ALU Pointer is set higher
    const uint8_t A = tdc.getALUPointer();
    const uint16_t res = tdc.readUncalibratedResult(A?A-1:0);


    measbuf[meas_index++] = res;
  }


}

void respondToCommand(ECommand cmd)
{
  switch (cmd) {
    case CMD_GET_ARDUINO_STATUS:
      {
        write_int8(ardStatus);
        break;
      }
    case CMD_GET_CYCLE_TIME_NS:
      {
        /* Send back cycle time (ns) as 4 bytes float
         * This should match should match 250 * corrFact * CLOCK_FACTOR / cal,
         * where corrFact = (resonator theoretical val)/(resonator measured val)
         */
        write_float(tdc.getCycleTime_ns());
        break;
      }
    case CMD_GET_UNCAL_RESULTS:
      {
        /* If meas_index==N (there are N results in measbuf), print 3+2*N bytes
         * Print N, the number of results following, (2 bytes), a status byte (0 OK), and 2*N result bytes
         * Then reset meas_index = 0
         */
        uint16_t N;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          N = meas_index;
        }

        write_uint16(N);

        write_uint8(N >= MEAS_BUF_LEN ? E_ARDUINO_OVERFLOW : E_OK);
        for (int i = 0; i < N; i++) {
          write_uint16(measbuf[i]);
        }
        
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          meas_index = 0;
        }
        
        tdc.sendOpcode(OPCODE_INIT);

        break;
      }
    case CMD_CALIBRATE:
      {
        /* Calibrate and sends back 12 bytes. Each little-endian:
         *    A: 1 byte int Tref_orig (ns) (constant, 250 for a 4MHz clock)
         *    B: 1 byte int CLOCK_FACTOR (constant based on initial DIV_CLKHS setting; 1,2 or 4)
         *    C: 4 bytes float theoretical resonator value, res_theor (constant based on initial settings)
         *    D: 4 bytes float measured resonator value, res_meas (varies, expect about 2000)
         *    E: 2 bytes int measured TDC cal cycles, cal (varies)
         */
        ardStatus = ardStatusFromCalibration();

        const tdc_calibration_t *cal = tdc.calibration();
        write_uint8(cal->Tref_theor_ns);
        write_uint8(cal->clock_factor);
        write_float(cal->resonator_theor_cycles);
        write_float(cal->resonator_meas_cycles);
        write_uint16(cal->tdc_cal_cycles);
        
        // get temp reading
        write_float(DS_update_temp_celcius());
        
        break;
      }


  }

}


/* Try to update calibration and return the Arduino status to send to PC
 * TODO: here the calibration status from GP22.h is cast as an Arduino status
 * for the serial protocol */
ECommStatus ardStatusFromCalibration()
{
    eCalibrationResult cal_res = tdc.updateCalibration();

    if (cal_res != E_CAL_OK) {
      if (DEBUG) {
        Serial.println(F("Calibration failed."));
      }
      return (ECommStatus)cal_res;
    }
    
    return E_OK;
}
