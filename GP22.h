#ifndef GP22_H
#define GP22_H

/* Functions and constants for talking to the Acam TDC-GP22
 * 
 * Definitions are based on datasheet DB_GP22_en V0.9 (March 13, 2014) with corrections
 */

#include "Arduino.h"
#include "arduino_io.h"
#include <util/atomic.h>


typedef uint32_t reg_t;

#define BITMASK(n) ((uint32_t)1 << n)


enum eParamsRegister0 {
    CFG0_ID0_0                    = BITMASK(0),
    CFG0_ID0_1                    = BITMASK(1),
    CFG0_ID0_2                    = BITMASK(2),
    CFG0_ID0_3                    = BITMASK(3),
    CFG0_ID0_4                    = BITMASK(4),
    CFG0_ID0_5                    = BITMASK(5),
    CFG0_ID0_6                    = BITMASK(6),
    CFG0_ID0_7                    = BITMASK(7),
    CFG0_NEG_START                = BITMASK(8),
    CFG0_NEG_STOP1                = BITMASK(9),
    CFG0_NEG_STOP2                = BITMASK(10),
    CFG0_MESSB2                   = BITMASK(11),
    CFG0_NO_CAL_AUTO              = BITMASK(12),
    CFG0_CALIBRATE                = BITMASK(13),
    CFG0_SEL_ECLK_TMP             = BITMASK(14),
    CFG0_ANZ_FAKE                 = BITMASK(15),
    CFG0_TCYCLE                   = BITMASK(16),
    CFG0_ANZ_PORT                 = BITMASK(17),
    CFG0_START_CLKHS_START_0      = BITMASK(18),
    CFG0_START_CLKHS_START_1      = BITMASK(19),
    CFG0_DIV_CLKHS_0              = BITMASK(20),
    CFG0_DIV_CLKHS_1              = BITMASK(21),
    CFG0_ANZ_PER_CALRES_0         = BITMASK(22),
    CFG0_ANZ_PER_CALRES_1         = BITMASK(23),
    CFG0_DIV_FIRE_0               = BITMASK(24),
    CFG0_DIV_FIRE_1               = BITMASK(25),
    CFG0_DIV_FIRE_2               = BITMASK(26),
    CFG0_DIV_FIRE_3               = BITMASK(27),
    CFG0_ANZ_FIRE_START_0         = BITMASK(28),
    CFG0_ANZ_FIRE_START_1         = BITMASK(29),
    CFG0_ANZ_FIRE_START_2         = BITMASK(30),
    CFG0_ANZ_FIRE_START_3         = BITMASK(31)
};

enum eParamsRegister1 {
    CFG1_ID1_0                    = BITMASK(0),
    CFG1_ID1_1                    = BITMASK(1),
    CFG1_ID1_2                    = BITMASK(2),
    CFG1_ID1_3                    = BITMASK(3),
    CFG1_ID1_4                    = BITMASK(4),
    CFG1_ID1_5                    = BITMASK(5),
    CFG1_ID1_6                    = BITMASK(6),
    CFG1_ID1_7                    = BITMASK(7),
    CFG1_SEL_TSTO1_0              = BITMASK(8),
    CFG1_SEL_TSTO1_1              = BITMASK(9),
    CFG1_SEL_TSTO1_2              = BITMASK(10),
    CFG1_SEL_TSTO2_0              = BITMASK(11),
    CFG1_SEL_TSTO2_1              = BITMASK(12),
    CFG1_SEL_TSTO2_2              = BITMASK(13),
    CFG1_SEL_START_FIRE           = BITMASK(14),
    CFG1_CURR32K                  = BITMASK(15),
    CFG1_HITIN1_0                 = BITMASK(16),
    CFG1_HITIN1_1                 = BITMASK(17),
    CFG1_HITIN1_2                 = BITMASK(18),
    CFG1_HITIN2_0                 = BITMASK(19),
    CFG1_HITIN2_1                 = BITMASK(20),
    CFG1_HITIN2_2                 = BITMASK(21),
    CFG1_KEEP_DEFAULT             = BITMASK(22),
    CFG1_EN_FAST_INIT             = BITMASK(23),
    CFG1_HIT1_0                   = BITMASK(24),
    CFG1_HIT1_1                   = BITMASK(25),
    CFG1_HIT1_2                   = BITMASK(26),
    CFG1_HIT1_3                   = BITMASK(27),
    CFG1_HIT2_0                   = BITMASK(28),
    CFG1_HIT2_1                   = BITMASK(29),
    CFG1_HIT2_2                   = BITMASK(30),
    CFG1_HIT2_3                   = BITMASK(31)
};

enum eParamsRegister2 {
    CFG2_ID2_0                    = BITMASK(0),
    CFG2_ID2_1                    = BITMASK(1),
    CFG2_ID2_2                    = BITMASK(2),
    CFG2_ID2_3                    = BITMASK(3),
    CFG2_ID2_4                    = BITMASK(4),
    CFG2_ID2_5                    = BITMASK(5),
    CFG2_ID2_6                    = BITMASK(6),
    CFG2_ID2_7                    = BITMASK(7),
    CFG2_DELVAL1_0                = BITMASK(8),
    CFG2_DELVAL1_1                = BITMASK(9),
    CFG2_DELVAL1_2                = BITMASK(10),
    CFG2_DELVAL1_3                = BITMASK(11),
    CFG2_DELVAL1_4                = BITMASK(12),
    CFG2_DELVAL1_5                = BITMASK(13),
    CFG2_DELVAL1_6                = BITMASK(14),
    CFG2_DELVAL1_7                = BITMASK(15),
    CFG2_DELVAL1_8                = BITMASK(16),
    CFG2_DELVAL1_9                = BITMASK(17),
    CFG2_DELVAL1_10               = BITMASK(18),
    CFG2_DELVAL1_11               = BITMASK(19),
    CFG2_DELVAL1_12               = BITMASK(20),
    CFG2_DELVAL1_13               = BITMASK(21),
    CFG2_DELVAL1_14               = BITMASK(22),
    CFG2_DELVAL1_15               = BITMASK(23),
    CFG2_DELVAL1_16               = BITMASK(24),
    CFG2_DELVAL1_17               = BITMASK(25),
    CFG2_DELVAL1_18               = BITMASK(26),
    CFG2_RFEDGE1                  = BITMASK(27),
    CFG2_RFEDGE2                  = BITMASK(28),
    CFG2_EN_INT_ALU           = BITMASK(29),
    CFG2_EN_INT_HITS           = BITMASK(30),
    CFG2_EN_INT_TDC_TIMEOUT           = BITMASK(31)
};

enum eParamsRegister3 {
    CFG3_ID3_0                    = BITMASK(0),
    CFG3_ID3_1                    = BITMASK(1),
    CFG3_ID3_2                    = BITMASK(2),
    CFG3_ID3_3                    = BITMASK(3),
    CFG3_ID3_4                    = BITMASK(4),
    CFG3_ID3_5                    = BITMASK(5),
    CFG3_ID3_6                    = BITMASK(6),
    CFG3_ID3_7                    = BITMASK(7),
    CFG3_DELVAL2_0                = BITMASK(8),
    CFG3_DELVAL2_1                = BITMASK(9),
    CFG3_DELVAL2_2                = BITMASK(10),
    CFG3_DELVAL2_3                = BITMASK(11),
    CFG3_DELVAL2_4                = BITMASK(12),
    CFG3_DELVAL2_5                = BITMASK(13),
    CFG3_DELVAL2_6                = BITMASK(14),
    CFG3_DELVAL2_7                = BITMASK(15),
    CFG3_DELVAL2_8                = BITMASK(16),
    CFG3_DELVAL2_9                = BITMASK(17),
    CFG3_DELVAL2_10               = BITMASK(18),
    CFG3_DELVAL2_11               = BITMASK(19),
    CFG3_DELVAL2_12               = BITMASK(20),
    CFG3_DELVAL2_13               = BITMASK(21),
    CFG3_DELVAL2_14               = BITMASK(22),
    CFG3_DELVAL2_15               = BITMASK(23),
    CFG3_DELVAL2_16               = BITMASK(24),
    CFG3_DELVAL2_17               = BITMASK(25),
    CFG3_DELVAL2_18               = BITMASK(26),
    CFG3_SEL_TIMO_MB2_0           = BITMASK(27),
    CFG3_SEL_TIMO_MB2_1           = BITMASK(28),
    CFG3_EN_ERR_VAL               = BITMASK(29),
    CFG3_EN_FIRST_WAVE            = BITMASK(30),
    CFG3_EN_AUTOCALC_MB2          = BITMASK(31)
};

enum eParamsRegister4 {
    CFG4_ID4_0                    = BITMASK(0),
    CFG4_ID4_1                    = BITMASK(1),
    CFG4_ID4_2                    = BITMASK(2),
    CFG4_ID4_3                    = BITMASK(3),
    CFG4_ID4_4                    = BITMASK(4),
    CFG4_ID4_5                    = BITMASK(5),
    CFG4_ID4_6                    = BITMASK(6),
    CFG4_ID4_7                    = BITMASK(7),
    CFG4_DELVAL3_0                = BITMASK(8),
    CFG4_DELVAL3_1                = BITMASK(9),
    CFG4_DELVAL3_2                = BITMASK(10),
    CFG4_DELVAL3_3                = BITMASK(11),
    CFG4_DELVAL3_4                = BITMASK(12),
    CFG4_DELVAL3_5                = BITMASK(13),
    CFG4_DELVAL3_6                = BITMASK(14),
    CFG4_DELVAL3_7                = BITMASK(15),
    CFG4_DELVAL3_8                = BITMASK(16),
    CFG4_DELVAL3_9                = BITMASK(17),
    CFG4_DELVAL3_10               = BITMASK(18),
    CFG4_DELVAL3_11               = BITMASK(19),
    CFG4_DELVAL3_12               = BITMASK(20),
    CFG4_DELVAL3_13               = BITMASK(21),
    CFG4_DELVAL3_14               = BITMASK(22),
    CFG4_DELVAL3_15               = BITMASK(23),
    CFG4_DELVAL3_16               = BITMASK(24),
    CFG4_DELVAL3_17               = BITMASK(25),
    CFG4_DELVAL3_18               = BITMASK(26),
    CFG4_KEEP_DEFAULT_0           = BITMASK(27),
    CFG4_KEEP_DEFAULT_1           = BITMASK(28),
    CFG4_KEEP_DEFAULT_2           = BITMASK(29),
    CFG4_KEEP_DEFAULT_3           = BITMASK(30),
    CFG4_KEEP_DEFAULT_4           = BITMASK(31)
};


enum eParamsRegister3FirstWave {
    CFG3FW_ID3_0                    = BITMASK(0),
    CFG3FW_ID3_1                    = BITMASK(1),
    CFG3FW_ID3_2                    = BITMASK(2),
    CFG3FW_ID3_3                    = BITMASK(3),
    CFG3FW_ID3_4                    = BITMASK(4),
    CFG3FW_ID3_5                    = BITMASK(5),
    CFG3FW_ID3_6                    = BITMASK(6),
    CFG3FW_ID3_7                    = BITMASK(7),
    CFG3FW_DELREL1_0                = BITMASK(8),
    CFG3FW_DELREL1_1                = BITMASK(9),
    CFG3FW_DELREL1_2                = BITMASK(10),
    CFG3FW_DELREL1_3                = BITMASK(11),
    CFG3FW_DELREL1_4                = BITMASK(12),
    CFG3FW_DELREL1_5                = BITMASK(13),
    CFG3FW_DELREL2_0                = BITMASK(14),
    CFG3FW_DELREL2_1                = BITMASK(15),
    CFG3FW_DELREL2_2                = BITMASK(16),
    CFG3FW_DELREL2_3                = BITMASK(17),
    CFG3FW_DELREL2_4                = BITMASK(18),
    CFG3FW_DELREL2_5                = BITMASK(19),
    CFG3FW_DELREL3_0                = BITMASK(20),
    CFG3FW_DELREL3_1                = BITMASK(21),
    CFG3FW_DELREL3_2                = BITMASK(22),
    CFG3FW_DELREL3_3                = BITMASK(23),
    CFG3FW_DELREL3_4                = BITMASK(24),
    CFG3FW_DELREL3_5                = BITMASK(25),
    CFG3FW_KEEP_DEFAULT             = BITMASK(26),
    CFG3FW_SEL_TIMO_MB2_0           = BITMASK(27),
    CFG3FW_SEL_TIMO_MB2_1           = BITMASK(28),
    CFG3FW_EN_ERR_VAL               = BITMASK(29),
    CFG3FW_EN_FIRST_WAVE            = BITMASK(30),
    CFG3FW_EN_AUTOCALC_MB2          = BITMASK(31)
};

enum eParamsRegister4FirstWave {
    CFG4FW_ID4_0                    = BITMASK(0),
    CFG4FW_ID4_1                    = BITMASK(1),
    CFG4FW_ID4_2                    = BITMASK(2),
    CFG4FW_ID4_3                    = BITMASK(3),
    CFG4FW_ID4_4                    = BITMASK(4),
    CFG4FW_ID4_5                    = BITMASK(5),
    CFG4FW_ID4_6                    = BITMASK(6),
    CFG4FW_ID4_7                    = BITMASK(7),
    CFG4FW_OFFS_0                   = BITMASK(8),
    CFG4FW_OFFS_1                   = BITMASK(9),
    CFG4FW_OFFS_2                   = BITMASK(10),
    CFG4FW_OFFS_3                   = BITMASK(11),
    CFG4FW_OFFS_4                   = BITMASK(12),
    CFG4FW_OFFSRNG1                 = BITMASK(13),
    CFG4FW_OFFSRNG2                 = BITMASK(14),
    CFG4FW_EDGE_FW                  = BITMASK(15),
    CFG4FW_DIS_PW                   = BITMASK(16),
    CFG4FW_KEEP_DEFAULT_0           = BITMASK(17),
    CFG4FW_KEEP_DEFAULT_1           = BITMASK(18),
    CFG4FW_KEEP_DEFAULT_2           = BITMASK(19),
    CFG4FW_KEEP_DEFAULT_3           = BITMASK(20),
    CFG4FW_KEEP_DEFAULT_4           = BITMASK(21),
    CFG4FW_KEEP_DEFAULT_5           = BITMASK(22),
    CFG4FW_KEEP_DEFAULT_6           = BITMASK(23),
    CFG4FW_KEEP_DEFAULT_7           = BITMASK(24),
    CFG4FW_KEEP_DEFAULT_8           = BITMASK(25),
    CFG4FW_KEEP_DEFAULT_9           = BITMASK(26),
    CFG4FW_KEEP_DEFAULT_10          = BITMASK(27),
    CFG4FW_KEEP_DEFAULT_11          = BITMASK(28),
    CFG4FW_KEEP_DEFAULT_12          = BITMASK(29),
    CFG4FW_KEEP_DEFAULT_13          = BITMASK(30),
    CFG4FW_KEEP_DEFAULT_14          = BITMASK(31)
};

enum eParamsRegister5 {
    CFG5_ID5_0                    = BITMASK(0),
    CFG5_ID5_1                    = BITMASK(1),
    CFG5_ID5_2                    = BITMASK(2),
    CFG5_ID5_3                    = BITMASK(3),
    CFG5_ID5_4                    = BITMASK(4),
    CFG5_ID5_5                    = BITMASK(5),
    CFG5_ID5_6                    = BITMASK(6),
    CFG5_ID5_7                    = BITMASK(7),
    CFG5_PHFIRE_0                 = BITMASK(8),
    CFG5_PHFIRE_1                 = BITMASK(9),
    CFG5_PHFIRE_2                 = BITMASK(10),
    CFG5_PHFIRE_3                 = BITMASK(11),
    CFG5_PHFIRE_4                 = BITMASK(12),
    CFG5_PHFIRE_5                 = BITMASK(13),
    CFG5_PHFIRE_6                 = BITMASK(14),
    CFG5_PHFIRE_7                 = BITMASK(15),
    CFG5_PHFIRE_8                 = BITMASK(16),
    CFG5_PHFIRE_9                 = BITMASK(17),
    CFG5_PHFIRE_10                = BITMASK(18),
    CFG5_PHFIRE_11                = BITMASK(19),
    CFG5_PHFIRE_12                = BITMASK(20),
    CFG5_PHFIRE_13                = BITMASK(21),
    CFG5_PHFIRE_14                = BITMASK(22),
    CFG5_PHFIRE_15                = BITMASK(23),
    CFG5_REPEAT_FIRE_0            = BITMASK(24),
    CFG5_REPEAT_FIRE_1            = BITMASK(25),
    CFG5_REPEAT_FIRE_2            = BITMASK(26),
    CFG5_DIS_PHASESHIFT           = BITMASK(27),
    CFG5_EN_STARTNOISE            = BITMASK(28),
    CFG5_CON_FIRE_0               = BITMASK(29),
    CFG5_CON_FIRE_1               = BITMASK(30),
    CFG5_CON_FIRE_2               = BITMASK(31)
};

enum eParamsRegister6 {
    CFG6_ID6_0                    = BITMASK(0),
    CFG6_ID6_1                    = BITMASK(1),
    CFG6_ID6_2                    = BITMASK(2),
    CFG6_ID6_3                    = BITMASK(3),
    CFG6_ID6_4                    = BITMASK(4),
    CFG6_ID6_5                    = BITMASK(5),
    CFG6_ID6_6                    = BITMASK(6),
    CFG6_ID6_7                    = BITMASK(7),
    CFG6_ANZ_FIRE_END_0           = BITMASK(8),
    CFG6_ANZ_FIRE_END_1           = BITMASK(9),
    CFG6_ANZ_FIRE_END_2           = BITMASK(10),
    CFG6_TEMP_PORTDIR             = BITMASK(11),
    CFG6_DOUBLE_RES               = BITMASK(12),
    CFG6_QUAD_RES                 = BITMASK(13),
    CFG6_FIREO_DEF                = BITMASK(14),
    CFG6_HZ60                     = BITMASK(15),
    CFG6_CYCLE_TOF_0              = BITMASK(16),
    CFG6_CYCLE_TOF_1              = BITMASK(17),
    CFG6_CYCLE_TEMP_0             = BITMASK(18),
    CFG6_CYCLE_TEMP_1             = BITMASK(19),
    CFG6_START_CLKHS_END          = BITMASK(20),
    CFG6_EN_INT_END               = BITMASK(21),
    CFG6_TW2_0                    = BITMASK(22),
    CFG6_TW2_1                    = BITMASK(23),
    CFG6_EMPTY_0                  = BITMASK(24),
    CFG6_DA_KORR_0                = BITMASK(25),
    CFG6_DA_KORR_1                = BITMASK(26),
    CFG6_DA_KORR_2                = BITMASK(27),
    CFG6_DA_KORR_3                = BITMASK(28),
    CFG6_EMPTY_1                  = BITMASK(29),
    CFG6_NEG_STOP_TEMP            = BITMASK(30),
    CFG6_EN_ANALOG                = BITMASK(31)
};

const int CFG_REGISTER_MAX = 7;

/* Default register values on reset */
const reg_t CFG_DEFAULT_BITMASKS[7] = {0x22066800,
                                    0x55400000,
                                    0x20000000,
                                    0x18000000,
                                    0x20000000,
                                    0x00000000,
                                    0x00000000};

/* Blank register values except for the "keep default" values */
const reg_t CFG_KEEP_DEFAULT_BITMASKS[7] = {0x00000000,
                                    0x00400000,
                                    0x00000000,
                                    0x00000000,
                                    0x20000000,
                                    0x00000000,
                                    0x00000000};

/* Read registers */
enum eReadRegister {
    READ_RES_0 = 0,
    READ_RES_1,
    READ_RES_2,
    READ_RES_3,
    READ_STAT,
    READ_REG_1,
    READ_PW1ST,
    READ_MAX
};

/* Status register bits */
enum eBitsStatusRegister {
    STAT_ALU_OP_PTR               = (1 << 0),  // ALU operation pointer (see datasheet) (3 bits) 
    STAT_HITS_CH1                 = (1 << 3),  // Number of hits registered on channel 1 (3 bits)
    STAT_HITS_CH2                 = (1 << 6),  // Number of hits registered on channel 2 (3 bits)
    STAT_TIMEOUT_TDC              = (1 << 9),  // Indicates an overflow of the TDC unit
    STATUS_TIMEOUT_PRECOUNTER     = (1 << 10), // Indicates an overflow of the 14 bit precounter in MR 2
    STAT_ERROR_OPEN               = (1 << 11), // Indicates an open sensor at temperature measurement
    STAT_ERROR_SHORT              = (1 << 12), // Indicates a shorted sensor at temperature measurement
    STAT_EEPROM_ERROR             = (1 << 13), // Single error in EEPROM which has been corrected
    STAT_EEPROM_DED               = (1 << 14), // Double error detection. A multiple error has been detected which can not be corrected
    STAT_EEPROM_EQ_CREG           = (1 << 15), // Indicates whether the content of the configuration registers equals the EEPROM
};  

/* Address of each read register */
const byte READ_REGISTER_ADDRS[READ_MAX] = {0,1,2,3,4,5,8};

/* Number of bits in each read register */
const byte READ_REGISTER_LENGTHS[READ_MAX] = {32, 32, 32, 32, 16, 8, 8};

/* Opcodes */
enum eOpcode {
    OPCODE_WRITE_ADDRESS        = 0x80, //3 LSB are the address to write
    OPCODE_READ_ADDRESS         = 0xB0, //3 LSB are the address to read
    OPCODE_READ_ID              = 0xB7,
    OPCODE_READ_PW1ST           = 0xB8,
    OPCODE_EEPROM_SAVE          = 0xC0, //Write configuration registers into EEPROM
    OPCODE_EEPROM_RESTORE       = 0xF0, //Transfer EEPROM content into configuration registers
    OPCODE_EEPROM_COMPARE       = 0xC6, //Compare configuration registers with EEPROM
    OPCODE_INIT                 = 0x70,
    OPCODE_POWER_ON_RESET       = 0x50,
    OPCODE_START_TOF            = 0x01,
    OPCODE_START_TEMP           = 0x02,
    OPCODE_START_CAL_RESONATOR  = 0x03,
    OPCODE_START_CAL_TDC        = 0x04,
    OPCODE_START_TOF_RESTART    = 0x05,
    OPCODE_START_TEMP_RESTART   = 0x06
};

const int BYTE_DELAY_MICROSECONDS = 1;

/* Struct for keeping calibration info 
 *  The time of one cycle in nanoseconds can be calculated using:
 *  [ns/cycle] = Tref_theor_ns * clock_factor * resonator_theor_cycles / resonator_meas_cycles / tdc_cal_cycles
 *             = A * B * C / D / E
 */
typedef struct tdc_calibration_t {
    uint8_t Tref_theor_ns; // A: Constant in ns based on clock speed. 250 for a 4MHz clock
    uint8_t clock_factor; // B: Constant after initial DIV_CLKHS setting. Can be 1, 2 or 4
    float resonator_theor_cycles; // C: Theoretical cycles during CAL_RESONATOR procedure (constant after initial settings)
    float resonator_meas_cycles; //  D: Measured cycles during CAL_RESONATOR procedure, should be close to theoretical
    uint16_t tdc_cal_cycles; // E: Measured TDC cal cycles 
};

/* Status enum for calibration function error codes.
 * Each corresponds to a point of failure during calibration.
 * These are negative for use in return value.
 */
enum eCalibrationResult {
 E_CAL_OK = 0,
 E_CAL_FAIL_RESONATOR = -1,
 E_CAL_FAIL_NO_HITS = -2,
 E_CAL_FAIL_NO_MEASUREMENT = -3,
 E_CAL_FAIL_TDC_TIMEOUT = -4,
 E_CAL_FAIL_WAIT = -5,
 E_CAL_FAIL_GARBAGE = -6
};

/* Class for interfacing with TDC-GP22 */
class GP22 {

public:
    GP22(int pinInt, bool debug, Print& outStream=Serial);
    ~GP22();
    
    // I/O methods
    void init();
    void attachInterruptFunc(void (*func)(void), int mode);
    void attachPreviousInterruptFunc();
    bool testCommunication();
    void printIDs();
    void printStatus();
    void printHexNum(uint8_t *data, size_t len);
    void printConfigRegisters();
    void printOutputRegisters();
    void writeRegister(byte address, reg_t data);
    uint32_t readRegister(eReadRegister reg);
    float readResult(int address);
    int16_t readUncalibratedResult(int address);
    void sendOpcode(eOpcode opcode);
    long waitForInterrupt(long timeout_us=0);
    int getALUPointer();
    int8_t getClockFactor();
    float getResonatorCycles();
    float getResonatorCyclesTheoretical();
    int16_t getCalCycles();
    eCalibrationResult updateCalibration();
    
    // Getters (results may not be valid until some I/O is done)
    const tdc_calibration_t* calibration() const { return &_calibration; }
    float getResonatorCorrectionFactor() const;
    float getCycleTime_ns() const;
        
private:
    void writeNBytes(byte opcode, uint32_t data, uint8_t n);
    uint32_t readNBytes(byte opcode, uint8_t n);
    void tempSaveRegisters();
    void tempRestoreRegisters();
    static void pinChangeISR();
    static void pinChangeISR_debug();
    void attachDefaultInterruptFunc();
    static inline float fixedPoint16ToFloat(uint32_t num) { return num / 65536.0; }
    static inline int fixedPoint16ToInt(uint32_t num) { return num / 65536; }
    inline bool checkInterrupt();

private:
    reg_t _configRegisters[CFG_REGISTER_MAX];
    reg_t _configRegistersTemp[CFG_REGISTER_MAX];
    int _pinInt;
    bool _bHardwareInterrupt;
    void (*volatile _ISRfunc)(void);
    volatile int _ISRmode;
    const bool _bDebug;
    Print& _serial;
    tdc_calibration_t _calibration;
    
    static const uint32_t HS_CLK_FREQ_HZ = 4000000; // External 4 Mhz resonator
    static const uint32_t REF_CLK_FREQ_HZ = 32768; // Internal 32 kHz clock
    static const uint32_t HS_CLK_PERIOD_NS = 1000000000 / HS_CLK_FREQ_HZ; // Period of 4 MHz resonator, aka Tref
    
    // Set ANZ_PER_CALRES, which defines the number of periods of the 32 kHz clock used for resonator calibration
    static const int DEFAULT_ANZ_PER_CALRES = 3; // use 16 periods, 488.281 us
    

};

extern volatile bool _intFlag; // whether interrupt pin is LOW

#endif //GP22_H
