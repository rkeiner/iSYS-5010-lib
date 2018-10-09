
#ifndef SERIAL_STACK_H
#define SERIAL_STACK_H


#include "serial_radarAPI_if.h"
#include "serial_radarAPI_basicTypes.h"

// transmit frames

#define FRAME_START 0
#define FRAME_DESTINATION 1
#define FRAME_SOURCE 2
#define FRAME_FUNCTION 3

// fixed data length frames
#define FRAME_FIXED_PDU_START 4

// variable length frames
#define FRAME_VARIABLE_START 0
#define FRAME_VARIABLE_LENGTH 1
#define FRAME_VARIABLE_LENGTH_REP 2
#define FRAME_VARIABLE_START_2 = 3
#define FRAME_VARIABLE_DESTINATION 4
#define FRAME_VARIABLE_SOURCE 5
#define FRAME_VARIABLE_FUNCTION 6
#define FRAME_VARIABLE_PDU_START 7

// no data included
#define FRAME_NO_DATA_END_DELIMITER 5

//
// frame type
//
#define FRAME_TYPE_NO_DATA 0x10
#define FRAME_TYPE_VARIABLE_DATA 0x68
#define FRAME_TYPE_FIXED_DATA 0xA2
#define END_DELIMITER 0x16

//
// commmands
//
typedef struct device_cmd {
    uint8_t frametype;
    uint8_t destination;
    uint8_t source;
    uint8_t function;
    uint8_t pdu;
    uint8_t fcs;
    uint8_t end_delim;
} device_cmd_t;

//
// read device
//
#define CMD_READ_DEVICE 0xD0 // Read Device name Read device name as ASCII string 


//
// Acquisition commands
//
#define CMD_ACQISITION 0xD1

// Acquisition sub-funcs
#define SUB_FUNC_START_ACQUISITION  0x0000
#define SUB_FUNC_STOP_ACQUISITION 0x0001

//
// read/write sensor
//
#define CMD_READ_SENSOR 0xD2 
#define CMD_WRITE_SENSOR 0xD3

// sensor sub-funcs
#define SUB_FUNC_ADDRESS 0x0001
#define SUB_FUNC_FREQUENCY_CHANNEL 0x0004
#define SUB_FUNC_MINIMUM 0x000B
#define SUB_FUNC_MEASUREMENT_MODE 0x0010
#define SUB_FUNC_THRESHOLD_SENSITIVIY_LEFT 0x0016
#define SUB_FUNC_THRESHOLD_SENSITIVIY_RIGHT 0x0017
#define SUB_FUNC_IP_CONFIG 0x0029
#define SUB_FUNC_IP_DESTINATION 0x002A


// application settings
#define CMD_READ_APPLICATION 0xD4 //Read application settings Read application parameters, e.g. min/max range 
#define CMD_WRITE_APPLICATION 0xD5 // Write application settings Write application parameters, e.g. min/max range 

/*
// application sub-funcs
0x00 00 // Line filter 1 Line filter 1 setting 1. Uint16_t: Enable (0= off, 1 = on) 2. Uint16_t: frequency in Hz 3. Uint16_t: offset in tenth of dB 6 (3 x uint16_t) 
0x00 01 // Line filter 2 Line filter 2 setting 1. Uint16_t: Enable (0= off, 1 = on) 2. Uint16_t: frequency in Hz 3. Uint16_t: offset in tenth of dB 6 (3 x uint16_t) 
0x0X 00 // Enable Digital output enable  0 = off  1 = digital  2 = pwm 2 (uint16_t) 
0x0X 01 // Rising delay Rising delay in multiples of cycle time 2 (uint16_t) 
0x0X 02 // falling delay falling delay in multiples of cycle time 2 (uint16_t) 
0x0X 04 // Setting1 Setting 1 for the digital outputs  0 = active low side  1 = active high side  2 = totem pole 2 (uint16_t) 
0x0X 05 // Setting2 Setting 2 for the digital outputs  0 = normally open 2 (uint16_t)
0x0X 06 // Angle min Min possible angle (value in tenth of degree) 2 (sint16_t) 
0x0X 07 // Angel max max possible angle (value in tenth of degree) 2 (sint16_t) 
0x0X 08 // Range min Min possible range (value in tenth of m) 2 (sint16_t) 
0x0X 09 // Range max Max possible range (value in tenth of m) 2 (sint16_t) 
0x0X 0A // Signal min Min possible signal strength (value in tenth of dB) 2 (sint16_t) 
0x0X 0B // Signal max Max possible signal strength (value in tenth of dB) 2 (sint16_t) 
0x0X 0C // Velocity min Min possible velocity (value in tenth of 𝑚𝑠) 2 (sint16_t) 
0x0X 0D // Velocity max Max possible velocity (value in tenth of 𝑚𝑠) 2 (sint16_t) 
0x0X 0E // Direction Moving direction of detect  1=approaching  2=receding  3=both directions Note: only available for sensors with velocity measurement function 2 (sint16_t) 
0x0X 15 // Output filter Type of single target filter  0=highest amplitude  1=mean  2=median  3=min  4=max 2 (uint16_t) 
0x0X 16 // Output filter signal Signal for single target filter  0=off  1=velocity radial  2=range radial 2 (uint16_t) 
0x0X 17 // Alpha filter velocity Alpha-filter parameter for velocity set between 0 and 100 in percent 2 (uint16_t) 
0x0X 18 // Alpha filter range Alpha-filter parameter for range Set between 1 and 100 in percent 2 (uint16_t) 
0x0X 19 // Range min extended Min possible range (value in centimeter (0.01m)) 2 (sint16_t) 
0x0X 1A // Range max extended Max possible range (value in centimeter (0.01m)) 2 (sint16_t)
0x05 06 // Mounting offset (iSYS-6203 only) Set an additional mounting offset in mm 4 (uint32_t) 
0x06 80 // Potis Reads/Writes the amplification potentiometer values 2 (uint16_t) 
0x07 07 // Temperature threshold (iSYS-6203 only) Sets the temperature threshold for the temperature warning/alarm outputs OUT1 and OUT2 7 (3 x uint16_t + 1) 
0x07 09 // Range/ temperature warning switch (iSYS-6203 only) Switch output between temperature and range warning 3 (3 x uint8_t) 
0x07 0A // Temperature threshold value iSYS-6203 only) Set/get temperature warning threshold 4 (2x uint8_t + sint16_t) 
0x07 0B // range threshold value iSYS-6203 only) Set/get range warning threshold 6 (2x uint8_t + uint32_t) 
0x08 28 // Threshold margin parameter for moving targets: near range Set/get the sensitivity margin for moving targets in near range (up to 2.7m) 3 (uint8_t + sint16_t) 
0x08 29 // Threshold margin parameter for moving targets: main range Set/get the sensitivity margin for moving targets in the main range (2.7m to 40.7 m) 3 (uint8_t + sint16_t) 
0x08 2A // Threshold margin parameter for moving targets: long range Set/get the sensitivity margin for moving targets in near range (above 40.7m) 3 (uint8_t + sint16_t) 
0x08 53 // Detection clustering Enables / disables the detection clustering 2 (2 x uint8_t) 
0x08 80 // Rcs output enable Enables / disables the rcs output 2 (2 x uint8_t)

*/


//
// read calibration 
//
#define CMD_READ_CALIBRATION_SETTINGS 0xD6 // Read calibration settings Read calibration settings, e.g. firmware version 


#define SUB_FUNC_FIRMWARE_VERSION 0x0101 // Firmware version Returns the firmware version running on the connected iSYS device. 6 (3 x uint16_t) 
#define SUB_FUNC_DSP_HARDWARE_VERSION 0x0102 // DSP-Hardware version Returns the dsp hardware version of the connected iSYS 6 (3 x uint16_t) 
#define SUB_FUNC_RFE_HARDWARE_VERSION 0x0103 // RFE-Hardware version Returns the rfe hardware version of the connected iSYS 6 (3 x uint16_t) 
#define SUB_FUNC_PRODUCT_INFO 0x0104 // Product info Returns the product info (product code) of the connected iSYS 2 (1 x uint16_t) 
#define SUB_FUNC_BOOTLOADER_VERSION 0x0220 // Bootloader version Returns the bootloader version of the connected iSYS device Command only supported by devices with newer firmware:  iSYS-6xxx: firmware >=0.400  iSYS-4xxx: firmware >=2.000  iSYS-5xxx: 6 (3 x uint16_t)

#define CMD_READ_TARGET_LIST 0xDA // Read target list Read target-list, e.g. 16-bit target list, 32-bit target list 

// in PDU byte[0]
#define CMD_TARGET_LIST_FILTER_1 1
#define CMD_TARGET_LIST_FILTER_2 2
#define CMD_TARGET_LIST_FILTER_3 3

// PDU byte[1]
#define CMD_TARGET_LIST_OUTPUT_16_BIT 16
#define CMD_TARGET_LIST_OUTPUT_32_BIT 32
//
// ????  0b1xxxxxxx Flag for requesting target list with fixed frame length if supported by sensor

#define CMD_READ_DIGITAL_OUTPUT_STATE 0xDB // Read digital output state Reads the output state of the digital outputs 
#define CMD_EEPROM 0xDF // EEPEOM EEPROM functions, e.g. set factory default, save sensor/application settings 
#define CMD_READ_RAW 0xE0 // Read raw signal iSYS sends the sampled raw data of one measurement / modulation cycle 
#define CMD_READ_RANGE_LIST 0xE1 // Read range list Reads range list consisting of different range values calculated by the modulation scheme and additional statistical values (only iSYS-600x, iSYS-6203) 
#define CMD_FAILURE 0xFD // Failure Failure

#endif // SERIAL_STACK_H