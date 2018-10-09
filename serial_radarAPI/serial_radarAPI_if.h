/**************************************************************************************

      II    N     N     N     N      OOOO      SSSSS     EEEEE    N     N    TTTTTTT
     II    NNN   N     NNN   N    OO    OO    S         E        NNN   N       T
    II    N NN  N     N NN  N    OO    OO    SSSSS     EEE      N NN  N       T
   II    N  NN N     N  NN N    OO    OO        S     E        N  NN N       T
  II    N    NN     N    NN      OOOO      SSSSS     EEEEE    N    NN       T
                         copyright (c) 2016, InnoSenT GmbH
                                 all rights reserved

***************************************************************************************

    filename:			serial_radarAPI_if.h
    brief:				API for communication between iSYS-4XXX/iSYS-6XXX and PC over serialport
    creation:			28.01.2016
    author:				Sebastian Weidmann

version:			v1.0
    last edit:          28.01.2016
    last editor:        Sebastian Weidmann
    change:             -first release
    compile switches:

version:			v2.0
    last edit:          18.02.2016
    last editor:        Sebastian Weidmann
    change:             -bug fixed: cmdSetCalibrationDataRangeOffset
                        -fixed typing error (Linefilter)
                        -changed some enums and set all iSYS enum to ISYS enum
    compile switches:

version:            v3.0
    last edit:
    last editor:
    change:             -fixed bug in iSYS_getDigitalOutputState function
    compile switches:

version:            v4.0
    last edit:          15.04.2016
    last editor:        Benjamin Grünewald
    change:             -added commands from IDS-1000
                        -changed "sensivity" to "sensitivity"
						-bug fixed: factor for angle changed
						-added support for iSYS-6203
                        -added new command Get/SetMeasurementMode and ENUM iSYSMeasurementMode
	compile switches:

version:            v4.1
    last edit:          18.05.2016
    last editor:        Thomas Popp
    change:             -bug fixed set/get calibration values
    compile switches:

version:            v5.1
    last edit:          25.11.2016
    last editor:        Benjamin Gruenewald
    change:             -added support for iSYS-5010

***************************************************************************************/

#ifndef ISYS_API_IF_H
#define ISYS_API_IF_H

#include "serial_radarAPI_basicTypes.h"

/**************************************************************************************
 defines
**************************************************************************************/
#define ISYS_API_LIBRARY 1
#ifdef _WIN32
    #ifdef ISYS_API_LIBRARY
        #define ISYS_API_EXPORT __declspec(dllexport)
    #else
        #define ISYS_API_EXPORT __declspec(dllimport)
    #endif
#else
    #define ISYS_API_EXPORT
#endif
 
#define MAX_TARGETS     (0x23)
#define MAX_IQ_PAIRS    (0x04)
#define MAX_DATA_SIZE   (2048)
#define MAX_ERROR_COUNT (100)

#ifdef __cplusplus
extern "C" {
#endif



/**************************************************************************************
 typedefs
**************************************************************************************/
typedef enum iSYSBaudrate
{
    ISYS_BAUDRATE_110    = 0,
    ISYS_BAUDRATE_300       ,
    ISYS_BAUDRATE_600       ,
    ISYS_BAUDRATE_1200      ,
    ISYS_BAUDRATE_2400      ,
    ISYS_BAUDRATE_4800      ,
    ISYS_BAUDRATE_9600      ,
    ISYS_BAUDRATE_19200     ,
    ISYS_BAUDRATE_38400     ,
    ISYS_BAUDRATE_57600     ,
    ISYS_BAUDRATE_115200    ,
    ISYS_BAUDRATE_256000    ,
    ISYS_BAUDRATE_512000
}iSYSBaudrate_t;

typedef enum iSYSOutput
{
    ISYS_OUTPUT_OFF     = 0,
    ISYS_OUTPUT_DIGITAL    ,
    ISYS_OUTPUT_PWM
} iSYSOutput_t;

typedef enum iSYSOutputNumber
{
    ISYS_OUTPUT_1     = 1,
    ISYS_OUTPUT_2        ,
    ISYS_OUTPUT_3
} iSYSOutputNumber_t;

typedef enum iSYSDirection_type
{
    ISYS_TARGET_DIRECTION_APPROACHING = 1,
    ISYS_TARGET_DIRECTION_RECEDING = 2,
    ISYS_TARGET_DIRECTION_BOTH = ISYS_TARGET_DIRECTION_APPROACHING | ISYS_TARGET_DIRECTION_RECEDING,
    ISYS_TARGET_DIRECTION_END
} iSYSDirection_type_t;

typedef enum iSYSVelocity_unit
{
    ISYS_VELOCITY_UNIT_MS       = 0,
    ISYS_VELOCITY_UNIT_KMH         ,
    ISYS_VELOCITY_UNIT_MPH
} iSYSVelocity_unit_t;

typedef enum iSYSTemperature_unit
{
    ISYS_TEMPERATURE_UNIT_KELVIN       = 0,
    ISYS_TEMPERATURE_UNIT_CELSIUS         ,
    ISYS_TEMPERATURE_UNIT_FAHRENHEIT
} iSYSTemperature_unit_t;

typedef enum iSYSOutput_filter
{
    ISYS_OUTPUT_FILTER_HIGHEST_SIGNAL        = 0,
    ISYS_OUTPUT_FILTER_MEAN                     ,
    ISYS_OUTPUT_FILTER_MEDIAN                   ,
    ISYS_OUTPUT_FILTER_MIN                      ,
    ISYS_OUTPUT_FILTER_MAX                      ,
    /* add new detection modes before this element */
    ISYS_OUTPUT_FILTER_END
} iSYSOutput_filter_t;

typedef enum iSYSFilter_signal{
    ISYS_SIGNAL_FILTER_OFF                   = 0,
    ISYS_SIGNAL_FILTER_VELOCITY_RADIAL          ,
    ISYS_SIGNAL_FILTER_RANGE_RADIAL             ,
    ISYS_SIGNAL_FILTER_ANGLE                    ,   /* not implemented */
    ISYS_SIGNAL_FILTER_VELOCITY_X               ,   /* not implemented */
    ISYS_SIGNAL_FILTER_VELOCITY_Y               ,   /* not implemented */
    ISYS_SIGNAL_FILTER_RANGE_X                  ,   /* not implemented */
    ISYS_SIGNAL_FILTER_RANGE_Y                  ,   /* not implemented */
    /* add new detection modes before this element */
    ISYS_SIGNAL_FILTER_END
} iSYSFilter_signal_t;

typedef enum iSYSFrequencyChannel
{
    ISYS_CHANNEL_1     = 1,
    ISYS_CHANNEL_2     = 2,
    ISYS_CHANNEL_3     = 3,
    ISYS_CHANNEL_4     = 4,
    ISYS_CHANNEL_5     = 5,
    ISYS_CHANNEL_6     = 6,
    ISYS_CHANNEL_7     = 7,
    ISYS_CHANNEL_8     = 8
} iSYSFrequencyChannel_t;

typedef enum IDSDirection_type
{
    IDS_DIRECTION_POSITIVE = 1,
    IDS_DIRECTION_NEGATIVE = 2,
    IDS_DIRECTION_END
} IDSDirection_type_t;

typedef enum iSYSSaveLocation{
    ISYS_LOCATION_RAM = 0,
    ISYS_LOCATION_EEPROM = 1
}iSYSSaveLocation_t;

typedef enum iSYSMeasurementMode
{
    ISYS_MEASUREMENT_MODE_FAST      = 0,
    ISYS_MEASUREMENT_MODE_ACCURATE  = 1,
} iSYSMeasurementMode_t;

typedef enum iSYSResult
{
    ERR_OK                                  = 0x0000,
    ERR_FUNCTION_DEPRECATED                 ,
    ERR_DLL_NOT_FINISHED                    ,
    ERR_HANDLE_NOT_INITIALIZED              ,
    ERR_COMPORT_DOESNT_EXIST                ,
    ERR_COMPORT_CANT_INITIALIZE             ,
    ERR_COMPORT_ACCESS_DENIED               ,
    ERR_COMPORT_BAUDRATE_NOT_VALID          ,
    ERR_COMPORT_CANT_OPEN                   ,
    ERR_COMPORT_CANT_SET_FLOW_CONTROL       ,
    ERR_COMPORT_CANT_SET_PARITY             ,
    ERR_COMPORT_CANT_SET_STOP_BITS          ,
    ERR_COMPORT_CANT_SET_DATA_BITS          ,
    ERR_COMPORT_CANT_SET_BAUDRATE           ,
    ERR_COMPORT_ALREADY_INITIALIZED         ,
    ERR_COMPORT_EQUALS_NULL                 ,
    ERR_COMPORT_NOT_OPEN                    ,
    ERR_COMPORT_NOT_READABLE                ,
    ERR_COMPORT_NOT_WRITEABLE               ,
    ERR_COMPORT_CANT_WRITE                  ,
    ERR_COMPORT_CANT_READ                   ,
    ERR_COMMAND_NOT_WRITTEN                 ,
    ERR_COMMAND_NOT_READ                    ,
    ERR_COMMAND_NO_DATA_RECEIVED            ,
    ERR_COMMAND_NO_VALID_FRAME_FOUND        ,
    ERR_COMMAND_RX_FRAME_DAMAGED            ,
    ERR_COMMAND_FAILURE                     ,
    ERR_UNDEFINED_READ                      ,
    ERR_COMPORT_LESS_DATA_READ              ,
    ERR_COMPORT_SYSTEM_INIT_FAILED          ,
    ERR_COMPORT_SYSTEM_ALREADY_INITIALIZED  ,
    ERR_COMMAND_RX_FRAME_LENGTH             ,
    ERR_COMMAND_MAX_DATA_OVERFLOW           ,
    ERR_COMMAND_MAX_IQPAIRS_OVERFLOW        ,
    ERR_COMMAND_NOT_ACCEPTED                ,
    ERR_NULL_POINTER                        ,
    ERR_CALC_CORRECTION_PARAMS              ,
    ERR_PARAMETER_OUT_OF_RANGE
} iSYSResult_t;

typedef struct iSYSHandle *iSYSHandle_t;

typedef enum iSYSTargetListError
{
    TARGET_LIST_OK                          = 0x00,
    TARGET_LIST_FULL                        = 0x01,
    TARGET_LIST_REFRESHED                   = 0x02,
    TARGET_LIST_ALREADY_REQUESTED           = 0x03,
    TARGET_LIST_ACQUISITION_NOT_STARTED     = 0x04
}iSYSTargetListError_t;

union iSYSTargetListError_u
{
    iSYSTargetListError_t iSYSTargetListError;
    uint32_t dummy;
};

typedef struct iSYSTarget {
    float32_t velocity;         /* radial velocity in m/s */
    float32_t range;            /* range in m */
    float32_t signal;           /* signal indicator */
    float32_t angle;            /* angle of detected object [°] */
} iSYSTarget_t;

typedef struct iSYSTargetList {
    union iSYSTargetListError_u error;
    uint8_t outputNumber;
    uint16_t nrOfTargets;
    uint32_t clippingFlag;
    iSYSTarget_t targets[MAX_TARGETS];
} iSYSTargetList_t;

typedef struct iSYSComplex{
    float32_t real;
    float32_t imag;
}iSYSComplex_t;

typedef struct iSYSIQSignal{
  iSYSComplex_t IQ1A[MAX_DATA_SIZE];
  iSYSComplex_t IQ1B[MAX_DATA_SIZE];
  iSYSComplex_t IQ1C[MAX_DATA_SIZE];
  iSYSComplex_t IQ1D[MAX_DATA_SIZE];
}iSYSIQSignal_t;

typedef struct iSYSRawData{
  iSYSIQSignal_t signal;
  uint32_t signalNrOfSamples;
  uint32_t signalNrOfIQPairs;
}iSYSRawData_t;

typedef struct iSYSDetectionData{
  uint32_t clippingFlag;
  sint16_t fftMagnitude[MAX_DATA_SIZE];
  sint16_t threshold[MAX_DATA_SIZE];
}iSYSDetectionData_t;

typedef struct iSYSDetection{
  iSYSDetectionData_t detection;
  uint32_t signalNrOfSamples;
  uint32_t nrOfDetections;
}iSYSDetection_t;

typedef struct iSYSRangeList{
    sint32_t range;
    sint16_t amplitude;
    sint16_t sensor_temperature;
    sint32_t standard_deviation;
    sint32_t variance;
    sint32_t statistical_min;
    sint32_t statistical_max;
} iSYSRangeList_t;


/**************************************************************************************
 api functions
**************************************************************************************/
ISYS_API_EXPORT iSYSResult_t iSYS_initComPort(iSYSHandle_t *pHandle, uint8_t comportNr, iSYSBaudrate_t baudrate);
ISYS_API_EXPORT iSYSResult_t iSYS_initSystem(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_exitComPort(iSYSHandle_t pHandle);
ISYS_API_EXPORT iSYSResult_t iSYS_exitSystem(iSYSHandle_t pHandle, uint8_t destAddress);
ISYS_API_EXPORT iSYSResult_t iSYS_getApiVersion(float32_t *version);

/*SD2 - D0 */
ISYS_API_EXPORT iSYSResult_t iSYS_ReadDeviceName(iSYSHandle_t pHandle, char *devicename_array, uint16_t array_length, uint8_t destAddress, uint32_t timeout);

/*SD2 - D1 */
ISYS_API_EXPORT iSYSResult_t iSYS_StartAcquisition(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_StopAcquisition(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_SetDefaultTemperatureThreshold(iSYSHandle_t pHandle, float32_t *tempThldOut1, float32_t *tempThldOut2, float32_t *tempThldOut3, uint8_t destAddress, uint32_t timeout); /*iSYS-6203 only*/

/*SD2 - D2 */
ISYS_API_EXPORT iSYSResult_t iSYS_getDeviceAddress(iSYSHandle_t pHandle,uint8_t *deviceaddress, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getFrequencyChannel(iSYSHandle_t pHandle, iSYSFrequencyChannel_t *channel, uint8_t destAddress, uint32_t timeout);   /* Measurement mode must be stopped before (cmdStopAcquisition) */
ISYS_API_EXPORT iSYSResult_t iSYS_getThresholdMin(iSYSHandle_t pHandle, sint16_t *sensitivity, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getThresholdSensitivityLeft(iSYSHandle_t pHandle, uint16_t *threshold, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getThresholdSensitivityRight(iSYSHandle_t pHandle, uint16_t *threshold, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getSerialNumber(iSYSHandle_t pHandle,uint32_t *serialNumber, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getFirmwareVersion(iSYSHandle_t pHandle, uint16_t *major, uint16_t *fix, uint16_t *minor, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getMeasurementMode(iSYSHandle_t pHandle, iSYSMeasurementMode_t *mode, uint8_t destAddress, uint32_t timeout);

/*SD2 - D3 */
ISYS_API_EXPORT iSYSResult_t iSYS_setDeviceAddress(iSYSHandle_t pHandle, uint8_t deviceaddress, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setFrequencyChannel(iSYSHandle_t pHandle, iSYSFrequencyChannel_t channel, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setThresholdMin(iSYSHandle_t pHandle, sint16_t sensitivity, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setThresholdSensitivityLeft(iSYSHandle_t pHandle, uint16_t threshold, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setThresholdSensitivityRight(iSYSHandle_t pHandle, uint16_t threshold, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setMeasurementMode(iSYSHandle_t pHandle, iSYSMeasurementMode_t mode, uint8_t destAddress, uint32_t timeout);

/*SD2 - D4 */
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputAngleMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *angle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputAngleMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *angle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputRangeMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *range, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputRangeMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *range, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputSignalMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *signal, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputSignalMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *signal, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputVelocityMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *velocity, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputVelocityMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *velocity, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputDirection(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSDirection_type_t *direction, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputFilter(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t *filter, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputSignalFilter(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t *signal, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputAlphaFilterVelocity(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *filter, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputAlphaFilterRange(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *filter, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getLineFilter1(iSYSHandle_t pHandle, uint16_t *enable, uint16_t *frequency, uint16_t *offset, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getLineFilter2(iSYSHandle_t pHandle, uint16_t *enable, uint16_t *frequency, uint16_t *offset, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputEnable(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSOutput_t *enable, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputRisingDelay(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *delay, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputFallingDelay(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *delay, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputPlausibility(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *plausibility, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputSetting1(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *setting, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputSetting2(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *setting, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getPotis(iSYSHandle_t pHandle, uint16_t *value, uint8_t destAddress , uint32_t timeout); /* iSYS-4001,4002,4003 only */
ISYS_API_EXPORT iSYSResult_t iSYS_getTemperatureThreshold(iSYSHandle_t pHandle, iSYSSaveLocation_t location, float32_t *tempThldOut1, float32_t *tempThldOut2, float32_t *tempThldOut3, uint8_t destAddress, uint32_t timeout); /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_getTargetClusteringEnable(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t *enable, uint8_t destAddress , uint32_t timeout); /* iSYS-5010 only */
ISYS_API_EXPORT iSYSResult_t iSYS_getRcsOutputEnable(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t *enable, uint8_t destAddress , uint32_t timeout); /* iSYS-5010 only */

/*SD2 - D5 */
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputAngleMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t angle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputAngleMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t angle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputRangeMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputRangeMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputSignalMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputSignalMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputVelocityMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputVelocityMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputDirection(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSDirection_type_t direction, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputFilter(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t filter, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputSignalFilter(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t signal, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputAlphaFilterVelocity(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t filter, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputAlphaFilterRange(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t filter, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setLineFilter1(iSYSHandle_t pHandle, uint16_t enable, uint16_t frequency, uint16_t offset, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setLineFilter2(iSYSHandle_t pHandle, uint16_t enable, uint16_t frequency, uint16_t offset, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputEnable(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSOutput_t enable, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputRisingDelay(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t delay, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputFallingDelay(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t delay, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputPlausibility(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t plausibility, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputSetting1(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t setting, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputSetting2(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t setting, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setPotis(iSYSHandle_t pHandle, uint16_t value, uint8_t destAddress , uint32_t timeout);    /* iSYS-4001,4002,4003 only */
ISYS_API_EXPORT iSYSResult_t iSYS_setTemperatureThreshold(iSYSHandle_t pHandle, iSYSSaveLocation_t location, float32_t tempThldOut1, float32_t tempThldOut2, float32_t tempThldOut3, uint8_t destAddress, uint32_t timeout);  /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_setTargetClusteringEnable(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t enable, uint8_t destAddress , uint32_t timeout); /* iSYS-5010 only */
ISYS_API_EXPORT iSYSResult_t iSYS_setRcsOutputEnable(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t enable, uint8_t destAddress , uint32_t timeout); /* iSYS-5010 only */


/*SD2 - D6 */
ISYS_API_EXPORT iSYSResult_t iSYS_getDspHardwareVersion(iSYSHandle_t pHandle, uint16_t *major, uint16_t *fix, uint16_t *minor, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getRfeHardwareVersion(iSYSHandle_t pHandle, uint16_t *major, uint16_t *fix, uint16_t *minor, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getProductInfo(iSYSHandle_t pHandle, uint16_t *productInfo, uint8_t destAddress, uint32_t timeout);


/*SD2 - DA Sensor must be in measurement mode (iSYS_StartAcquisition) */
ISYS_API_EXPORT iSYSResult_t iSYS_getTargetList(iSYSHandle_t pHandle, iSYSTargetList_t *pTargetList, iSYSOutputNumber_t outputnumber, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getTargetList16(iSYSHandle_t pHandle, iSYSTargetList_t *pTargetList, iSYSOutputNumber_t outputnumber, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getTargetList32(iSYSHandle_t pHandle, iSYSTargetList_t *pTargetList, iSYSOutputNumber_t outputnumber, uint8_t destAddress, uint32_t timeout);

/*SD2 - DB Sensor must be in measurement mode (iSYS_StartAcquisition) */
ISYS_API_EXPORT iSYSResult_t iSYS_getDigitalOutputState(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, sint16_t *value, uint8_t destAddress, uint32_t timeout);

/*SD2 - DC Sensor must be in measurement mode (iSYS_StartAcquisition) */
ISYS_API_EXPORT iSYSResult_t iSYS_getRawData(iSYSHandle_t pHandle, iSYSRawData_t *data, uint8_t destAddress, uint32_t timeout);

/*SD2 - DD Sensor must be in measurement mode (iSYS_StartAcquisition) */
ISYS_API_EXPORT iSYSResult_t iSYS_getFftData(iSYSHandle_t pHandle, iSYSDetection_t *data, uint8_t destAddress, uint32_t timeout);

/*SD2 - DE */

/*SD2 - DF */
ISYS_API_EXPORT iSYSResult_t iSYS_setFactorySetting(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_saveSensorSettings(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_saveApplicationSettings(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_saveSensorAndApplicationSettings(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);

/* iSYS-6003 only */
/*SD2 - E1 Sensor must be in measurement mode (iSYS_StartAcquisition) */
ISYS_API_EXPORT iSYSResult_t iSYS_getRangeList(iSYSHandle_t pHandle, iSYSRangeList_t *pRangeList, uint8_t destAddress, uint32_t timeout);


/* IDS-1000 only (default destAddress = 128) */
/* set functions */
ISYS_API_EXPORT iSYSResult_t IDS_setOutputEnable(iSYSHandle_t pHandle, bool_t enable, uint8_t destAddress); /* enable = 0 -> OFF ; enable = 1 -> ON */
ISYS_API_EXPORT iSYSResult_t IDS_setFrequency(iSYSHandle_t pHandle, uint32_t frequency, uint8_t destAddress); /* set frequency in Hz */
ISYS_API_EXPORT iSYSResult_t IDS_setDirection(iSYSHandle_t pHandle, IDSDirection_type_t direction, uint8_t destAddress);

/* get functions */
ISYS_API_EXPORT iSYSResult_t IDS_getOutputEnable(iSYSHandle_t pHandle, bool_t *enable, uint8_t destAddress); /* enable = 0 -> OFF ; enable = 1 -> ON */
ISYS_API_EXPORT iSYSResult_t IDS_getFrequency(iSYSHandle_t pHandle, uint32_t *frequency, uint8_t destAddress); /* get frequency in Hz */
ISYS_API_EXPORT iSYSResult_t IDS_getDirection(iSYSHandle_t pHandle, IDSDirection_type_t *direction, uint8_t destAddress);

/* save settings to eeprom */
ISYS_API_EXPORT iSYSResult_t IDS_saveSettings(iSYSHandle_t pHandle, uint8_t destAddress);


#ifdef __cplusplus
}
#endif

#endif // ISYS_API_IF_H
