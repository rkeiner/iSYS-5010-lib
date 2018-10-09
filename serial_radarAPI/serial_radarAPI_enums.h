/**************************************************************************************

      II    N     N     N     N      OOOO      SSSSS     EEEEE    N     N    TTTTTTT
     II    NNN   N     NNN   N    OO    OO    S         E        NNN   N       T
    II    N NN  N     N NN  N    OO    OO    SSSSS     EEE      N NN  N       T
   II    N  NN N     N  NN N    OO    OO        S     E        N  NN N       T
  II    N    NN     N    NN      OOOO      SSSSS     EEEEE    N    NN       T
                         copyright (c) 2011, InnoSenT GmbH
                                 all rights reserved

***************************************************************************************

    filename:		serial_radarAPI_enums.h
    brief:			enums that uses serial radar api
    creation:		07.12.2016
    author:			Benjamin Gruenewald
    version:		v1.0


**************************************************************************************/

#ifndef INCLUSION_GUARDS_SERIAL_RADARAPI_ENUMS_H
#define INCLUSION_GUARDS_SERIAL_RADARAPI_ENUMS_H


/**************************************************************************************
   serial radar standard enumerators
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

typedef enum iSYSSaveLocation{
    ISYS_LOCATION_RAM = 0,
    ISYS_LOCATION_EEPROM = 1
}iSYSSaveLocation_t;

typedef enum iSYSMeasurementMode
{
    ISYS_MEASUREMENT_MODE_FAST      = 0,
    ISYS_MEASUREMENT_MODE_ACCURATE  = 1,
} iSYSMeasurementMode_t;


typedef enum iSYSTargetListError
{
    TARGET_LIST_OK                          = 0x00,
    TARGET_LIST_FULL                        = 0x01,
    TARGET_LIST_REFRESHED                   = 0x02,
    TARGET_LIST_ALREADY_REQUESTED           = 0x03,
    TARGET_LIST_ACQUISITION_NOT_STARTED     = 0x04
}iSYSTargetListError_t;

typedef enum iSYSApplicationMode
{
    ISYS_TARGET_LIST_MODE                   = 0x00,
    ISYS_RAW_DATA_MODE                      = 0x01
} iSYSApplicationMode_t;

typedef enum ISYSTestOutput
{
    ISYS_LIVE_DATA                     = 0,   /* disable all test outputs */
    ISYS_RAWDATA_PATTERN_RISING_RAMP   = 1,
}ISYSTestOutput_t;


typedef enum IDSDirection_type
{
    IDS_DIRECTION_POSITIVE = 1,
    IDS_DIRECTION_NEGATIVE = 2,
    IDS_DIRECTION_END
} IDSDirection_type_t;

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

#endif /* INCLUSION_GUARDS_SERIAL_RADARAPI_ENUMS_H */
