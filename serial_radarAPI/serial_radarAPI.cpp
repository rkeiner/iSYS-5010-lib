
#include <string.h>
#include "serial_radarAPI_basicTypes.h"
#include "serial_stack.h"

using namespace std;
/*  nothing in the api file
iSYS_setProcessingRcsCalibrationEnable
iSYS_getProcessingRcsCalibrationEnable

iSYS_setThresholdMovingTargetsNearRangeMargin
iSYS_getThresholdMovingTargetsNearRangeMargin
iSYS_setThresholdMovingTargetsMainRangeMargin
iSYS_getThresholdMovingTargetsMainRangeMargin
iSYS_setThresholdMovingTargetsLongRangeMargin
iSYS_getThresholdMovingTargetsLongRangeMargin

#define FRAME_TYPE_NO_DATA 0x10
#define FRAME_TYPE_VARIABLE_DATA 0x68
#define FRAME_TYPE_FIXED_DATA 0xA2
*/

uint8_t calc_fcs(device_cmd_t cmd)
{
	return cmd.destination + cmd.source + cmd.function + cmd.pdu;
}

void populate_cmd(device_cmd_t cmd, uint8_t dest, uint8_t type, uint8_t function, uint16_t sub_func)
{
	cmd.frametype = type;
	cmd.destination = dest;
	cmd.function = function;
	cmd.pdu = 0x00;
	cmd.fcs = calc_fcs(cmd);
	switch (type)
	{
	case FRAME_TYPE_NO_DATA:
	{
		break;
	}
	case FRAME_TYPE_VARIABLE_DATA:
	{
		break;
	}
	case FRAME_TYPE_FIXED_DATA:
	{
		break;
	}
	default:
		break;
	}
}

iSYSResult_t send_command(device_cmd_t cmd, char *retval, uint32_t timeout)
{
	return ERR_OK;
}
/**************************************************************************************
 api functions
**************************************************************************************/
iSYSResult_t iSYS_initComPort(iSYSHandle_t *pHandle, uint8_t comportNr, iSYSBaudrate_t baudrate)
{
	return ERR_OK;
}

iSYSResult_t iSYS_initSystem(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout)
{
	return ERR_OK;
}
iSYSResult_t iSYS_exitComPort(iSYSHandle_t pHandle)
{
	return ERR_OK;
}
iSYSResult_t iSYS_exitSystem(iSYSHandle_t pHandle, uint8_t destAddress)
{
	return ERR_OK;
}
iSYSResult_t iSYS_getApiVersion(float32_t *version)
{
	return ERR_OK;
}

/*SD2 - D0 */
iSYSResult_t iSYS_ReadDeviceName(iSYSHandle_t pHandle, char *devicename_array, uint16_t array_length, uint8_t destAddress, uint32_t timeout)
{
	iSYSResult_t res;
	device_cmd_t radar_cmd;
	memset(&radar_cmd, 0, sizeof radar_cmd);
	populate_cmd(radar_cmd, destAddress, FRAME_TYPE_VARIABLE_DATA, CMD_READ_DEVICE, 0);
	res = send_command(radar_cmd, devicename_array, timeout);
	if (res != ERR_OK)
	{
		return res;
	}
	return ERR_OK;
}

/*SD2 - D1 */
iSYSResult_t iSYS_StartAcquisition(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout)
{
	iSYSResult_t res;
	device_cmd_t radar_cmd;
	memset(&radar_cmd, 0, sizeof radar_cmd);

	populate_cmd(radar_cmd, destAddress, FRAME_TYPE_VARIABLE_DATA, CMD_ACQISITION, SUB_FUNC_START_ACQUISITION);
	res = send_command(radar_cmd, nullptr, timeout);
	if (res != ERR_OK)
	{
		return res;
	}
	return ERR_OK;
}
iSYSResult_t iSYS_StopAcquisition(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout)
{
	iSYSResult_t res;
	device_cmd_t radar_cmd;
	memset(&radar_cmd, 0, sizeof radar_cmd);

	populate_cmd(radar_cmd, destAddress, FRAME_TYPE_VARIABLE_DATA, CMD_ACQISITION, SUB_FUNC_STOP_ACQUISITION);
	res = send_command(radar_cmd, nullptr, timeout);
	if (res != ERR_OK)
	{
		return res;
	}
	return ERR_OK;
}

/*SD2 - D2 */
iSYSResult_t iSYS_getDeviceAddress(iSYSHandle_t pHandle, uint8_t *deviceaddress, uint8_t destAddress, uint32_t timeout)
{
	return ERR_OK;
}
iSYSResult_t iSYS_getFrequencyChannel(iSYSHandle_t pHandle, iSYSFrequencyChannel_t *channel, uint8_t destAddress, uint32_t timeout) /* Measurement mode must be stopped before (cmdStopAcquisition) */
{
	iSYSResult_t res;
	device_cmd_t radar_cmd;
	memset(&radar_cmd, 0, sizeof radar_cmd);

	populate_cmd(radar_cmd, destAddress, FRAME_TYPE_VARIABLE_DATA, CMD_READ_SENSOR, SUB_FUNC_FREQUENCY_CHANNEL);
	res = send_command(radar_cmd, nullptr, timeout);
	if (res != ERR_OK)
	{
		return res;
	}
	return ERR_OK;
}

iSYSResult_t iSYS_getThresholdMin(iSYSHandle_t pHandle, sint16_t *sensitivity, uint8_t destAddress, uint32_t timeout)
{
	iSYSResult_t res;
	device_cmd_t radar_cmd;
	memset(&radar_cmd, 0, sizeof radar_cmd);

	populate_cmd(radar_cmd, destAddress, FRAME_TYPE_VARIABLE_DATA, CMD_READ_SENSOR, SUB_FUNC_MINIMUM);
	res = send_command(radar_cmd, nullptr, timeout);
	if (res != ERR_OK)
	{
		return res;
	}
	return ERR_OK;
}
iSYSResult_t iSYS_getSerialNumber(iSYSHandle_t pHandle, uint32_t *serialNumber, uint8_t destAddress, uint32_t timeout)
{
	iSYSResult_t res;
	device_cmd_t radar_cmd;
	memset(&radar_cmd, 0, sizeof radar_cmd);

	populate_cmd(radar_cmd, destAddress, FRAME_TYPE_VARIABLE_DATA, CMD_READ_DEVICE, 0);
	// res = send_command(radar_cmd, serialNumber, timeout);
	res = send_command(radar_cmd, nullptr, timeout);
	if (res != ERR_OK)
	{
		return res;
	}
	return ERR_OK;
}
iSYSResult_t iSYS_getFirmwareVersion(iSYSHandle_t pHandle, uint16_t *major, uint16_t *fix, uint16_t *minor, uint8_t destAddress, uint32_t timeout)
{
	iSYSResult_t res;
	device_cmd_t radar_cmd;
	memset(&radar_cmd, 0, sizeof radar_cmd);

	populate_cmd(radar_cmd, destAddress, FRAME_TYPE_VARIABLE_DATA, CMD_READ_CALIBRATION_SETTINGS, SUB_FUNC_FIRMWARE_VERSION);
	res = send_command(radar_cmd, nullptr, timeout);
	if (res != ERR_OK)
	{
		return res;
	}
	return ERR_OK;
}

/*SD2 - D3 */

iSYSResult_t iSYS_setDeviceAddress(iSYSHandle_t pHandle, uint8_t deviceaddress, uint8_t destAddress, uint32_t timeout)
{
	return ERR_OK;
}
iSYSResult_t iSYS_setFrequencyChannel(iSYSHandle_t pHandle, iSYSFrequencyChannel_t channel, uint8_t destAddress, uint32_t timeout)
{
	iSYSResult_t res;
	device_cmd_t radar_cmd;
	memset(&radar_cmd, 0, sizeof radar_cmd);

	populate_cmd(radar_cmd, destAddress, FRAME_TYPE_VARIABLE_DATA, CMD_WRITE_SENSOR, SUB_FUNC_FREQUENCY_CHANNEL);
	res = send_command(radar_cmd, nullptr, timeout);
	if (res != ERR_OK)
	{
		return res;
	}
	return ERR_OK;
}
iSYSResult_t iSYS_setThresholdMin(iSYSHandle_t pHandle, sint16_t sensitivity, uint8_t destAddress, uint32_t timeout)
{
	iSYSResult_t res;
	device_cmd_t radar_cmd;
	memset(&radar_cmd, 0, sizeof radar_cmd);

	populate_cmd(radar_cmd, destAddress, FRAME_TYPE_VARIABLE_DATA, CMD_WRITE_SENSOR, SUB_FUNC_MINIMUM);
	res = send_command(radar_cmd, nullptr, timeout);
	if (res != ERR_OK)
	{
		return res;
	}
	return ERR_OK;
}

/*SD2 - D6 */
iSYSResult_t iSYS_getDspHardwareVersion(iSYSHandle_t pHandle, uint16_t *major, uint16_t *fix, uint16_t *minor, uint8_t destAddress, uint32_t timeout)
{
	iSYSResult_t res;
	device_cmd_t radar_cmd;
	memset(&radar_cmd, 0, sizeof radar_cmd);

	populate_cmd(radar_cmd, destAddress, FRAME_TYPE_VARIABLE_DATA, CMD_READ_CALIBRATION_SETTINGS, SUB_FUNC_DSP_HARDWARE_VERSION);
	res = send_command(radar_cmd, nullptr, timeout);
	if (res != ERR_OK)
	{
		return res;
	}
	return ERR_OK;
}
iSYSResult_t iSYS_getRfeHardwareVersion(iSYSHandle_t pHandle, uint16_t *major, uint16_t *fix, uint16_t *minor, uint8_t destAddress, uint32_t timeout)
{
	iSYSResult_t res;
	device_cmd_t radar_cmd;
	memset(&radar_cmd, 0, sizeof radar_cmd);

	populate_cmd(radar_cmd, destAddress, FRAME_TYPE_VARIABLE_DATA, CMD_READ_CALIBRATION_SETTINGS, SUB_FUNC_RFE_HARDWARE_VERSION);
	res = send_command(radar_cmd, nullptr, timeout);
	if (res != ERR_OK)
	{
		return res;
	}
	return ERR_OK;
}
iSYSResult_t iSYS_getProductInfo(iSYSHandle_t pHandle, uint16_t *productInfo, uint8_t destAddress, uint32_t timeout)
{
	iSYSResult_t res;
	device_cmd_t radar_cmd;
	memset(&radar_cmd, 0, sizeof radar_cmd);

	populate_cmd(radar_cmd, destAddress, FRAME_TYPE_VARIABLE_DATA, CMD_READ_CALIBRATION_SETTINGS, SUB_FUNC_PRODUCT_INFO);
	res = send_command(radar_cmd, nullptr, timeout);
	if (res != ERR_OK)
	{
		return res;
	}
	return ERR_OK;
}

/* get functions */
iSYSResult_t IDS_getOutputEnable(iSYSHandle_t pHandle, bool_t *enable, uint8_t destAddress) /* enable = 0 -> OFF ; enable = 1 -> ON */
{
	return ERR_OK;
}
iSYSResult_t IDS_getFrequency(iSYSHandle_t pHandle, uint32_t *frequency, uint8_t destAddress) /* get frequency in Hz */
{
	iSYSResult_t res;
	device_cmd_t radar_cmd;
	memset(&radar_cmd, 0, sizeof radar_cmd);

	populate_cmd(radar_cmd, destAddress, FRAME_TYPE_VARIABLE_DATA, CMD_READ_SENSOR, SUB_FUNC_FREQUENCY_CHANNEL);
	res = send_command(radar_cmd, nullptr, 0);
	if (res != ERR_OK)
	{
		return res;
	}
	return ERR_OK;
}
iSYSResult_t IDS_getDirection(iSYSHandle_t pHandle, IDSDirection_type_t *direction, uint8_t destAddress)
{
	return ERR_OK;
}
/* save settings to eeprom */
iSYSResult_t IDS_saveSettings(iSYSHandle_t pHandle, uint8_t destAddress)
{
	return ERR_OK;
}

iSYSResult_t iSYS_setTargetClusteringEnable(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t enable, uint8_t destAddress, uint32_t timeout) /* iSYS-5010 only */
{
	return ERR_OK;
}
iSYSResult_t iSYS_getTargetClusteringEnable(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t *enable, uint8_t destAddress, uint32_t timeout) /* iSYS-5010 only */
{
	return ERR_OK;
}

