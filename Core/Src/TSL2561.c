/*
 * TSL2561.c
 *
 *  Created on: Feb 11, 2023
 *      Author: Emre Dur
 */

#include "TSL2561.h"

HAL_StatusTypeDef TSL2561_Init(TSL2561* tsl, I2C_HandleTypeDef* userHandle, uint8_t sensorAddress)
{
	//Pass the I2C handle to the sensor struct
	tsl->i2cHandle = userHandle;

	//Pass the sensor address to the sensor struct
	tsl->address = sensorAddress << 1;

	//Power on sequence
	return TSL2561_WriteByte(tsl, TSL2561_COMMAND_STATE_CMD | TSL2561_REG_CONTROL, TSL2561_POWER_ON_STATE);

}

HAL_StatusTypeDef TSL2561_GetLux(TSL2561* tsl, float* luxValue)
{
	//Variables for storing Data registers
	uint16_t data0, data1;

	//Get the values from the data registers
	TSL2561_ReadData(tsl, &data0, &data1);

	//T, FN, and CL Package calculation
	//Calculate the ratio of the photodiodes
	float ratio = ((float)data1) / ((float)data0);

	//Calculating Lux
	//Ratio between 0 and 0.50
	if(ratio <= 0.50) *luxValue = 0.0304 * data0 - 0.062* data0 * pow(ratio, 1.4);

	//If the ratio is between 0.51 and 0.61
	else if(ratio <= 0.61) *luxValue = 0.0224 * data0 - 0.031 * data1;

	//If the ratio is between 0.62 and 0.81
	else if(ratio <= 0.80) *luxValue = 0.0128 * data0 - 0.0153 * data1;

	//If the ratio is between 0.82 and 1.39
	else if(ratio <= 1.30) *luxValue = 0.00146 * data0 - 0.00112 * data1;

	//If the ratio is bigger than 1.30
	else *luxValue = 0;

	return HAL_OK;
}

HAL_StatusTypeDef TSL2561_ReadData(TSL2561* tsl, uint16_t* data0Val, uint16_t* data1Val)
{
	//Variables for storing received data
	uint8_t data0Low, data0High, data1Low, data1High;

	//Read Data0 registers
	TSL2561_ReadByte(tsl, (TSL2561_COMMAND_STATE_CMD|TSL2561_REG_DATA0LOW),	&data0Low);
	TSL2561_ReadByte(tsl, (TSL2561_COMMAND_STATE_CMD|TSL2561_REG_DATA0HIGH), &data0High);

	//Read Data1 registers
	TSL2561_ReadByte(tsl, (TSL2561_COMMAND_STATE_CMD|TSL2561_REG_DATA1LOW),	&data1Low);
	TSL2561_ReadByte(tsl, (TSL2561_COMMAND_STATE_CMD|TSL2561_REG_DATA1HIGH), &data1High);

	//Combine Data register values into single 16 bit variable
	*data0Val = data0High * 256 + data0Low;
	*data1Val = data1High * 256 + data1Low;

	return HAL_OK;
}

HAL_StatusTypeDef TSL2561_WriteByte(TSL2561* tsl, uint8_t reg, uint8_t value)
{
	return HAL_I2C_Mem_Write(tsl->i2cHandle, tsl->address, reg, 1, &value, 1, TSL2561_MAX_DELAY);
}

HAL_StatusTypeDef TSL2561_ReadByte(TSL2561* tsl, uint8_t reg, uint8_t* value)
{
	return HAL_I2C_Mem_Read(tsl->i2cHandle, tsl->address, reg, 1, value, 1, TSL2561_MAX_DELAY);
}
