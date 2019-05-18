#include "stm32f4_discovery_l3gd20.h"

extern SPI_HandleTypeDef hspi2;
TM_L3GD20_Scale_t TM_L3GD20_INT_Scale;

TM_L3GD20_Result_t TM_L3GD20_Init() {

	static uint8_t read_Data;

	read_Data = TM_L3GD20_READ_REG(L3GD20_REG_WHO_AM_I);
	if (read_Data != L3GD20_WHO_AM_I) {
		/* Sensor connected is not L3GD20 */
		return TM_L3GD20_Result_Error;
	}
	/* Enable L3GD20 Power bit */
	//TM_L3GD20_INT_WriteSPI
	TM_L3GD20_WRITE_REG(L3GD20_REG_CTRL_REG1, 0xFF);
//	data[0] = L3GD20_REG_CTRL_REG1;
//	data[1] = 0xFF;
//	L3GD20H_CS_LOW();
//	HAL_SPI_Transmit(&hspi2,data,2,1);
//	L3GD20H_CS_HIGH();

	/* Set L3GD20 scale
	if (scale == TM_L3GD20_Scale_250) {
		TM_L3GD20_INT_WriteSPI(L3GD20_REG_CTRL_REG4, 0x00);
	} else if (scale == TM_L3GD20_Scale_500) {
		TM_L3GD20_INT_WriteSPI(L3GD20_REG_CTRL_REG4, 0x10);
	} else if (scale == TM_L3GD20_Scale_2000) {
		TM_L3GD20_INT_WriteSPI(L3GD20_REG_CTRL_REG4, 0x20);
	}*/
//	data[0] = L3GD20_REG_CTRL_REG4;
//	data[1] = 0x20;

	TM_L3GD20_WRITE_REG(L3GD20_REG_CTRL_REG4, 0x00);
//	L3GD20H_CS_LOW();
//	HAL_SPI_Transmit(&hspi2,data,2,1);
//	L3GD20H_CS_HIGH();


	/* Set high-pass filter settings */
	//TM_L3GD20_INT_WriteSPI(L3GD20_REG_CTRL_REG2, 0x00);

	TM_L3GD20_WRITE_REG(L3GD20_REG_CTRL_REG2, 0x00);
//	data[0] = L3GD20_REG_CTRL_REG2;
//	data[1] = 0x00;
//
//	L3GD20H_CS_LOW();
//	HAL_SPI_Transmit(&hspi2,data,2,1);
//	L3GD20H_CS_HIGH();

	/* Enable high-pass filter */
//	TM_L3GD20_INT_WriteSPI(L3GD20_REG_CTRL_REG5, 0x10);

	TM_L3GD20_WRITE_REG(L3GD20_REG_CTRL_REG5, 0x10);
//	data[0] = L3GD20_REG_CTRL_REG5;
//	data[1] = 0x10;
//
//	L3GD20H_CS_LOW();
//	HAL_SPI_Transmit(&hspi2,data,2,1);
//	L3GD20H_CS_HIGH();


	/* Everything OK */
	return TM_L3GD20_Result_Ok;
}

TM_L3GD20_Result_t TM_L3GD20_Read(TM_L3GD20_t* L3DG20_Data) {
	float temp, s;

	L3DG20_Data->X = TM_L3GD20_READ_REG(L3GD20_REG_OUT_X_L);
	L3DG20_Data->X |= (int16_t)(TM_L3GD20_READ_REG(L3GD20_REG_OUT_X_H)) << 8;
	L3DG20_Data->Y = TM_L3GD20_READ_REG(L3GD20_REG_OUT_Y_L);
	L3DG20_Data->Y |= (int16_t)(TM_L3GD20_READ_REG(L3GD20_REG_OUT_Y_H)) << 8;
	L3DG20_Data->Z = TM_L3GD20_READ_REG(L3GD20_REG_OUT_Z_L);
	L3DG20_Data->Z |= (int16_t)(TM_L3GD20_READ_REG(L3GD20_REG_OUT_Z_H)) << 8;

	s = L3GD20_SENSITIVITY_250 * 0.001;

	temp = (float)L3DG20_Data->X * s;
	L3DG20_Data->X = (int16_t) temp;
	temp = (float)L3DG20_Data->Y * s;
	L3DG20_Data->Y = (int16_t) temp;
	temp = (float)L3DG20_Data->Z * s;
	L3DG20_Data->Z = (int16_t) temp;

	/* Return OK */
	return TM_L3GD20_Result_Ok;
}

int8_t TM_L3GD20_READ_REG(uint8_t u8reg)
{
	int8_t data[1];

	data[0] = u8reg| 0x80;
	L3GD20H_CS_LOW();
	HAL_SPI_Transmit(&hspi2,data,1,1);
	HAL_SPI_Receive(&hspi2,data,1,10);
	L3GD20H_CS_HIGH();

	return (int8_t)data[0];
}

void TM_L3GD20_WRITE_REG(uint8_t u8reg, uint8_t value)
{
	uint8_t data[2];

	data[0] = u8reg;
	data[1] = value;
	L3GD20H_CS_LOW();
	HAL_SPI_Transmit(&hspi2,data,2,1);
	L3GD20H_CS_HIGH();
}


uint8_t TM_L3GD20_INT_ReadSPI(uint8_t address)
{
	uint8_t data[1] = {0};

	data[0] = address;
	L3GD20H_CS_LOW();
	HAL_SPI_Receive(&hspi2,data,1,1);
	L3GD20H_CS_HIGH();

	return data[0];
}


