#ifndef TM_L3GD20_H
#define TM_L3GD20_H

#include "stm32f4xx.h"
#include "main.h"

#define L3GD20H_SPI_CS_GPIO_PORT 	GPIOE
#define L3GD20H_SPI_CS_PIN			GPIO_PIN_2
#define L3GD20H_CS_LOW()       HAL_GPIO_WritePin(L3GD20H_SPI_CS_GPIO_PORT, L3GD20H_SPI_CS_PIN, GPIO_PIN_RESET);
#define L3GD20H_CS_HIGH()      HAL_GPIO_WritePin(L3GD20H_SPI_CS_GPIO_PORT, L3GD20H_SPI_CS_PIN, GPIO_PIN_SET);

/* Identification number */
#define L3GD20_WHO_AM_I				0xD7

/* Registers addresses */
#define L3GD20_REG_WHO_AM_I			0x0F
#define L3GD20_REG_CTRL_REG1		0x20
#define L3GD20_REG_CTRL_REG2		0x21
#define L3GD20_REG_CTRL_REG3		0x22
#define L3GD20_REG_CTRL_REG4		0x23
#define L3GD20_REG_CTRL_REG5		0x24
#define L3GD20_REG_REFERENCE		0x25
#define L3GD20_REG_OUT_TEMP			0x26
#define L3GD20_REG_STATUS_REG		0x27
#define L3GD20_REG_OUT_X_L			0x28
#define L3GD20_REG_OUT_X_H			0x29
#define L3GD20_REG_OUT_Y_L			0x2A
#define L3GD20_REG_OUT_Y_H			0x2B
#define L3GD20_REG_OUT_Z_L			0x2C
#define L3GD20_REG_OUT_Z_H			0x2D
#define L3GD20_REG_FIFO_CTRL_REG	0x2E
#define L3GD20_REG_FIFO_SRC_REG		0x2F
#define L3GD20_REG_INT1_CFG			0x30
#define L3GD20_REG_INT1_SRC			0x31
#define L3GD20_REG_INT1_TSH_XH		0x32
#define L3GD20_REG_INT1_TSH_XL		0x33
#define L3GD20_REG_INT1_TSH_YH		0x34
#define L3GD20_REG_INT1_TSH_YL		0x35
#define L3GD20_REG_INT1_TSH_ZH		0x36
#define L3GD20_REG_INT1_TSH_ZL		0x37
#define L3GD20_REG_INT1_DURATION	0x38

#define L3GD20_SENSITIVITY_250		8.75	/* 8.75 mdps/digit */
#define L3GD20_SENSITIVITY_500		17.5	/* 17.5 mdps/digit */
#define L3GD20_SENSITIVITY_2000		70		/* 70 mdps/digit */

typedef struct {
	int16_t X; /*!< X axis rotation */
	int16_t Y; /*!< Y axis rotation */
	int16_t Z; /*!< Z axis rotation */
} TM_L3GD20_t;

typedef enum {
	TM_L3GD20_Result_Ok,   /*!< Everything OK */
	TM_L3GD20_Result_Error /*!< Error occurred */
} TM_L3GD20_Result_t;

typedef enum {
	TM_L3GD20_Scale_250, /*!< Set full scale to 250 mdps */
	TM_L3GD20_Scale_500, /*!< Set full scale to 500 mdps */
	TM_L3GD20_Scale_2000 /*!< Set full scale to 2000 mdps */
} TM_L3GD20_Scale_t;

TM_L3GD20_Result_t TM_L3GD20_Init();
TM_L3GD20_Result_t TM_L3GD20_Read(TM_L3GD20_t* L3DG20_Data);
void TM_L3GD20_WRITE_REG(uint8_t u8reg, uint8_t value);
int8_t TM_L3GD20_READ_REG(uint8_t u8reg);
uint8_t TM_L3GD20_INT_ReadSPI(uint8_t address);

#endif
