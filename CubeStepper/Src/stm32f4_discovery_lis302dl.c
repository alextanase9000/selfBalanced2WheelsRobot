/**
  ******************************************************************************
  * @file    stm32f4_discovery_lis302dl.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    28-October-2011
  * @brief   This file provides a set of functions needed to manage the LIS302DL
  *          MEMS accelerometer available on STM32F4-Discovery Kit.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery_lis302dl.h"

extern SPI_HandleTypeDef hspi1;
/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup STM32F4_DISCOVERY
  * @{
  */ 

/** @addtogroup STM32F4_DISCOVERY_LIS302DL
  * @{
  */


/** @defgroup STM32F4_DISCOVERY_LIS302DL_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F4_DISCOVERY_LIS302DL_Private_Defines
  * @{
  */
__IO uint32_t  LIS302DLTimeout = LIS302DL_FLAG_TIMEOUT;   

/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

/**
  * @}
  */
void TM_LIS302DL_Read(TM_LIS302DL_t* L3DG20_Data)
{
	uint8_t Buffer[6];

	LIS302DL_Read(Buffer, LIS302DL_OUT_X_ADDR, 6);
	L3DG20_Data->X = Buffer[0];
	L3DG20_Data->Y = Buffer[2];
	L3DG20_Data->Z = Buffer[4];
}
/** @defgroup STM32F4_DISCOVERY_LIS302DL_Private_Macros
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup STM32F4_DISCOVERY_LIS302DL_Private_Variables
  * @{
  */ 

/**
  * @}
  */

/** @defgroup STM32F4_DISCOVERY_LIS302DL_Private_FunctionPrototypes
  * @{
  */


/**
  * @}
  */

/** @defgroup STM32F4_DISCOVERY_LIS302DL_Private_Functions
  * @{
  */


/**
  * @brief  Set LIS302DL Initialization.
  * @param  LIS302DL_Config_Struct: pointer to a LIS302DL_Config_TypeDef structure 
  *         that contains the configuration setting for the LIS302DL.
  * @retval None
  */
void LIS302DL_Init(LIS302DL_InitTypeDef *LIS302DL_InitStruct)
{
  uint8_t ctrl = 0x00;
  
  /* Configure the low level interface ---------------------------------------*/
  //LIS302DL_LowLevel_Init();
  
  /* Configure MEMS: data rate, power mode, full scale, self test and axes */
  ctrl = (uint8_t) (LIS302DL_InitStruct->Output_DataRate | LIS302DL_InitStruct->Power_Mode | \
                    LIS302DL_InitStruct->Full_Scale | LIS302DL_InitStruct->Self_Test | \
                    LIS302DL_InitStruct->Axes_Enable);
  
  /* Write value to MEMS CTRL_REG1 regsister */
  LIS302DL_Write(&ctrl, LIS302DL_CTRL_REG1_ADDR, 1);
}

/**
  * @brief  Set LIS302DL Internal High Pass Filter configuration.
  * @param  LIS302DL_Filter_ConfigTypeDef: pointer to a LIS302DL_FilterConfig_TypeDef 
  *         structure that contains the configuration setting for the LIS302DL Filter.
  * @retval None
  */
void LIS302DL_FilterConfig(LIS302DL_FilterConfigTypeDef *LIS302DL_FilterConfigStruct)
{
  uint8_t ctrl = 0x00;
  
  /* Read CTRL_REG2 register */
  LIS302DL_Read(&ctrl, LIS302DL_CTRL_REG2_ADDR, 1);
  
  /* Clear high pass filter cut-off level, interrupt and data selection bits*/
  ctrl &= (uint8_t)~(LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER | \
                     LIS302DL_HIGHPASSFILTER_LEVEL_3 | \
                     LIS302DL_HIGHPASSFILTERINTERRUPT_1_2);
  /* Configure MEMS high pass filter cut-off level, interrupt and data selection bits */                     
  ctrl |= (uint8_t)(LIS302DL_FilterConfigStruct->HighPassFilter_Data_Selection | \
                    LIS302DL_FilterConfigStruct->HighPassFilter_CutOff_Frequency | \
                    LIS302DL_FilterConfigStruct->HighPassFilter_Interrupt);
  
  /* Write value to MEMS CTRL_REG2 register */
  LIS302DL_Write(&ctrl, LIS302DL_CTRL_REG2_ADDR, 1);
}

/**
  * @brief Set LIS302DL Interrupt configuration
  * @param  LIS302DL_InterruptConfig_TypeDef: pointer to a LIS302DL_InterruptConfig_TypeDef 
  *         structure that contains the configuration setting for the LIS302DL Interrupt.
  * @retval None
  */
void LIS302DL_InterruptConfig(LIS302DL_InterruptConfigTypeDef *LIS302DL_IntConfigStruct)
{
  uint8_t ctrl = 0x00;
  
  /* Read CLICK_CFG register */
  LIS302DL_Read(&ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);
  
  /* Configure latch Interrupt request, click interrupts and double click interrupts */                   
  ctrl = (uint8_t)(LIS302DL_IntConfigStruct->Latch_Request| \
                   LIS302DL_IntConfigStruct->SingleClick_Axes | \
                   LIS302DL_IntConfigStruct->DoubleClick_Axes);
  
  /* Write value to MEMS CLICK_CFG register */
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);
}

/**
  * @brief  Change the lowpower mode for LIS302DL
  * @param  LowPowerMode: new state for the lowpower mode.
  *   This parameter can be one of the following values:
  *     @arg LIS302DL_LOWPOWERMODE_POWERDOWN: Power down mode
  *     @arg LIS302DL_LOWPOWERMODE_ACTIVE: Active mode  
  * @retval None
  */
void LIS302DL_LowpowerCmd(uint8_t LowPowerMode)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG1 register */
  LIS302DL_Read(&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
  
  /* Set new low power mode configuration */
  tmpreg &= (uint8_t)~LIS302DL_LOWPOWERMODE_ACTIVE;
  tmpreg |= LowPowerMode;
  
  /* Write value to MEMS CTRL_REG1 regsister */
  LIS302DL_Write(&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
}

/**
  * @brief  Data Rate command 
  * @param  DataRateValue: Data rate value
  *   This parameter can be one of the following values:
  *     @arg LIS302DL_DATARATE_100: 100 Hz output data rate 
  *     @arg LIS302DL_DATARATE_400: 400 Hz output data rate    
  * @retval None
  */
void LIS302DL_DataRateCmd(uint8_t DataRateValue)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG1 register */
  LIS302DL_Read(&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
  
  /* Set new Data rate configuration */
  tmpreg &= (uint8_t)~LIS302DL_DATARATE_400;
  tmpreg |= DataRateValue;
  
  /* Write value to MEMS CTRL_REG1 regsister */
  LIS302DL_Write(&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
}

/**
  * @brief  Change the Full Scale of LIS302DL
  * @param  FS_value: new full scale value. 
  *   This parameter can be one of the following values:
  *     @arg LIS302DL_FULLSCALE_2_3: +-2.3g
  *     @arg LIS302DL_FULLSCALE_9_2: +-9.2g   
  * @retval None
  */
void LIS302DL_FullScaleCmd(uint8_t FS_value)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG1 register */
  LIS302DL_Read(&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
  
  /* Set new full scale configuration */
  tmpreg &= (uint8_t)~LIS302DL_FULLSCALE_9_2;
  tmpreg |= FS_value;
  
  /* Write value to MEMS CTRL_REG1 regsister */
  LIS302DL_Write(&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
}

/**
  * @brief  Reboot memory content of LIS302DL
  * @param  None
  * @retval None
  */
void LIS302DL_RebootCmd(void)
{
  uint8_t tmpreg;
  /* Read CTRL_REG2 register */
  LIS302DL_Read(&tmpreg, LIS302DL_CTRL_REG2_ADDR, 1);
  
  /* Enable or Disable the reboot memory */
  tmpreg |= LIS302DL_BOOT_REBOOTMEMORY;
  
  /* Write value to MEMS CTRL_REG2 regsister */
  LIS302DL_Write(&tmpreg, LIS302DL_CTRL_REG2_ADDR, 1);
}

/**
  * @brief  Writes one byte to the LIS302DL.
  * @param  pBuffer : pointer to the buffer  containing the data to be written to the LIS302DL.
  * @param  WriteAddr : LIS302DL's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  * @retval None
  */
void LIS302DL_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit: 
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  LIS302DL_CS_LOW();
  
  /* Send the Address of the indexed register */
  HAL_SPI_Transmit(&hspi1,&WriteAddr,1,1);
  //LIS302DL_SendByte(WriteAddr);
  /* Send the data that will be written into the device (MSB First) */

  HAL_SPI_Transmit(&hspi1,pBuffer,NumByteToWrite,1);
  //  while(NumByteToWrite >= 0x01)
//  {
//    LIS302DL_SendByte(*pBuffer);
//    NumByteToWrite--;
//    pBuffer++;
//  }
  
  /* Set chip select High at the end of the transmission */ 
  LIS302DL_CS_HIGH();

}

/**
  * @brief  Reads a block of data from the LIS302DL.
  * @param  pBuffer : pointer to the buffer that receives the data read from the LIS302DL.
  * @param  ReadAddr : LIS302DL's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the LIS302DL.
  * @retval None
  */
void LIS302DL_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  LIS302DL_CS_LOW();
  
  /* Send the Address of the indexed register */
  //LIS302DL_SendByte(ReadAddr);

  HAL_SPI_Transmit(&hspi1,&ReadAddr,1,1);
  /* Receive the data that will be read from the device (MSB First) */
  HAL_SPI_Receive(&hspi1,pBuffer,NumByteToRead,1);

//  while(NumByteToRead > 0x00)
//  {
//    /* Send dummy byte (0x00) to generate the SPI clock to LIS302DL (Slave device) */
//    *pBuffer = LIS302DL_SendByte(DUMMY_BYTE);
//    NumByteToRead--;
//    pBuffer++;
//  }
  
  /* Set chip select High at the end of the transmission */ 
  LIS302DL_CS_HIGH();
}

/**
  * @brief  Read LIS302DL output register, and calculate the acceleration 
  *         ACC[mg]=SENSITIVITY* (out_h*256+out_l)/16 (12 bit rappresentation)
  * @param  s16 buffer to store data
  * @retval None
  */
void LIS302DL_ReadACC(int32_t* out)
{
  uint8_t buffer[6];
  uint8_t crtl, i = 0x00;
   
  LIS302DL_Read(&crtl, LIS302DL_CTRL_REG1_ADDR, 1);  
  LIS302DL_Read(buffer, LIS302DL_OUT_X_ADDR, 6);
  
  switch(crtl & 0x20) 
    {
    /* FS bit = 0 ==> Sensitivity typical value = 18milligals/digit*/ 
    case 0x00:
      for(i=0; i<0x03; i++)
      {
        *out =(int32_t)(LIS302DL_SENSITIVITY_2_3G *  (int8_t)buffer[2*i]);
        out++;
      }
      break;
    /* FS bit = 1 ==> Sensitivity typical value = 72milligals/digit*/ 
    case 0x20:
      for(i=0; i<0x03; i++)
      {
        *out =(int32_t)(LIS302DL_SENSITIVITY_9_2G * (int8_t)buffer[2*i]);
        out++;
      }         
      break;
    default:
      break;
    }
 }



#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LIS302DL_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
  while (1)
  {   
  }
}
#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */


  

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
