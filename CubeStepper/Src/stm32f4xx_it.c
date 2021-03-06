/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4_discovery_lis302dl.h"
#include "stm32f4_discovery_l3gd20.h"
#include "main.h"
#define LIS302DL_OUT_X_ADDR                  0x29

/* USER CODE BEGIN 0 */
uint8_t Counter  = 0;
__IO uint32_t TimingDelay = 0;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */

	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1)
	{
	}
	/* USER CODE BEGIN HardFault_IRQn 1 */

	/* USER CODE END HardFault_IRQn 1 */
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1)
	{
	}
	/* USER CODE BEGIN MemoryManagement_IRQn 1 */

	/* USER CODE END MemoryManagement_IRQn 1 */
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1)
	{
	}
	/* USER CODE BEGIN BusFault_IRQn 1 */

	/* USER CODE END BusFault_IRQn 1 */
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1)
	{
	}
	/* USER CODE BEGIN UsageFault_IRQn 1 */

	/* USER CODE END UsageFault_IRQn 1 */
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */void Delay(__IO uint32_t nTime)
 {
	 TimingDelay = nTime;

	 while(TimingDelay != 0);
 }

 /**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
 void TimingDelay_Decrement(void)
 {
	 if (TimingDelay != 0x00)
	 {
		 TimingDelay--;
	 }
 }
 void SysTick_Handler(void)
 {
	 /* USER CODE BEGIN SysTick_IRQn 0 */

	 /* USER CODE END SysTick_IRQn 0 */
	 HAL_IncTick();
	 HAL_SYSTICK_IRQHandler();
	 /* USER CODE BEGIN SysTick_IRQn 1 */
//	 uint8_t temp1, temp2 = 0;
//	 TM_L3GD20_t L3DG20_Data;
//
//	 if (TimingDelay != 0x00)
//	 {
//		 TimingDelay_Decrement();
//	 }
//	 else
//	 {
//		 Counter ++;
//		 if (Counter == 10)
//		 {
//			 Buffer[2] = 0;
//			 LIS302DL_Read(Buffer, LIS302DL_OUT_X_ADDR, 6);
//
//			 //			 if (((int8_t)Buffer[2] > -5) && ((int8_t)Buffer[2] < 5))
//			 //				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
//			 //			 else
//			 //				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
//			 //
//			 //			 if((int8_t)Buffer[2] > 5)
//			 //				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
//			 //			 else
//			 //				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
//			 //
//			 //			 Stepper_Set_Speed(Buffer[2]);
//
//			 /* Remove the offsets values from data */
//			 Buffer[0] -= XOffset;
//			 Buffer[2] -= YOffset;
//
//			 /* Update autoreload and capture compare registers value*/
//			 temp1 = ABS((int8_t)(Buffer[0]));
//			 temp2 = ABS((int8_t)(Buffer[2]));
//			 TempAcceleration = MAX(temp1, temp2);
//
//
//			 TM_L3GD20_Read(&L3DG20_Data);
//
//
//			 if(TempAcceleration != 0)
//			 {
//
//				 if ((int8_t)Buffer[2] > 7)
//				 {
//					 //spre(LED5);
//
//					// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
//					 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
//					 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
//
//
//				 }
//				 if ((int8_t)Buffer[2] < -7)
//				 {
//					 //spre(LED4);
//
//					// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
//					 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
//					 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
//				 }
//			 }
////			 else
////			 {
////				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
////			 }
//			 Counter = 0x00;
//			 Stepper_Set_Speed(Buffer[2]);
//		 }
//
//	 }

	 /* USER CODE END SysTick_IRQn 1 */
 }
	 /******************************************************************************/
	 /* STM32F4xx Peripheral Interrupt Handlers                                    */
	 /* Add here the Interrupt Handlers for the used peripherals.                  */
	 /* For the available peripheral interrupt handler names,                      */
	 /* please refer to the startup file (startup_stm32f4xx.s).                    */
	 /******************************************************************************/

	 /* USER CODE BEGIN 1 */

	 void TIM4_IRQHandler(void)
	 {
		 while(1);
	 }


	 /* USER CODE END 1 */
	 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
