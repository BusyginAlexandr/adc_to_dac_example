/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "adc.h"
#include "string.h"

#define TIMOUT_MS 0x64   /// 100 * 10 ms = 1 s averaging time
#define VOLTAGE   0.8056 /// (3.3 V * 1000 mV) / 4096 = 0.8056 mV/degree
#define UNIT_MV   4      /// voltage unit in mV and ASCII code
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
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
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
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
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
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
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
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
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
static uint32_t averageValue   = 0; /// Variable in which the average value is stored
static uint32_t cntSourceValue = 0; /// The number of instantaneous values of the analog signal for a given interval

void DMA1_Channel1_IRQHandler(void)
{
	LL_DMA_ClearFlag_TC1(DMA1);
	
	cntSourceValue++;             /// $cntSourceValue +1 sampling from ADC
	averageValue += sourceSignal; /// accumulate samples from the ADC
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */

void TIM1_UP_TIM16_IRQHandler(void)
{	
	static uint16_t timeoutMS = 0;  /// The time over which the instantaneous values of the analog signal are averaged multiplied by 10(t*10) 
	static uint32_t sampling  = 0;  /// Instant ADC value averaged over 1 s
	static uint8_t  cntChar   = 0;  /// Count ASCII symbol
	static char  measurValue[4] = {' ',' ',' ',' '}; /// The ADC instantaneous value averaged over 1 s in ASCII code
	uint8_t unit[4] = {' ','m', 'V', '\r'};             /// Voltage unit in mV (ASCII)
	uint8_t i = 0; 
	
	LL_TIM_ClearFlag_UPDATE(TIM1);
	LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	/// Write to the DAC averaged value every 10 ms
  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1,averageValue/cntSourceValue);
	sampling += averageValue/cntSourceValue;      /// accumulate values for 1 s 
	averageValue   = 0;
	cntSourceValue = 0;
	
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  /// 1 s
  if(timeoutMS == TIMOUT_MS)
	{
	   timeoutMS = 0;
	   sampling = (sampling / TIMOUT_MS)*VOLTAGE; /// Average value for 1 s
		 cntChar = itoa(sampling, measurValue);     /// Convert number to ascii
     // Write to usart average value for 1 s in mV
		 for(i = 0; i < cntChar; i++)
		 {
			  while (!LL_USART_IsActiveFlag_TC( USART1));
			  LL_USART_TransmitData8(USART1, measurValue[i]);   
		 }
     // Write to uart " mV\r"
		 for(i = 0; i < UNIT_MV; i++)
		 {
			  while (!LL_USART_IsActiveFlag_TC( USART1));
			  LL_USART_TransmitData8(USART1, unit[i]);   
		 }
		 timeoutMS = 0;
	}
	else
		timeoutMS++;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
