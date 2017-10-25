/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include <stdbool.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef USBD_Device;
pFunction JumpToApplication;
uint32_t JumpAddress;
RTC_HandleTypeDef RtcHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void rtcSetup();
void blinkLED(int times, int duration);
void pulseLED();
void jumpToApp();
void boot_jump();
uint32_t dfuActive(USBD_HandleTypeDef *pdev);
void disconnectUsb();
uint32_t userAppExists();
uint32_t checkAndClearBootFlag();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	  /* STM32F446xx HAL library initialization */
	  HAL_Init();

	  /* Configure the System clock to have a frequency of 180 MHz */
	  SystemClock_Config();

	  MX_GPIO_Init();

	  pulseLED();

	  HAL_Delay(250);

	  rtcSetup();
	  uint32_t bootloaderFlag = checkAndClearBootFlag();

	  if(bootloaderFlag != BOOTLOADER_FLAG_SKIP) {

		  /* Enter DFU mode to allow user programming application */
		  USBD_Init(&USBD_Device, &DFU_Desc, 0);						/* Init Device Library */
		  USBD_RegisterClass(&USBD_Device, USBD_DFU_CLASS);				/* Add Supported Class */
		  USBD_DFU_RegisterMedia(&USBD_Device, &USBD_DFU_Flash_fops);	/* Add DFU Media interface */
		  USBD_Start(&USBD_Device);										/* Start Device Process */

		  bool buttonPressed = !HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);

		  int loops = DFU_WAIT_LOOPS;
		  int dfuDone = 0;
		  uint32_t appExists = userAppExists();

		  while (loops > 0 || buttonPressed || !appExists || bootloaderFlag == BOOTLOADER_FLAG_DFU) {
			  pulseLED();

			  if(dfuActive(&USBD_Device) == 0) {
				  //no DFU transfer at present. Count down to normal boot.
				  loops--;

				  //if transfer has previously taken place, skip count down and boot.
				  if(dfuDone > 2) {
					  break;
				  }
			  } else {
				  dfuDone++;
			  }
		  }
		  disconnectUsb();
	  }
	  jumpToApp();
}

uint32_t userAppExists() {
	if (((*(__IO uint32_t *) USBD_DFU_APP_DEFAULT_ADD) & 0xFFFD0FFF) ==	0x20000000)	{
		return 1;
	} else {
		return 0;
	}
}

void rtcSetup() {
	RtcHandle.Instance = RTC;

	if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

uint32_t dfuActive(USBD_HandleTypeDef *pdev) {
	USBD_DFU_HandleTypeDef   *hdfu;
	hdfu = (USBD_DFU_HandleTypeDef*) pdev->pClassData;
	if(hdfu->dev_state == DFU_STATE_IDLE) {
		return 0;
	} else {
		return 1;
	}
}

void disconnectUsb() {
	GPIO_InitTypeDef GPIO_InitStruct;

	/*Configure GPIO pin : GPIO_PIN_12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(50);
}

void blinkLED(int times, int duration) {
	for(int i = 0; i < times; i++) {
		HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		HAL_Delay(duration);
		HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(duration);
	}
}

void pulseLED() {
	int duration = 1600;	//approx 500ms at 180MHz (measured)
	for(int j = 0; j < duration; j++) {
		for(int k = 0; k < duration-j; k++) {
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			asm("NOP");
		}
		for(int k = duration; k > duration-j; k--) {
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			asm("NOP");
		}
	}

	for(int j = 0; j < duration; j++) {
		for(int k = 0; k < duration-j; k++) {
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			asm("NOP");
		}
		for(int k = duration; k > duration-j; k--) {
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			asm("NOP");
		}
	}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 6;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 96;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void jumpToApp(void) {

	//disable global interrupt
	//__disable_irq();

	// Switch off core clock before switching vector table
	SysTick->CTRL = 0 ;

	//disable all peripheral interrupts
	HAL_NVIC_DisableIRQ(SysTick_IRQn);
	HAL_NVIC_DisableIRQ(WWDG_IRQn);
	HAL_NVIC_DisableIRQ(PVD_IRQn);
	HAL_NVIC_DisableIRQ(TAMP_STAMP_IRQn);
	HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
	HAL_NVIC_DisableIRQ(FLASH_IRQn);
	HAL_NVIC_DisableIRQ(RCC_IRQn);
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Stream2_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Stream4_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
	//HAL_NVIC_DisableIRQ(ADC_IRQn);
	//HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
	//HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
	//HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
	//HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
	//HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
	HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
	HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn);
	HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
	//HAL_NVIC_DisableIRQ(TIM2_IRQn);
	//HAL_NVIC_DisableIRQ(TIM3_IRQn);
	//HAL_NVIC_DisableIRQ(TIM4_IRQn);
	//HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
	//HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
	//HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
	//HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
	//HAL_NVIC_DisableIRQ(SPI1_IRQn);
	//HAL_NVIC_DisableIRQ(SPI2_IRQn);
	//HAL_NVIC_DisableIRQ(USART1_IRQn);
	//HAL_NVIC_DisableIRQ(USART2_IRQn);
	//HAL_NVIC_DisableIRQ(USART3_IRQn);
	//HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
	HAL_NVIC_DisableIRQ(OTG_FS_WKUP_IRQn);
	//HAL_NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
	//HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
	//HAL_NVIC_DisableIRQ(TIM8_TRG_COM_TIM14_IRQn);
	//HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Stream7_IRQn);
	//HAL_NVIC_DisableIRQ(FMC_IRQn);
	//HAL_NVIC_DisableIRQ(SDIO_IRQn);
	//HAL_NVIC_DisableIRQ(TIM5_IRQn);
	//HAL_NVIC_DisableIRQ(SPI3_IRQn);
	//HAL_NVIC_DisableIRQ(UART4_IRQn);
	//HAL_NVIC_DisableIRQ(UART5_IRQn);
	//HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
	//HAL_NVIC_DisableIRQ(TIM7_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream4_IRQn);
	//HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
	//HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
	//HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
	//HAL_NVIC_DisableIRQ(CAN2_SCE_IRQn);
	HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream5_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream6_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream7_IRQn);
	//HAL_NVIC_DisableIRQ(USART6_IRQn);
	//HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
	//HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);
	HAL_NVIC_DisableIRQ(OTG_HS_EP1_OUT_IRQn);
	HAL_NVIC_DisableIRQ(OTG_HS_EP1_IN_IRQn);
	HAL_NVIC_DisableIRQ(OTG_HS_WKUP_IRQn);
	HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
	//HAL_NVIC_DisableIRQ(DCMI_IRQn);
	HAL_NVIC_DisableIRQ(FPU_IRQn);
	//HAL_NVIC_DisableIRQ(SPI4_IRQn);
	//HAL_NVIC_DisableIRQ(SAI1_IRQn);
	//HAL_NVIC_DisableIRQ(SAI2_IRQn);
	//HAL_NVIC_DisableIRQ(QUADSPI_IRQn);
	HAL_NVIC_DisableIRQ(CEC_IRQn);
	//HAL_NVIC_DisableIRQ(SPDIF_RX_IRQn);
	//HAL_NVIC_DisableIRQ(FMPI2C1_EV_IRQn);
	//HAL_NVIC_DisableIRQ(FMPI2C1_ER_IRQn);

	// Switch vector table
	SCB->VTOR = USBD_DFU_APP_DEFAULT_ADD ;

	//Jump to start address
	boot_jump() ;

}

void boot_jump() {
	/* Jump to user application */
	JumpAddress = *(__IO uint32_t *) (USBD_DFU_APP_DEFAULT_ADD + 4);
	JumpToApplication = (pFunction) JumpAddress;

	/* Initialize user application's Stack Pointer */
	__set_MSP((*(__IO uint32_t *) USBD_DFU_APP_DEFAULT_ADD));
	JumpToApplication();
}

uint32_t checkAndClearBootFlag() {

	uint32_t bootloaderFlag = HAL_RTCEx_BKUPRead(&RtcHandle, BOOTLOADER_FLAG_REGISTER);
	HAL_RTCEx_BKUPWrite(&RtcHandle, BOOTLOADER_FLAG_REGISTER, BOOTLOADER_FLAG_NONE);

	return bootloaderFlag;

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
	  blinkLED(1,125);
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
