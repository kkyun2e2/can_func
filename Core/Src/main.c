/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef CAN_TXH;
CAN_RxHeaderTypeDef CAN_RXH;
CAN_FilterTypeDef filtername;
uint32_t Mailbox;
uint8_t CAN_TX_Data_8[8] = {0};
uint8_t CAN_RX_Data_8[8] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_RX_Header_defunc()
{
	 filtername.FilterActivation = CAN_FILTER_ENABLE; // filter on,off
	 filtername.FilterBank = 1; // filterbank initialize single can = 0~13, dual can = 0~27
	 filtername.FilterFIFOAssignment =CAN_FILTER_FIFO0; // fifo assgin 0 or 1
	 filtername.FilterIdHigh = 0x0000; //
	 filtername.FilterIdLow = 0x0000;
	 filtername.FilterMaskIdHigh = 0x0000;
	 filtername.FilterMaskIdLow = 0x0000;
	 filtername.FilterMode = CAN_FILTERMODE_IDMASK;  // filter mode -> mask or list
	 filtername.FilterScale = CAN_FILTERSCALE_16BIT; // filter scale
	 // filtername.SlaveStartFilterBank // only dual can

	 HAL_CAN_ConfigFilter(&hcan1,&filtername);
}
void CAN_TX_Header_defunc()
{
	 CAN_TXH.DLC=8;
	 CAN_TXH.IDE = CAN_ID_STD;
	 CAN_TXH.RTR = CAN_RTR_DATA;
	 CAN_TXH.StdId = 0x101;
	 CAN_TXH.TransmitGlobalTime = DISABLE;
}

void CAN_Error_Handler()
{
	if (HAL_CAN_ConfigFilter(&hcan1, &filtername) != HAL_OK)
	  {
	    /* Filter configuration Error */
	    Error_Handler();
	  }

	  /* Can Start */
	  if (HAL_CAN_Start(&hcan1) != HAL_OK)
	  {
	    /* Start Error */
	    Error_Handler();
	  }

	  /* Activate CAN RX notification */
	  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	  {
	    /* Notification Error */
	    Error_Handler();
	  }
}
void callbackSystick() //1ms count
	{
	   static int count=0;
	   count ++;
	   if(count == 10)  //10ms send can tx message
	   {
		   if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
		   	  {
		   		  HAL_CAN_AddTxMessage(&hcan1,&CAN_TXH,CAN_TX_Data_8,&Mailbox);
		   	  }
		   count = 0;
	   }
	}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &CAN_RXH, CAN_RX_Data_8) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  if(CAN_RXH.StdId == 0x00)
    {

    }
  if(CAN_RXH.StdId == 0x00)
    {

    }
  if(CAN_RXH.StdId == 0x00)
    {

    }
  if(CAN_RXH.StdId == 0x00)
    {

    }
  if(CAN_RXH.StdId == 0x00)
    {

    }
  if(CAN_RXH.StdId == 0x00)
    {

    }

}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan1);
  CAN_RX_Header_defunc();
  CAN_TX_Header_defunc();
  CAN_Error_Handler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
