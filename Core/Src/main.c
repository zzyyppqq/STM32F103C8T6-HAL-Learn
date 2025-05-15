/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <memory.h>
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

uint8_t Buffer[1];
uint8_t sendBuff[] = "USART test by DMA\r\n";

uint8_t recvBuff[BUFFER_SIZE];  //接收数据缓存数组
volatile uint8_t recvLength = 0;  //接收一帧数据的长度
volatile uint8_t recvDndFlag = 0; //一帧数据接收完成标志

// ADC转换值
__IO uint32_t ADC_ConvertedValue;
// 用于保存转换计算后的电压值
float ADC_Vol;

#define ADDR_24LCxx_Write 0xA0
#define ADDR_24LCxx_Read 0xA1
#define BufferSize 256
uint8_t WriteBuffer[BufferSize] = {0};
uint8_t ReadBuffer[BufferSize] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
    HAL_UART_Receive_IT(&huart1, (uint8_t *)Buffer, 1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能IDLE中断
    HAL_UART_Receive_DMA(&huart1, recvBuff, BUFFER_SIZE);
    // HAL_UART_Receive_DMA(&huart1, (uint8_t *)Buffer, 1);
  /**
   * 添加定时器启动函数
   * 现在进入 main 函数并在 while 循环前加入开启定时器函数 HAL_TIM_Base_Start_IT()，这里所传入的 htim4 就是刚刚定时器初始化后的结构体。
   */
    HAL_TIM_Base_Start_IT(&htim4);

    // ADC初始化后，添加ADC中断开启函数，这样在第一次接收到数据的时候才会触发中断
    HAL_ADCEx_Calibration_Start(&hadc1);    //AD校准
    HAL_ADC_Start_IT(&hadc1); //开启ADC中断转换


    // STM32F103C8T6无独立EEPROM，可用Flash模拟
    printf("\r\n***************I2C Example*******************************\r\n");
    uint32_t i;
    uint8_t j;
    // 0x00 ~ 0xFF
    for(i = 0; i < 256; i++)
    {
        WriteBuffer[i] = i;    /* WriteBuffer init */
        printf("0x%02X ", WriteBuffer[i]);
        if(i % 16 == 15)
        {
            printf("\n\r");
        }
    }
    /* wrinte date to EEPROM */
    for (j = 0; j < 32; j++)
    {
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, ADDR_24LCxx_Write, 8*j, I2C_MEMADD_SIZE_8BIT, WriteBuffer+8*j, 8, 100);
        if(status == HAL_OK)
        {
            printf("\r\n EEPROM 24C02 Write Test OK \r\n");
        }
        else
        {
            printf("\r\n EEPROM 24C02 Write Test False [status: 0x%02X]  \r\n", status);
        }
        HAL_Delay(5);
    }
    /* read date from EEPROM */
    HAL_I2C_Mem_Read(&hi2c2, ADDR_24LCxx_Read, 0, I2C_MEMADD_SIZE_8BIT, ReadBuffer, BufferSize, 1000);
    for(i = 0; i < 256; i++)
    {
        printf("0x%02X  ",ReadBuffer[i]);
        if(i%16 == 15)
        {
            printf("\n\r");
        }
    }

    if(memcmp(WriteBuffer,ReadBuffer,BufferSize) == 0 ) /* check date */
    {
        printf("\r\n EEPROM 24C02 Read Test OK\r\n");
    }
    else
    {
        printf("\r\n EEPROM 24C02 Read Test False\r\n");
    }

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
//  MX_FREERTOS_Init();

  /* Start scheduler */
//  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
      // 注意：如果不开启串口中断，则程序只能发送一次数据,程序不能判断DMA传输是否完成，USART一直处于busy状态。
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)sendBuff, sizeof(sendBuff));
      HAL_Delay(1000);

      // 添加电压值转换
      ADC_Vol =(float) ADC_ConvertedValue/4096*3.3; // 读取转换的AD倿
      printf("The current AD ADC_ConvertedValue = 0x%04X", ADC_ConvertedValue);
      printf("The current AD ADC_Vol = %f V \r\n",ADC_Vol); //实际电压倿
      HAL_Delay(1000);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)Buffer, 1, 0xffff);
        HAL_UART_Receive_IT(&huart1, (uint8_t *)Buffer, 1);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // 在中断回调函数中进行读取数据，将数据存放在变量 ADC_ConvertedValue 中。
    ADC_ConvertedValue = HAL_ADC_GetValue(hadc);
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
    static uint32_t time = 0;
    if(htim->Instance == TIM4)  // 定时器4基地址
    {
        // 自定义应用程序
        time++;           // 每1ms进来1次
        if(time == 1000)  // 每1秒LED灯翻转一次
        {
//            HAL_GPIO_TogglePin(LED_WHITE_GPIO_Port,LED_WHITE_Pin);
//            HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin);
            time = 0;
        }
    }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
