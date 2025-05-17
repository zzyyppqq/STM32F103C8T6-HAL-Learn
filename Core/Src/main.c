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
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "wwdg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <memory.h>
#include "W25Q64.h"
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

// I2C
#define ADDR_24LCxx_Write 0xA0
#define ADDR_24LCxx_Read 0xA1
#define BufferSize 256
uint8_t WriteBuffer[BufferSize] = {0};
uint8_t ReadBuffer[BufferSize] = {0};

// SPI
uint16_t device_id;
uint8_t read_buf[10] = {0};
uint8_t write_buf[10] = {0};
int i;

// PWM
uint16_t indexWave[] = {
        1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4,
        4, 5, 5, 6, 7, 8, 9, 10, 11, 13,
        15, 17, 19, 22, 25, 28, 32, 36,
        41, 47, 53, 61, 69, 79, 89, 102,
        116, 131, 149, 170, 193, 219, 250,
        284, 323, 367, 417, 474, 539, 613,
        697, 792, 901, 1024, 1024, 901, 792,
        697, 613, 539, 474, 417, 367, 323,
        284, 250, 219, 193, 170, 149, 131,
        116, 102, 89, 79, 69, 61, 53, 47, 41,
        36, 32, 28, 25, 22, 19, 17, 15, 13,
        11, 10, 9, 8, 7, 6, 5, 5, 4, 4, 3, 3,
        2, 2, 2, 2, 1, 1, 1, 1
};

uint16_t POINT_NUM = sizeof(indexWave)/sizeof(indexWave[0]);

// RTC
RTC_DateTypeDef GetData;  //获取日期结构体
RTC_TimeTypeDef GetTime;   //获取时间结构体

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void I2C_Example() {
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

}

void SPI_Example() {
    printf("\r\n***************SPI Example*******************************\r\n");

    device_id = W25QXX_ReadID();
    printf("W25Q64 Device ID is 0x%04x\r\n", device_id);

    /* 为了验证，首先读取要写入地址处的数据 */
    printf("-------- read data before write -----------\r\n");
    W25QXX_Read(read_buf, 0, 10);

    for(i = 0; i < 10; i++)
    {
        printf("[0x%08x]:0x%02x\r\n", i, *(read_buf+i));
    }

    /* 擦除该扇区 */
    printf("-------- erase sector 0 -----------\r\n");
    W25QXX_Erase_Sector(0);

    /* 再次读数据 */
    printf("-------- read data after erase -----------\r\n");
    W25QXX_Read(read_buf, 0, 10);
    for(i = 0; i < 10; i++)
    {
        printf("[0x%08x]:0x%02x\r\n", i, *(read_buf+i));
    }

    /* 写数据 */
    printf("-------- write data -----------\r\n");
    for(i = 0; i < 10; i++)
    {
        write_buf[i] = i;
    }
    W25QXX_Page_Program(write_buf, 0, 10);

    /* 再次读数据 */
    printf("-------- read data after write -----------\r\n");
    W25QXX_Read(read_buf, 0, 10);
    for(i = 0; i < 10; i++)
    {
        printf("[0x%08x]:0x%02x\r\n", i, *(read_buf+i));
    }
}

// CRC
#define BUFFER_SIZE    114
static const uint32_t dataBuffer[BUFFER_SIZE] =
        {
                0x00001021, 0x20423063, 0x408450a5, 0x60c670e7, 0x9129a14a, 0xb16bc18c,
                0xd1ade1ce, 0xf1ef1231, 0x32732252, 0x52b54294, 0x72f762d6, 0x93398318,
                0xa35ad3bd, 0xc39cf3ff, 0xe3de2462, 0x34430420, 0x64e674c7, 0x44a45485,
                0xa56ab54b, 0x85289509, 0xf5cfc5ac, 0xd58d3653, 0x26721611, 0x063076d7,
                0x569546b4, 0xb75ba77a, 0x97198738, 0xf7dfe7fe, 0xc7bc48c4, 0x58e56886,
                0x78a70840, 0x18612802, 0xc9ccd9ed, 0xe98ef9af, 0x89489969, 0xa90ab92b,
                0x4ad47ab7, 0x6a961a71, 0x0a503a33, 0x2a12dbfd, 0xfbbfeb9e, 0x9b798b58,
                0xbb3bab1a, 0x6ca67c87, 0x5cc52c22, 0x3c030c60, 0x1c41edae, 0xfd8fcdec,
                0xad2abd0b, 0x8d689d49, 0x7e976eb6, 0x5ed54ef4, 0x2e321e51, 0x0e70ff9f,
                0xefbedfdd, 0xcffcbf1b, 0x9f598f78, 0x918881a9, 0xb1caa1eb, 0xd10cc12d,
                0xe16f1080, 0x00a130c2, 0x20e35004, 0x40257046, 0x83b99398, 0xa3fbb3da,
                0xc33dd31c, 0xe37ff35e, 0x129022f3, 0x32d24235, 0x52146277, 0x7256b5ea,
                0x95a88589, 0xf56ee54f, 0xd52cc50d, 0x34e224c3, 0x04817466, 0x64475424,
                0x4405a7db, 0xb7fa8799, 0xe75ff77e, 0xc71dd73c, 0x26d336f2, 0x069116b0,
                0x76764615, 0x5634d94c, 0xc96df90e, 0xe92f99c8, 0xb98aa9ab, 0x58444865,
                0x78066827, 0x18c008e1, 0x28a3cb7d, 0xdb5ceb3f, 0xfb1e8bf9, 0x9bd8abbb,
                0x4a755a54, 0x6a377a16, 0x0af11ad0, 0x2ab33a92, 0xed0fdd6c, 0xcd4dbdaa,
                0xad8b9de8, 0x8dc97c26, 0x5c644c45, 0x3ca22c83, 0x1ce00cc1, 0xef1fff3e,
                0xdf7caf9b, 0xbfba8fd9, 0x9ff86e17, 0x7e364e55, 0x2e933eb2, 0x0ed11ef0
        };

/* Expected CRC Value */
uint32_t uwExpectedCRCValue = 0x379E9F06;

void CRC_Example() {
    __IO uint32_t CRCValue = 0;
    CRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)dataBuffer, BUFFER_SIZE);
    printf("\r\n32-bit CRC:0x%X\n", CRCValue);
    if(CRCValue != uwExpectedCRCValue)
    {
        printf("\n\r CRC wrong value\n\r");
    }
    else
    {
        printf("\n\r CRC right value\n\r");
    }
}

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
  MX_SPI1_Init();
  MX_IWDG_Init();
  MX_WWDG_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_CRC_Init();
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

    // PWM启动
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    // I2C_Example();
    // SPI_Example();
    CRC_Example();

//    printf("\n\r***** IWDG Test Start *****\n\r");
//
//    printf("\n\r***** WWDG Test Start *****\n\r");

    // RTC设置初始时间

// 设置时间（23:19:45）
    GetTime.Hours = 23;
    GetTime.Minutes = 24;
    GetTime.Seconds = 45;
    //GetTime.TimeFormat = RTC_HOURFORMAT_24;  // 24小时制
    HAL_RTC_SetTime(&hrtc, &GetTime, RTC_FORMAT_BIN);

// 设置日期（2025年5月16日，星期五）
    GetData.WeekDay = RTC_WEEKDAY_FRIDAY;  // 星期需根据实际日期计算
    GetData.Month = RTC_MONTH_MAY;
    GetData.Date = 16;
    GetData.Year = 25;  // 年份为2025的后两位
    HAL_RTC_SetDate(&hrtc, &GetData, RTC_FORMAT_BIN);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
//      // DMA 注意：如果不开启串口中断，则程序只能发送一次数据,程序不能判断DMA传输是否完成，USART一直处于busy状态。
//      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)sendBuff, sizeof(sendBuff));
//      HAL_Delay(1000);
//
//      // ADC 添加电压值转换
//      ADC_Vol =(float) ADC_ConvertedValue/4096*3.3; // 读取转换的AD倿
//      printf("The current AD ADC_ConvertedValue = 0x%04X", ADC_ConvertedValue);
//      printf("The current AD ADC_Vol = %f V \r\n",ADC_Vol); //实际电压倿
//      HAL_Delay(1000);

      // IWDG
//      printf("\n\r Refreshes the IWDG !!!\n\r");
//      // 因为设置超时溢出为 1 秒，所以这里每隔 800 毫秒喂狗一次 HAL_IWDG_Refresh(&hiwdg);
//      HAL_IWDG_Refresh(&hiwdg);
//      HAL_Delay(800);

      // WWDG
//      printf("\n\r Running...\n\r");
//      HAL_Delay(1000);


      /* Get the RTC current Time */
      HAL_RTC_GetTime(&hrtc, &GetTime, RTC_FORMAT_BIN);
      /* Get the RTC current Date */
      HAL_RTC_GetDate(&hrtc, &GetData, RTC_FORMAT_BIN);

      /* Display date Format : yy/mm/dd */
      printf("%02d/%02d/%02d\r\n",2000 + GetData.Year, GetData.Month, GetData.Date);
      /* Display time Format : hh:mm:ss */
      printf("%02d:%02d:%02d\r\n",GetTime.Hours, GetTime.Minutes, GetTime.Seconds);

      printf("\r\n");

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
    HAL_WWDG_Refresh(hwwdg);
}

//  封装SPI Flash(W25Q64)的命令和底层函数
/**
 * 向 SPI Flash 发送数据的函数
 * @brief    SPI发送指定长度的数据
 * @param    buf  —— 发送数据缓冲区首地址
 * @param    size —— 要发送数据的字节数
 * @retval   成功返回HAL_OK
 */
HAL_StatusTypeDef SPI_Transmit(uint8_t* send_buf, uint16_t size)
{
    return HAL_SPI_Transmit(&hspi1, send_buf, size, 100);
}

/**
 * 从 SPI Flash 接收数据的函数
 * @brief   SPI接收指定长度的数据
 * @param   buf  —— 接收数据缓冲区首地址
 * @param   size —— 要接收数据的字节数
 * @retval  成功返回HAL_OK
 */
HAL_StatusTypeDef SPI_Receive(uint8_t* recv_buf, uint16_t size)
{
    return HAL_SPI_Receive(&hspi1, recv_buf, size, 100);
}

/**
 * 发送数据的同时读取数据的函数
 * @brief   SPI在发送数据的同时接收指定长度的数据
 * @param   send_buf  —— 接收数据缓冲区首地址
 * @param   recv_buf  —— 接收数据缓冲区首地址
 * @param   size —— 要发送/接收数据的字节数
 * @retval  成功返回HAL_OK
 */
HAL_StatusTypeDef SPI_TransmitReceive(uint8_t* send_buf, uint8_t* recv_buf, uint16_t size)
{
    return HAL_SPI_TransmitReceive(&hspi1, send_buf, recv_buf, size, 100);
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

    static uint8_t pwm_index = 1;        /* 用于PWM查表 */
    static uint8_t period_cnt = 0;       /* 用于计算周期数 */
    if (htim->Instance == TIM2) {

        period_cnt++;
        /* 若输出的周期数大于20，输出下一种脉冲宽的PWM波 */
        if(period_cnt >= 20)
        {
            /* 根据PWM表修改定时器的比较寄存器值 */
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, indexWave[pwm_index]);

            /* 标志PWM表的下一个元素 */
            pwm_index++;
            /* 若PWM脉冲表已经输出完成一遍，重置PWM查表标志 */
            if( pwm_index >=  POINT_NUM)
            {
                pwm_index=0;
            }
            /* 重置周期计数标志 */
            period_cnt=0;
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
