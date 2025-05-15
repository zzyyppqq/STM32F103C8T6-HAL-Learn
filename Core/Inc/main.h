/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
HAL_StatusTypeDef SPI_Transmit(uint8_t* send_buf, uint16_t size);

/**
 * 从 SPI Flash 接收数据的函数
 * @brief   SPI接收指定长度的数据
 * @param   buf  —— 接收数据缓冲区首地址
 * @param   size —— 要接收数据的字节数
 * @retval  成功返回HAL_OK
 */
HAL_StatusTypeDef SPI_Receive(uint8_t* recv_buf, uint16_t size);

/**
 * 发送数据的同时读取数据的函数
 * @brief   SPI在发送数据的同时接收指定长度的数据
 * @param   send_buf  —— 接收数据缓冲区首地址
 * @param   recv_buf  —— 接收数据缓冲区首地址
 * @param   size —— 要发送/接收数据的字节数
 * @retval  成功返回HAL_OK
 */
HAL_StatusTypeDef SPI_TransmitReceive(uint8_t* send_buf, uint8_t* recv_buf, uint16_t size);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY2_Pin GPIO_PIN_13
#define KEY2_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOA
#define LED_WHITE_Pin GPIO_PIN_2
#define LED_WHITE_GPIO_Port GPIOA
#define W25Q64_CHIP_SELECT_Pin GPIO_PIN_4
#define W25Q64_CHIP_SELECT_GPIO_Port GPIOA
#define KEY3_Pin GPIO_PIN_0
#define KEY3_GPIO_Port GPIOB
#define KEY3_EXTI_IRQn EXTI0_IRQn

/* USER CODE BEGIN Private defines */
#define BUFFER_SIZE 256
extern uint8_t recvBuff[BUFFER_SIZE];  //接收数据缓存
extern volatile uint8_t recvLength;  //接收一帧数据的长度
extern volatile uint8_t recvDndFlag; //一帧数据接收完成标志

#define ManufactDeviceID_CMD	0x90
#define READ_STATU_REGISTER_1   0x05
#define READ_STATU_REGISTER_2   0x35
#define READ_DATA_CMD	        0x03
#define WRITE_ENABLE_CMD	    0x06
#define WRITE_DISABLE_CMD	    0x04
#define SECTOR_ERASE_CMD	    0x20
#define CHIP_ERASE_CMD	        0xc7
#define PAGE_PROGRAM_CMD        0x02

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
