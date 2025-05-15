//
// Created by zhangyipeng on 2025/5/15.
//

#ifndef STM32F103C8T6_FREERTOS_UART_LEARN_W25Q64_H
#define STM32F103C8T6_FREERTOS_UART_LEARN_W25Q64_H

#include "main.h"

/**
 * @brief   读取Flash内部的ID
 * @param   none
 * @retval  成功返回device_id
 */
uint16_t W25QXX_ReadID(void);

/**
 * @brief     读取W25QXX的状态寄存器，W25Q64一共有2个状态寄存器
 * @param     reg  —— 状态寄存器编号(1~2)
 * @retval    状态寄存器的值
 */
static uint8_t W25QXX_ReadSR(uint8_t reg);

/**
 * @brief	阻塞等待Flash处于空闲状态
 * @param   none
 * @retval  none
 */
static void W25QXX_Wait_Busy(void);

/**
 * @brief   读取SPI FLASH数据
 * @param   buffer      —— 数据存储区
 * @param   start_addr  —— 开始读取的地址(最大32bit)
 * @param   nbytes      —— 要读取的字节数(最大65535)
 * @retval  成功返回0，失败返回-1
 */
int W25QXX_Read(uint8_t* buffer, uint32_t start_addr, uint16_t nbytes);

/**
 * @brief    W25QXX写使能,将S1寄存器的WEL置位
 * @param    none
 * @retval
 */
void W25QXX_Write_Enable(void);

/**
 * @brief    W25QXX写禁止,将WEL清零
 * @param    none
 * @retval    none
 */
void W25QXX_Write_Disable(void);

/**
 * @brief    W25QXX擦除一个扇区
 * @param   sector_addr    —— 扇区地址 根据实际容量设置
 * @retval  none
 * @note    阻塞操作
 */
void W25QXX_Erase_Sector(uint32_t sector_addr);

/**
 * @brief    页写入操作
 * @param    dat —— 要写入的数据缓冲区首地址
 * @param    WriteAddr —— 要写入的地址
 * @param   byte_to_write —— 要写入的字节数（0-256）
 * @retval    none
 */
void W25QXX_Page_Program(uint8_t* dat, uint32_t WriteAddr, uint16_t nbytes);

#endif //STM32F103C8T6_FREERTOS_UART_LEARN_W25Q64_H
