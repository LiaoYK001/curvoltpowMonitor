#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include "i2c.h" /* 使用 CubeMX 生成的 i2c.h，以获得 extern I2C_HandleTypeDef hi2c2 */
#include <stdint.h>

/* 原来你使用的 8-bit 地址 0x90，转换为 7-bit 地址用于 HAL */
#define DEV_ADDR_8BIT 0x90U
#define DEV_ADDR_7BIT 0x48

/* 寄存器定义（保留原定义） */
#define NOOP 0x00
#define DEVID 0x01
#define SYNC 0x02
#define CONFIG 0x03
#define GAIN 0x04
#define TRIGGER 0x05
#define STATUS 0x06
#define DAC_DATA 0x08

/* I2C 超时（ms） */
#ifndef I2C_TIMEOUT
#define I2C_TIMEOUT 100U
#endif

/* 高层读写接口（与原函数名一致） */
void I2C_WriteByte(uint8_t reg_addr, uint16_t data);
uint16_t I2C_ReadByte(uint8_t reg_addr);

#endif /* __BSP_IIC_H */
