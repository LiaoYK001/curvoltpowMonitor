#ifndef __DAC60501_H
#define __DAC60501_H

/* 适配 CubeMX 生成的 i2c.c / i2c.h，那里通常声明 extern I2C_HandleTypeDef hi2c2
 */
#include "i2c.h"
#include <stdint.h>

/* ----------------- 地址定义（A0 = AGND） -----------------
 * A0 = AGND -> 7-bit 地址 = 0x48 ，8-bit 写地址 = 0x90
 * 使用 HAL 时常以 7-bit 左移 1 的形式传入（即 (DEV_ADDR_7BIT << 1)）。
 */
#define DEV_ADDR_7BIT 0x48U
#define DEV_ADDR_8BIT (DEV_ADDR_7BIT << 1) /* = 0x90 */
#define DEV2_ADDR_7BIT 0x49U
#define DEV2_ADDR_8BIT (DEV2_ADDR_7BIT << 1) /* = 0x92 */
#define DAC_VOLTAGE_ADDR 0x48U               // 电压控制DAC地址
#define DAC_CURRENT_ADDR 0x49U               // 电流控制DAC地址
/* ----------------- 寄存器偏移定义 -----------------
 * 保留你原来的寄存器偏移（datasheet 里以字节为单位的偏移）
 */
#define NOOP 0x00
#define DEVID 0x01
#define SYNC 0x02
#define CONFIG 0x03
#define GAIN 0x04
#define TRIGGER 0x05
#define STATUS 0x06
#define DAC_DATA 0x08
/* I2C 超时时间（ms） */
#ifndef I2C_TIMEOUT
#define I2C_TIMEOUT 100U
#endif

/* GAIN 寄存器位（根据 datasheet）
 * - REF-DIV 在 bit8：1 = 将参考电压除以 2
 * - BUFF-GAIN 在 bit0：0 -> 增益 1；1 -> 增益 2
 */
#define GAIN_REF_DIV_BIT (1U << 8) // 用来表示“第 8 位”的掩码（bitmask）
#define GAIN_BUFF_GAIN_BIT (1U << 0)

/* STATUS 寄存器位：REF-ALARM (bit0) -> 1 表示参考报警（参考缓冲被关闭，输出 =
 * 0V） */
#define STATUS_REF_ALARM (1U << 0)
/* ----------------- DAC配置常量(外部声明)dac60501.c中 ----------------- */
extern const float DAC_VREF;     // DAC内部参考电压
extern const float DAC_VOUT_MAX; // DAC最大输出电压
extern const float MAX_VOLTAGE;  // 系统最大电压
extern const float MAX_CURRENT;  // 系统最大电流
/* ----------------- 导出的函数原型 ----------------- */
/* 基础 I2C 读写 - 增加设备地址参数 */
void I2C_WriteByte_Addr(uint8_t dev_addr, uint8_t reg_addr, uint16_t data);
uint16_t I2C_ReadByte_Addr(uint8_t dev_addr, uint8_t reg_addr);

/* 设备初始化（安全初始化：启用内部参考、设置 GAIN 为安全值等） */
/* 设备初始化 */
void DAC60501_Init_Addr(uint8_t dev_addr);
void DAC60501_Init_All(void); // 初始化所有DAC60501设备

/* 读设备 ID / STATUS */
uint16_t DAC60501_ReadDeviceId_Addr(uint8_t dev_addr);
uint16_t DAC60501_ReadStatus_Addr(uint8_t dev_addr);

/* 设置 GAIN：ref_div = 0/1, buff_gain = 0/1
 * 返回：0 = OK，<0 = 错误（I2C 通信失败等）
 */
int DAC60501_SetGain_Addr(uint8_t dev_addr, uint8_t ref_div, uint8_t buff_gain);

/* 设置输出电压（安全方式配置 REF/GAIN 的前提下）
 * vout: 目标电压 (V)
 * vref: 参考电压 (V) - 你的内部参考 = 2.5V
 * 返回：0 = OK，-1 = 参数错误，-2 = ref alarm，-3 = I2C 通信错误
 *
 * 计算说明见实现：12-bit (0..4095)，寄存器左对齐（写入值 = code << 4）
 */
/* 设置输出电压 */
int DAC60501_SetVoltage_Addr(uint8_t dev_addr, float vout, float vref);
int DAC60501_RefAlarm_Addr(uint8_t dev_addr);

/* 高层应用接口 */
int DAC60501_SetVoltageOutput(float voltage,float current_ma); // 设置电压输出(通过0x48)
int DAC60501_SetCurrentOutput(float current); // 设置电流输出(通过0x49)
int DAC60501_SetVoltage(float vout, float vref);
int DAC60501_RefAlarm(void);
#endif /* __DAC60501_H */
