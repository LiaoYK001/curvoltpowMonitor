/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     xph    first version
 */

#ifndef _INA260__H
#define _INA260__H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

// -- register addresses --
#define INA260_REG_CONFIG 0x00
#define INA260_REG_CURRENT 0x01
#define INA260_REG_VOLTAGE 0x02
#define INA260_REG_POWER 0x03
#define INA260_REG_MASK_EN 0x06
#define INA260_REG_ALRTLIM 0x07
#define INA260_REG_MFG_ID 0xFE
#define INA260_REG_DEV_ID 0xFF

// -- register LSB values --
#define INA260_LSB_CURRENT 1.25F
#define INA260_LSB_VOLTAGE 1.25F
#define INA260_LSB_POWER 10.00F

/****************************
 *         A0   A1  addr
 *         0    0   0x40
 *         0    1   0x41
 *         1    0   0x44
 *         1    1   0x45
 *************************/
#define INA260_SLAVE_ADDRESS 0x40 // INA260 的Slave默认地址

// hardware default settings: continuous voltage and current measurements, each
// using 1 sample of a 1.1 ms ADC conversion
#define DEFAULT_OPERATING_TYPE iotPower
#define DEFAULT_OPERATING_MODE iomContinuous
#define DEFAULT_CURRENT_CTIME ictConvert1p1ms
#define DEFAULT_VOLTAGE_CTIME ictConvert1p1ms
#define DEFAULT_SAMPLE_SIZE issSample1;

#define INA_STATUS_OK (0U)
#define INA_STATUS_ERROR (-1U)
#define INA_STATUS_TIMEOUT (-2U)

// convert value at addr to little-endian (16-bit)
/*
1. 16位小端序转换宏 (__LEu16)
__LEu16(addr) 宏旨在将存储在地址 addr 处的两个连续字节解释为一个 16
位的无符号整数。
逻辑分析：它通过指针算术访问内存。*(((uint8_t *)(addr)) + 0)
获取第一个字节，*(((uint8_t *)(addr)) + 1) 获取第二个字节。 位操作：它将地址 +0
处的字节左移 8 位（作为高字节），并将地址 +1
处的字节保持在低位（作为低字节），然后通过按位或（|）组合起来。
注意：尽管注释写着 "convert... to
little-endian"（转换为小端序），但从代码逻辑来看（addr+0
移位到高位），这实际上是在将一个大端序（Big-Endian）存储的字节流（高字节在前）转换为当前处理器可用的
16 位整数。这在 I2C 通信中非常常见，因为 I2C 协议通常以大端序传输数据。
*/
#define __LEu16(addr)                                                          \
  (((((uint16_t)(*(((uint8_t *)(addr)) + 1)))) |                               \
    (((uint16_t)(*(((uint8_t *)(addr)) + 0))) << 8U)))

// convert value at addr to little-endian (32-bit)
/*
2. 32位小端序转换宏 (__LEu32)
__LEu32(addr) 宏执行类似的操作，但是针对 32 位整数。
逻辑分析：它读取从 addr 开始的四个连续字节。
位操作：
addr + 0 左移 24 位（最高字节）。
addr + 1 左移 16 位。
addr + 2 左移 8 位。
addr + 3 保持在低位（最低字节）。
用途：同样，这通常用于将网络字节序或大端序的原始数据流重组为 32 位整数值。
*/
#define __LEu32(addr)                                                          \
  (((((uint32_t)(*(((uint8_t *)(addr)) + 3)))) |                               \
    (((uint32_t)(*(((uint8_t *)(addr)) + 2))) << 8U) |                         \
    (((uint32_t)(*(((uint8_t *)(addr)) + 1))) << 16U) |                        \
    (((uint32_t)(*(((uint8_t *)(addr)) + 0))) << 24U)))

// swap two values `a` and `b` with a given type `t`
/*
3. 通用交换宏 (__SWAP)
__SWAP(t, a, b) 是一个通用的交换宏，用于交换两个变量的值。
参数：它接受三个参数：t 是变量的类型，a 和 b 是要交换的两个变量。
实现：它使用了一个临时的代码块 { ... }
来创建一个局部作用域。在这个作用域内，它声明了一个类型为 t 的临时变量
s，通过经典的“三步走”方式（保存 a 到 s，赋值 b 给 a，赋值 s 给 b）来完成交换。
优点：使用代码
块 {} 可以防止宏在 if-else 语句中展开时出现语法错误（即著名的 "dangling else"
问题），这是一种安全的宏编写习惯。
*/
#define __SWAP(t, a, b)                                                        \
  {                                                                            \
    t s;                                                                       \
    s = a;                                                                     \
    a = b;                                                                     \
    b = s;                                                                     \
  }

// format of the MASK/ENABLE register (06h)
/* TODO: add ALRT support */
typedef union {
  uint16_t u16;
  struct {
    uint8_t alert_latch_enable : 1;  //  0
    uint8_t alert_polarity : 1;      //  1
    uint8_t math_overflow : 1;       //  2
    uint8_t conversion_ready : 1;    //  3
    uint8_t alert_function_flag : 1; //  4
    uint8_t resv : 5;                //  5 -  9
    uint8_t alert_conversion : 1;    // 10
    uint8_t alert_over_power : 1;    // 11
    uint8_t alert_under_voltage : 1; // 12
    uint8_t alert_over_voltage : 1;  // 13
    uint8_t alert_under_current : 1; // 14
    uint8_t alert_over_current : 1;  // 15
  };
} ina260_mask_enable_t;

// format of the DEVICE_ID register (FFh)
typedef union {
  uint16_t u16;
  struct {
    uint8_t revision : 4;
    uint16_t device : 12;
  };
} ina260_identification_t;

// determines the number of samples that are collected and averaged. each sample
// requires a given amount of ADC conversion time, and there are only a small
// number of discrete times that may be selected (see ina260_conversion_time_t).
typedef enum {
  issSample1,    // = 0 (000b) -- default
  issSample4,    // = 1 (001b)
  issSample16,   // = 2 (010b)
  issSample64,   // = 3 (011b)
  issSample128,  // = 4 (100b)
  issSample256,  // = 5 (101b)
  issSample512,  // = 6 (110b)
  issSample1024, // = 7 (111b)
} ina260_sample_size_t;

// sets the conversion time for the voltage and current measurement. these are
// the only intervals recognized by the INA260 hardware (per the datasheet).
// one complete lapse in the selected duration represents 1 sample. therefore
// the total time required for a single measurement is calculated as selected
// conversion time (ina260_conversion_time_t) multiplied by the selected number
// of samples (ina260_sample_size_t).
typedef enum {
  ictConvert140us,   // = 0 (000b)
  ictConvert204us,   // = 1 (001b)
  ictConvert332us,   // = 2 (010b)
  ictConvert588us,   // = 3 (011b)
  ictConvert1p1ms,   // = 4 (100b) -- default (voltage, current)
  ictConvert2p116ms, // = 5 (101b)
  ictConvert4p156ms, // = 6 (110b)
  ictConvert8p244ms, // = 7 (111b)
} ina260_conversion_time_t;

// determines how measurements should be performed and updated in internal
// data registers. you can read the contents at any time in both modes, but
// this will affect when they update.
typedef enum {
  iomTriggered,  // = 0 (000b)
  iomContinuous, // = 1 (001b) -- default
} ina260_operating_mode_t;

// specifies which measurements are performed for each conversion. you can
// perform current-only, voltage-only, or both current and voltage. note the
// bit patterns result in the following equalities:
//   iotShutdown == 0
//   iotPower    == ( iotVoltage | iotCurrent )
typedef enum {
  iotShutdown, // = 0 (000b)
  iotCurrent,  // = 1 (001b)
  iotVoltage,  // = 2 (010b)
  iotPower,    // = 3 (011b) -- default
} ina260_operating_type_t;

// format of CONFIGURATION register (00h)
typedef union {
  uint16_t u16;
  struct {
    ina260_operating_type_t type : 2;   //  0 -  1
    ina260_operating_mode_t mode : 1;   //  2
    ina260_conversion_time_t ctime : 3; //  3 -  5
    ina260_conversion_time_t vtime : 3; //  6 -  8
    ina260_sample_size_t ssize : 3;     //  9 - 11
    uint8_t resv : 3;                   // 12 - 14
    uint8_t reset : 1;                  // 15
  };
} ina260_configuration_t;

typedef int (*i2c_func)(uint8_t addr, uint16_t *data, uint8_t data_len);

// struct definition for primary device type
typedef struct _ina260_device_t {
  i2c_func i2c_read;
  i2c_func i2c_write;
  uint8_t i2c_addr;
  ina260_configuration_t config;
} ina260_device_t;

/** -- exported functions -- */
int ina260_init(i2c_func i2c_read, i2c_func i2c_write, uint8_t addr);
int ina260_ready(void);
int ina260_wait_until_ready(uint32_t timeout);
int ina260_set_config(ina260_operating_type_t operating_type,
                      ina260_operating_mode_t operating_mode,
                      ina260_conversion_time_t current_ctime,
                      ina260_conversion_time_t voltage_ctime,
                      ina260_sample_size_t sample_size);
int ina260_set_operating_type(ina260_operating_type_t operating_type);
int ina260_set_operating_mode(ina260_operating_mode_t operating_mode);
int ina260_set_conversion_time(ina260_conversion_time_t time);
int ina260_set_current_conversion_time(ina260_conversion_time_t time);
int ina260_set_voltage_conversion_time(ina260_conversion_time_t time);
int ina260_set_sample_size(ina260_sample_size_t sample_size);
int ina260_reset(uint8_t init);
int ina260_conversion_ready(void);
int ina260_conversion_start(void);
int ina260_get_voltage(float *voltage);
int ina260_get_current(float *current);
int ina260_get_power(float *power);
int ina260_init_default(uint8_t addr); // 添加初始化函数
#ifdef __cplusplus
}
#endif

#endif /* INA260_H_ */
