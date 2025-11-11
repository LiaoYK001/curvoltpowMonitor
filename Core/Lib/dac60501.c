#include "dac60501.h"
/* for HAL_Delay, HAL status types (i2c.h already includes main.h normally) */
#include "stm32f1xx_hal.h"
#include <string.h>

/* ----------------- 基础 I2C 读写实现 -----------------
 * 使用 HAL_I2C_Mem_Write / HAL_I2C_Mem_Read，寄存器地址宽度为
 * 8-bit（I2C_MEMADD_SIZE_8BIT）。
 *
 * 说明：
 *  - HAL 的 DevAddress 参数通常传入 (7bit_addr << 1)（即 8-bit 形式）。
 *  - I2C_WriteByte / I2C_ReadByte 保持原有签名以兼容历史调用。
 */
/* DAC配置常量 */
const float DAC_VREF = 2.5f;     // DAC内部参考电压
const float DAC_VOUT_MAX = 2.5f; // DAC最大输出电压
const float MAX_VOLTAGE = 16.0f; // 系统最大电压
const float MAX_CURRENT = 1.0f;  // 系统最大电流
/* ----------------- 基础 I2C 读写实现 ----------------- */

/* 写 16-bit 数据到指定设备的寄存器 */
void I2C_WriteByte_Addr(uint8_t dev_addr, uint8_t reg_addr, uint16_t data) {
  uint8_t tx[2];
  tx[0] = (uint8_t)(data >> 8);
  tx[1] = (uint8_t)(data & 0xFF);

  uint16_t dev = (uint16_t)(dev_addr << 1);

  if (HAL_I2C_Mem_Write(&hi2c2, dev, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT,
                        tx, 2, I2C_TIMEOUT) != HAL_OK) {
    // 错误处理
  }
  HAL_Delay(1);
}

/* 读 16-bit 数据 */
uint16_t I2C_ReadByte_Addr(uint8_t dev_addr, uint8_t reg_addr) {
  uint8_t rx[2];
  uint16_t dev = (uint16_t)(dev_addr << 1);

  if (HAL_I2C_Mem_Read(&hi2c2, dev, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT,
                       rx, 2, I2C_TIMEOUT) != HAL_OK) {
    return 0xFFFF;
  }

  return (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
}

/* ----------------- 设备级别封装函数 ----------------- */

/* 安全初始化：
 * - 确保内部参考被启用（CONFIG = 0x0000 -> REF_PWDWN = 0，DAC 不掉电）
 * - 将 GAIN 设为安全值（REF-DIV = 0, BUFF-GAIN = 0）以避免超过 VDD headroom
 * - 执行可选软复位（TRIGGER = 0x000A），并等待参考稳定
 *
 * 返回：无（如果需要可修改为返回错误码）
 */
/* ----------------- 设备初始化 ----------------- */

/* 初始化指定地址的DAC */
void DAC60501_Init_Addr(uint8_t dev_addr) {
  HAL_Delay(10);

  /* 1) 确保内部参考开启 */
  I2C_WriteByte_Addr(dev_addr, CONFIG, 0x0000);
  HAL_Delay(2);

  /* 2) 设置GAIN */
  I2C_WriteByte_Addr(dev_addr, GAIN, 0x0101);
  HAL_Delay(2);

  /* 3) 同步寄存器 */
  I2C_WriteByte_Addr(dev_addr, SYNC, 0x0000);
  HAL_Delay(2);

  /* 4) 输出归零 */
  I2C_WriteByte_Addr(dev_addr, DAC_DATA, 0x0000);
  HAL_Delay(2);
}

/* 初始化所有DAC */
void DAC60501_Init_All(void) {
  DAC60501_Init_Addr(DAC_VOLTAGE_ADDR);
  DAC60501_Init_Addr(DAC_CURRENT_ADDR);
}

/* 读取设备 ID */
uint16_t DAC60501_ReadDeviceId_Addr(uint8_t dev_addr) {
  return I2C_ReadByte_Addr(dev_addr, DEVID);
}

/* 读取 STATUS 寄存器 */
uint16_t DAC60501_ReadStatus_Addr(uint8_t dev_addr) {
  return I2C_ReadByte_Addr(dev_addr, STATUS);
}

/* 设置 GAIN */
int DAC60501_SetGain_Addr(uint8_t dev_addr, uint8_t ref_div,
                          uint8_t buff_gain) {
  uint16_t cur = I2C_ReadByte_Addr(dev_addr, GAIN);
  if (cur == 0xFFFF)
    return -1;

  cur &= ~(GAIN_REF_DIV_BIT | GAIN_BUFF_GAIN_BIT);

  if (ref_div)
    cur |= GAIN_REF_DIV_BIT;
  if (buff_gain)
    cur |= GAIN_BUFF_GAIN_BIT;

  I2C_WriteByte_Addr(dev_addr, GAIN, cur);
  HAL_Delay(2);

  return 0;
}

/* 检查 REF-ALARM */
int DAC60501_RefAlarm_Addr(uint8_t dev_addr) {
  uint16_t s = DAC60501_ReadStatus_Addr(dev_addr);
  if (s == 0xFFFF)
    return -1;
  return ((s & STATUS_REF_ALARM) ? 1 : 0);
}

/* 设置DAC输出电压 */
int DAC60501_SetVoltage_Addr(uint8_t dev_addr, float vout, float vref) {
  if (vref <= 0.0f)
    return -1;

  uint16_t gain_reg = I2C_ReadByte_Addr(dev_addr, GAIN);
  if (gain_reg == 0xFFFF)
    return -3;

  uint8_t ref_div = ((gain_reg & GAIN_REF_DIV_BIT) ? 1 : 0);
  uint8_t buff_gain = ((gain_reg & GAIN_BUFF_GAIN_BIT) ? 1 : 0);

  float vref_eff = ref_div ? (vref / 2.0f) : vref;
  float gain = buff_gain ? 2.0f : 1.0f;

  float vmax = vref_eff * gain;
  if (vout < 0.0f)
    vout = 0.0f;
  if (vout > vmax)
    vout = vmax;

  const uint32_t maxcode = 4095U;
  float ratio = 0.0f;
  if (vmax > 0.0f)
    ratio = vout / (vref_eff * gain);
  uint32_t code = (uint32_t)(ratio * (float)maxcode + 0.5f);
  if (code > maxcode)
    code = maxcode;

  uint16_t regval = (uint16_t)(code << 4);

  I2C_WriteByte_Addr(dev_addr, DAC_DATA, regval);
  HAL_Delay(1);

  int alarm = DAC60501_RefAlarm_Addr(dev_addr);
  if (alarm == 1) {
    return -2;
  } else if (alarm == -1) {
    return -3;
  }

  return 0;
}

/* ----------------- 高层应用接口 ----------------- */

/**
 * @brief  设置电压输出(通过0x48 DAC控制)
 * @param  voltage: 目标电压 0~16V
 * @retval 0: 成功, <0: 失败
 */
int DAC60501_SetVoltageOutput(float voltage) {
  // 限幅
  if (voltage < 0.0f)
    voltage = 0.0f;
  if (voltage > MAX_VOLTAGE)
    voltage = MAX_VOLTAGE;

  // 线性映射: voltage(0~16V) -> DAC_Vout(0~2.5V)
  float dac_vout = (voltage / MAX_VOLTAGE) * DAC_VOUT_MAX;

  return DAC60501_SetVoltage_Addr(DAC_VOLTAGE_ADDR, dac_vout, DAC_VREF);
}

/**
 * @brief  设置电流输出(通过0x49 DAC控制)
 * @param  current: 目标电流 0~1A
 * @retval 0: 成功, <0: 失败
 */
int DAC60501_SetCurrentOutput(float current) {
  // 限幅
  if (current < 0.0f)
    current = 0.0f;
  if (current > MAX_CURRENT)
    current = MAX_CURRENT;

  // 线性映射: current(0~1A) -> DAC_Vout(0~2.5V)
  float dac_vout = (current / MAX_CURRENT) * DAC_VOUT_MAX;

  return DAC60501_SetVoltage_Addr(DAC_CURRENT_ADDR, dac_vout, DAC_VREF);
}