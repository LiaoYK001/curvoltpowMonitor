#include "dac60501.h"
#include "stm32f1xx_hal.h" /* for HAL_Delay, HAL status types (i2c.h already includes main.h normally) */
#include <string.h>

/* ----------------- 基础 I2C 读写实现 -----------------
 * 使用 HAL_I2C_Mem_Write / HAL_I2C_Mem_Read，寄存器地址宽度为
 * 8-bit（I2C_MEMADD_SIZE_8BIT）。
 *
 * 说明：
 *  - HAL 的 DevAddress 参数通常传入 (7bit_addr << 1)（即 8-bit 形式）。
 *  - I2C_WriteByte / I2C_ReadByte 保持原有签名以兼容历史调用。
 */

/* 写 16-bit 数据到指定寄存器（高字节先行） */
void I2C_WriteByte(uint8_t reg_addr, uint16_t data) {
  uint8_t tx[2];
  tx[0] = (uint8_t)(data >> 8);   /* 高字节 */
  tx[1] = (uint8_t)(data & 0xFF); /* 低字节 */

  uint16_t dev = (uint16_t)(DEV_ADDR_7BIT << 1);

  /* HAL_I2C_Mem_Write 会发送： START + DevAddr(W) + MemAddr + Data[] + STOP */
  if (HAL_I2C_Mem_Write(&hi2c2, dev, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT,
                        tx, 2, I2C_TIMEOUT) != HAL_OK) {
    /* 发送失败：在实际工程中可以在这里加重试、记录错误日志或设置故障标志 */
    /* 例如：HAL_I2C_GetError(&hi2c2) 可用于获取失败原因 */
  }

  /* 可选短延时（与原 bit-bang 行为保持相似） */
  HAL_Delay(1);
}

/* 读 16-bit 数据（高字节在前），读取失败返回 0xFFFF */
uint16_t I2C_ReadByte(uint8_t reg_addr) {
  uint8_t rx[2];
  uint16_t dev = (uint16_t)(DEV_ADDR_7BIT << 1);

  if (HAL_I2C_Mem_Read(&hi2c2, dev, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT,
                       rx, 2, I2C_TIMEOUT) != HAL_OK) {
    /* 读取失败 */
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
void DAC60501_Init(void) {

  HAL_Delay(10);
  /* 1) 确保内部参考开启（这里直接写 0x0000，最保险） */
  I2C_WriteByte(CONFIG, 0x0000);
  HAL_Delay(2); /* 给内部参考一些上电稳定时间 */

  /* 2)
   * https://e2echina.ti.com/support/machine-translation/mt-data-converters/f/mt-data-converters-forum/367292/dac60501
   * 修正问题 使用 write 覆盖
   */
  I2C_WriteByte(GAIN, 0x0101);
  HAL_Delay(2);

  /* 3) 软复位（可选）: TRIGGER 写 0x000A 做 soft reset（datasheet 指示）
   * 如果不需要可注释掉这一行。
   */
  // I2C_WriteByte(TRIGGER, 0x000A);
  // HAL_Delay(2);

  I2C_WriteByte(SYNC, 0x0000);
  HAL_Delay(2);

  /* 3) 输出进行归零 */
  I2C_WriteByte(DAC_DATA, 0x0000);
  HAL_Delay(2);
}

/* 读取设备 ID（DEVID） */
uint16_t DAC60501_ReadDeviceId(void) { return I2C_ReadByte(DEVID); }

/* 读取 STATUS 寄存器 */
uint16_t DAC60501_ReadStatus(void) { return I2C_ReadByte(STATUS); }

/* 设置 GAIN（read-modify-write）
 * - 我们先读取当前 GAIN 寄存器值，再只修改 REF-DIV 和 BUFF-GAIN
 * 位，避免破坏保留位或厂家默认位。
 * - ref_div: 0/1
 * - buff_gain: 0/1
 * 返回：0 = OK；-1 = 读失败；-2 = 写失败 (写失败无法直接检测，因 I2C_WriteByte
 * 无返回，这里简单返回 0)
 */
int DAC60501_SetGain(uint8_t ref_div, uint8_t buff_gain) {
  uint16_t cur = I2C_ReadByte(GAIN);
  if (cur == 0xFFFF)
    return -1; /* 读取失败 */

  /* 只清除我们要改的位（bit8 和 bit0），保留其它位 */
  cur &= ~(GAIN_REF_DIV_BIT | GAIN_BUFF_GAIN_BIT);

  if (ref_div)
    cur |= GAIN_REF_DIV_BIT;
  if (buff_gain)
    cur |= GAIN_BUFF_GAIN_BIT;

  I2C_WriteByte(GAIN, cur);
  HAL_Delay(2);

  return 0;
}

/* 检查 REF-ALARM（1 表示参考缓冲关闭 -> 输出 0V）
 * 返回：1 = alarm，0 = ok，-1 = i2c 错误
 */
int DAC60501_RefAlarm(void) {
  uint16_t s = DAC60501_ReadStatus();
  if (s == 0xFFFF)
    return -1;
  return ((s & STATUS_REF_ALARM) ? 1 : 0);
}

/* 将期望电压写入 DAC（假设已按安全方式设置 REF/DIV/Gain）
 * - vout: 目标电压 (V)
 * - vref: 参考电压 (V)（内部 ref = 2.5V）
 *
 * 12-bit 公式（详见注释）：
 *   CODE = round( vout / (vref_eff * G) * (2^12 - 1) )
 *   reg_value = CODE << 4
 *
 * 返回：
 *   0 = OK
 *  -1 = 参数错误 (vref <= 0)
 *  -2 = 写入后检测到 REF-ALARM（参考报警）
 *  -3 = I2C 读写失败（如 I2C_ReadByte 返回 0xFFFF）
 */
int DAC60501_SetVoltage(float vout, float vref) {
  if (vref <= 0.0f)
    return -1;

  /* 读取当前 GAIN 寄存器来确定 REF-DIV / BUFF-GAIN */
  uint16_t gain_reg = I2C_ReadByte(GAIN);
  if (gain_reg == 0xFFFF)
    return -3; /* I2C 读失败 */

  uint8_t ref_div = ((gain_reg & GAIN_REF_DIV_BIT) ? 1 : 0);
  uint8_t buff_gain = ((gain_reg & GAIN_BUFF_GAIN_BIT) ? 1 : 0);

  /* 计算生效的参考与放大倍数 */
  float vref_eff = ref_div ? (vref / 2.0f) : vref;
  float gain = buff_gain ? 2.0f : 1.0f;

  /* 限幅：vout 必须在 [0, vref_eff * gain] */
  float vmax = vref_eff * gain;
  if (vout < 0.0f)
    vout = 0.0f;
  if (vout > vmax)
    vout = vmax;

  const uint32_t maxcode = 4095U; /* 12-bit 最大码 */
  /* 计算 CODE（四舍五入） */
  float ratio = 0.0f;
  if (vmax > 0.0f)
    ratio = vout / (vref_eff * gain);
  uint32_t code = (uint32_t)(ratio * (float)maxcode + 0.5f);
  if (code > maxcode)
    code = maxcode;

  /* 左对齐 12-bit 到 16-bit */
  uint16_t regval = (uint16_t)(code << 4);

  /* 写入 DAC_DATA 寄存器 */
  I2C_WriteByte(DAC_DATA, regval);
  HAL_Delay(1);

  /* 写后检查 REF-ALARM（若 alarm 发生，输出会被拉到 0V） */
  int alarm = DAC60501_RefAlarm();
  if (alarm == 1) {
    /* 参考报警：建议上层回退配置或触发错误处理 */
    return -2;
  } else if (alarm == -1) {
    return -3; /* I2C 读 status 错误 */
  }

  return 0;
}
