#include "dac60501.h"
#include "string.h"

/*
 * 使用 HAL_I2C_Mem_Write / HAL_I2C_Mem_Read 实现：
 * 设备地址传给 HAL 时使用 (DEV_ADDR_7BIT << 1)
 * 内部寄存器地址用 8-bit (I2C_MEMADD_SIZE_8BIT)
 */

void DAC60501_Init(void) {
  /* 1) 确保 CONFIG: 关闭参考掉电使用内部参考，DAC 不掉电 (REF_PWDWN=0,
   * DAC_PWDWN=0) */
  I2C_WriteByte(CONFIG, 0x0000);

  /* 2) GAIN: 如果想要 buffer gain = 1 (默认), 不改变; 示例把 BUFF-GAIN=1 */
  I2C_WriteByte(GAIN, 0x0000); /* 低位 bit0 = BUFF-GAIN */

  /* 3) 硬件或软件复位（可选）: TRIGGER 写 0x000A 做 soft reset */
  I2C_WriteByte(TRIGGER, 0x000A);

  /* 小延时让内部参考稳定 */
  HAL_Delay(2);
}

/* 写 16-bit 数据到指定寄存器（reg_addr），数据以高字节在前（与原 bit-bang
 * 代码一致） */
void I2C_WriteByte(uint8_t reg_addr, uint16_t data) {
  uint8_t tx[2];
  tx[0] = (uint8_t)(data >> 8);
  tx[1] = (uint8_t)(data & 0xFF);

  /* HAL 期望的 DevAddress 为 7-bit 左移 1（即 8-bit 地址格式） */
  uint16_t dev = (uint16_t)(DEV_ADDR_7BIT << 1);

  /* 使用 Mem 写：发送 [DevAddr][RegAddr][data_hi][data_lo] */
  if (HAL_I2C_Mem_Write(&hi2c2, dev, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT,
                        tx, 2, I2C_TIMEOUT) != HAL_OK) {
    /* 发送失败：可以在此添加重试或错误记录 */
    /* 示例不做额外处理 */
  }

  /* 可选短延时（与原 bit-bang 版本类似） */
  HAL_Delay(1);
}

/* 读 16-bit 数据（先写寄存器地址，再读 2 字节） */
/* 返回：16-bit 数据（高字节在前）；读取失败返回 0xFFFF */
uint16_t I2C_ReadByte(uint8_t reg_addr) {
  uint8_t rx[2];
  uint16_t dev = (uint16_t)(DEV_ADDR_7BIT << 1);

  /* 使用 Mem 读，HAL 会先发送寄存器地址然后读数据 */
  if (HAL_I2C_Mem_Read(&hi2c2, dev, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT,
                       rx, 2, I2C_TIMEOUT) != HAL_OK) {
    /* 读取失败 */
    return 0xFFFF;
  }

  return (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
}
