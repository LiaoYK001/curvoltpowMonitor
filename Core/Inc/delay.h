#ifndef DELAY_H
#define DELAY_H

#include "main.h" /* 确保 HAL 的 HAL_Delay 可用与 u32 类型定义 */

void delay_init(void);
void delay_ms(uint32_t ms);
void DWT_Delay_Init(void);
void DWT_Delay_us(uint32_t us);
#endif /* DELAY_H */
