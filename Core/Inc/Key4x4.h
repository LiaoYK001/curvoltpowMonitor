#ifndef __KEY4X4_H
#define __KEY4X4_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 全局变量(可选择是否暴露)
extern uint8_t anxia;
extern uint8_t key;
extern uint8_t key_long_press; // 新增：长按标志

// 函数声明
void KEY_4x4_Init(void);
void KEY_Scan(void);
uint16_t Key_Read(void);
uint8_t Key_IsPressed(void);
void Key_Clear(void);
uint8_t Key_IsLongPress(void); // 新增：检查是否长按

#ifdef __cplusplus
}
#endif

#endif /* __KEY4X4_H */

// AI Claude Sonnet 4.5 辅助