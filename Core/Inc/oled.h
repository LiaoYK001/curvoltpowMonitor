/* oled.h 通过 AI 辅助修复部分内容和增加部分内容 */

#ifndef __OLED_H
#define __OLED_H

/* 用 HAL 方式：包含 main.h（CubeMX 生成，包含 stm32f1xx_hal.h 与引脚定义） */
#include "main.h"
#include <stdlib.h>

/* types 兼容：如果你有 types.h 则包含它，否则包含 stdint 并定义短名 */
#ifdef HAS_TYPES_H
#include "types.h"
#else
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
#endif

/* ----------------- OLED I/O 宏（HAL 版本）-----------------
   说明：
   - main.h（或 CubeMX）里有如下宏：
       oled_scl_Pin, oled_scl_GPIO_Port
       oled_sda_Pin, oled_sda_GPIO_Port

   - 如果你使用不同的命名，请把下面的映射改成相应宏名，或直接在这里写
   GPIOA/GPIO_PIN_x
*/

#ifndef OLED_SCL_PORT
#ifdef oled_scl_GPIO_Port
#define OLED_SCL_PORT oled_scl_GPIO_Port
#else
#define OLED_SCL_PORT GPIOB /* fallback: 修改为你的端口 */
#endif
#endif

#ifndef OLED_SCL_PIN
#ifdef oled_scl_Pin
#define OLED_SCL_PIN oled_scl_Pin
#else
#define OLED_SCL_PIN GPIO_PIN_6 /* fallback: 修改为你的引脚 */
#endif
#endif

#ifndef OLED_SDA_PORT
#ifdef oled_sda_GPIO_Port
#define OLED_SDA_PORT oled_sda_GPIO_Port
#else
#define OLED_SDA_PORT GPIOB
#endif
#endif

#ifndef OLED_SDA_PIN
#ifdef oled_sda_Pin
#define OLED_SDA_PIN oled_sda_Pin
#else
#define OLED_SDA_PIN GPIO_PIN_7
#endif
#endif

/* 可选 RES 引脚（若未在项目中定义，可在 main.h 加定义或在此处修改） */
#ifndef OLED_RES_PORT
#ifdef oled_res_GPIO_Port
#define OLED_RES_PORT oled_res_GPIO_Port
#else
/* 如果没有 RES，保持定义但指向 SCL（仅占位，建议在 main.h 中定义实际引脚） */
#define OLED_RES_PORT OLED_SCL_PORT
#endif
#endif

#ifndef OLED_RES_PIN
#ifdef oled_res_Pin
#define OLED_RES_PIN oled_res_Pin
#else
#define OLED_RES_PIN OLED_SCL_PIN
#endif
#endif

/* HAL 读写宏（统一接口，便于 oled.c 不改大量代码） */
#define OLED_SCL_Set()                                                         \
  HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET)
#define OLED_SCL_Clr()                                                         \
  HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_RESET)

#define OLED_SDA_Set()                                                         \
  HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define OLED_SDA_Clr()                                                         \
  HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)

#define OLED_RES_Set()                                                         \
  HAL_GPIO_WritePin(OLED_RES_PORT, OLED_RES_PIN, GPIO_PIN_SET)
#define OLED_RES_Clr()                                                         \
  HAL_GPIO_WritePin(OLED_RES_PORT, OLED_RES_PIN, GPIO_PIN_RESET)

#define OLED_SDA_Read() HAL_GPIO_ReadPin(OLED_SDA_PORT, OLED_SDA_PIN)

#define OLED_SCL_Toggle() HAL_GPIO_TogglePin(OLED_SCL_PORT, OLED_SCL_PIN)
#define OLED_SDA_Toggle() HAL_GPIO_TogglePin(OLED_SDA_PORT, OLED_SDA_PIN)

/* ------------------------------------------------------------------ */
#define OLED_CMD 0  // д����
#define OLED_DATA 1 // д����

void OLED_ClearPoint(u8 x, u8 y);
void OLED_ColorTurn(u8 i);
void OLED_DisplayTurn(u8 i);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_WaitAck(void);
void Send_Byte(u8 dat);
void OLED_WR_Byte(u8 dat, u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x, u8 y, u8 t);
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2, u8 mode);
void OLED_DrawCircle(u8 x, u8 y, u8 r);
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size1, u8 mode);
void OLED_ShowChar6x8(u8 x, u8 y, u8 chr, u8 mode);
void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size1, u8 mode);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size1, u8 mode);
void OLED_ShowChinese(u8 x, u8 y, u8 num, u8 size1, u8 mode);
void OLED_ScrollDisplay(u8 num, u8 space, u8 mode);
void OLED_ShowPicture(u8 x, u8 y, u8 sizex, u8 sizey, u8 BMP[], u8 mode);
void OLED_Init(void);
void OLED_CN(u8 x, u8 y, u8 num, u8 size1, u8 mode, u8 num2);
//  补充HYZ的函数在声明
//  注：传入 char*（signed char），会收到编译器警告。推荐在 main.c 中使用 u8
//  mes[]。
#endif
