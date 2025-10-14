#include "delay.h"
#include "stm32f1xx.h"

/* 如果需要用 SysTick 已由 HAL_Init 配置，delay_init 可以为空或做一些初始化 */
void delay_init(void) {
  /* HAL 已经配置 SysTick 在 HAL_Init() 中，通常不需要额外操作 */
}

/* 简单封装 HAL_Delay */
void delay_ms(uint32_t ms) { HAL_Delay(ms); }

/* delay_dwt.c */

static uint8_t dwt_inited = 0;

void DWT_Delay_Init(void) {
  if (dwt_inited)
    return;
  /* enable trace and debug block */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  dwt_inited = 1;
}

/* delay in microseconds */
void DWT_Delay_us(uint32_t us) {
  if (!dwt_inited)
    DWT_Delay_Init();
  uint32_t cycles = (SystemCoreClock / 1000000UL) * us;
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles) {
    __NOP();
  }
}
