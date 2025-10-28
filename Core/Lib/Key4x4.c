#include "Key4x4.h"
#include "delay.h"
#include "main.h"
#include "stm32f1xx_hal.h"

uint8_t anxia = 0;
uint8_t key = 0;

// 行引脚定义 (输出) - 使用 CubeMX 生成的宏定义
const uint16_t row_pins[4] = {
    KEY_ROW0_Pin, // 或 GPIO_PIN_0
    KEY_ROW1_Pin, // 或 GPIO_PIN_1
    KEY_ROW2_Pin, // 或 GPIO_PIN_2
    KEY_ROW3_Pin  // 或 GPIO_PIN_3
};

// 列引脚定义 (输入)
const uint16_t col_pins[4] = {
    KEY_COL0_Pin, // 或 GPIO_PIN_4
    KEY_COL1_Pin, // 或 GPIO_PIN_5
    KEY_COL2_Pin, // 或 GPIO_PIN_6
    KEY_COL3_Pin  // 或 GPIO_PIN_7
};

// 键值映射表
// 键值映射表 - 修改 '#' 为 '.', 'D' 为 'E'(Enter/确认)
const uint16_t keys[16] = {
    '1', '2', '3', 'A', '4', '5', '6', 'B',
    '7', '8', '9', 'C', '*', '0', '.', 'E' // '#' -> '.', 'D' -> 'E'
};

/**
 * @brief  初始化 4x4 矩阵键盘
 * @note   引脚已通过 CubeMX 配置,此函数仅设置初始状态
 *         如果未使用 CubeMX,需要取消注释下面的手动初始化代码
 */
void KEY_4x4_Init(void) {
  // CubeMX 已经初始化引脚,这里只设置初始状态
  HAL_GPIO_WritePin(KEY_ROW0_GPIO_Port,
                    KEY_ROW0_Pin | KEY_ROW1_Pin | KEY_ROW2_Pin | KEY_ROW3_Pin,
                    GPIO_PIN_SET);

  /* 如果没有使用 CubeMX,取消注释以下代码进行手动初始化:

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // 使能 GPIOA 时钟
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // 配置行引脚 (PA0-PA3) 为推挽输出
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // 配置列引脚 (PA4-PA7) 为上拉输入
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // 初始状态:所有行引脚置高
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                    GPIO_PIN_SET);
  */
}

/**
 * @brief  设置所有行引脚状态
 * @param  state: GPIO_PIN_SET 或 GPIO_PIN_RESET
 */
static void Set_All_Rows(GPIO_PinState state) {
  for (uint8_t i = 0; i < 4; i++) {
    HAL_GPIO_WritePin(KEY_ROW0_GPIO_Port, row_pins[i], state);
  }
}

/**
 * @brief  检测按键按下并等待释放
 * @param  col_pin: 列引脚编号
 * @param  key_index: 键值索引
 */
static void Do_Click(uint16_t col_pin, uint8_t key_index) {
  anxia = 1;
  key = key_index;

  // 等待按键释放
  while (HAL_GPIO_ReadPin(KEY_COL0_GPIO_Port, col_pin) == GPIO_PIN_RESET) {
    DWT_Delay_us(100);
  }
}

/**
 * @brief  扫描某一行的按键
 * @param  row: 行号 (0-3)
 * @retval 1: 检测到按键, 0: 未检测到
 */
static uint8_t KEY_Scan_Row(uint8_t row) {
  // 设置当前行为低电平,其他行为高电平
  for (uint8_t i = 0; i < 4; i++) {
    HAL_GPIO_WritePin(KEY_ROW0_GPIO_Port, row_pins[i],
                      (i == row) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }

  // 短暂延迟让电平稳定
  DWT_Delay_us(10);

  // 检查所有列
  for (uint8_t col = 0; col < 4; col++) {
    if (HAL_GPIO_ReadPin(KEY_COL0_GPIO_Port, col_pins[col]) == GPIO_PIN_RESET) {
      // 消抖延迟
      delay_ms(10);

      // 再次确认
      if (HAL_GPIO_ReadPin(KEY_COL0_GPIO_Port, col_pins[col]) ==
          GPIO_PIN_RESET) {
        uint8_t key_index = row * 4 + col;
        Do_Click(col_pins[col], key_index);
        return 1;
      }
    }
  }

  return 0;
}

/**
 * @brief  扫描 4x4 矩阵键盘
 * @note   调用此函数检测按键,建议在主循环中周期调用
 */
void KEY_Scan(void) {
  anxia = 0;

  // 扫描所有行
  for (uint8_t row = 0; row < 4; row++) {
    if (KEY_Scan_Row(row)) {
      break;
    }
  }

  // 恢复所有行为高电平
  Set_All_Rows(GPIO_PIN_SET);
}

/**
 * @brief  读取按键值
 * @retval 按键字符 ('0'-'9', 'A'-'D', '*', '#')
 *         如果没有按键按下,返回 0
 */
uint16_t Key_Read(void) {
  if (anxia && key < 16) {
    return keys[key];
  }
  return 0;
}

/**
 * @brief  检查是否有按键按下
 * @retval 1: 有按键按下, 0: 无按键
 */
uint8_t Key_IsPressed(void) { return anxia; }

/**
 * @brief  清除按键状态
 */
void Key_Clear(void) {
  anxia = 0;
  key = 0;
}