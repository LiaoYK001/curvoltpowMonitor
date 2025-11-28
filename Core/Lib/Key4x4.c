#include "Key4x4.h"
#include "delay.h"
#include "main.h"
#include "stm32f1xx_hal.h"

// anxia是是否按下且触发事件标志(释放)
// anxia为完整按键触发流程,包括按下和释放(长按类释放)
uint8_t anxia = 0;
// key是按下的键值索引
uint8_t key = 0;
// 新增：长按标志 (1=长按, 0=短按)
uint8_t key_long_press = 0;

// 长按时间阈值 (毫秒)
#define LONG_PRESS_TIME 1000

// 新增：按键状态机
static uint8_t key_pressed = 0;          // 按键是否被按下
static uint8_t key_processed = 0;        // 按键是否已处理（防止重复触发）
static uint32_t key_press_start = 0;     // 按键按下时间戳
static uint8_t last_key_index = 0xFF;    // 上次按键索引
static uint8_t long_press_triggered = 0; // 长按是否已触发

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

/**
 * @brief  键值映射表
 * @note   - 修改 '#' 为 '.', 'D' 为 'E'(Enter/确认)
 *         - A对应电压设定，B对应电流设定
 *         - C对应Clear全清空(后续修改为开关),*对应取消Cancel
 *
 */
const uint16_t keys[16] = {
    '1', '2', '3', 'A', '4', '5', '6', 'B',
    '7', '8', '9', 'C', '*', '0', '.', 'E' // '#' -> '.', 'D' -> 'E'
};

/**
 * @brief  初始化 4x4 矩阵键盘
 * @note   引脚已通过 CubeMX 配置,此函数仅设置初始状态
 *
 */
void KEY_4x4_Init(void) {
  // CubeMX 已经初始化引脚,这里只设置初始状态:
  HAL_GPIO_WritePin(KEY_ROW0_GPIO_Port,
                    KEY_ROW0_Pin | KEY_ROW1_Pin | KEY_ROW2_Pin | KEY_ROW3_Pin,
                    GPIO_PIN_SET);

  /* 如果没有使用 CubeMX,以下代码进行手动初始化

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
 * @brief  检查指定按键是否仍然按下
 * @param  row: 行号
 * @param  col: 列号
 * @retval 1: 按下, 0: 释放
 */
static uint8_t Is_Key_Still_Pressed(uint8_t row, uint8_t col) {
  // 设置当前行为低电平
  for (uint8_t i = 0; i < 4; i++) {
    HAL_GPIO_WritePin(KEY_ROW0_GPIO_Port, row_pins[i],
                      (i == row) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }

  DWT_Delay_us(10); // 等待电平稳定

  // 检查列引脚
  GPIO_PinState state = HAL_GPIO_ReadPin(KEY_COL0_GPIO_Port, col_pins[col]);

  return (state == GPIO_PIN_RESET) ? 1 : 0;
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

  // 短暂延迟让电平稳定 10 微秒
  DWT_Delay_us(10);

  // 检查所有列
  for (uint8_t col = 0; col < 4; col++) {
    if (HAL_GPIO_ReadPin(KEY_COL0_GPIO_Port, col_pins[col]) == GPIO_PIN_RESET) {
      // 软件消抖延迟
      delay_ms(20);

      // 再次确认
      if (HAL_GPIO_ReadPin(KEY_COL0_GPIO_Port, col_pins[col]) ==
          GPIO_PIN_RESET) {
        uint8_t key_index = row * 4 + col;

        // 如果是新按键按下
        if (!key_pressed || last_key_index != key_index) {
          key_pressed = 1;
          key_processed = 0;
          key_press_start = HAL_GetTick();
          last_key_index = key_index;
          key = key_index;
          key_long_press = 0;
          long_press_triggered = 0;
          anxia = 0; // 按下时不触发事件
        }
        // 如果是同一个按键持续按下
        else if (key_index == 12 &&
                 !long_press_triggered) { // 只对 '*' 键检测长按
          uint32_t press_duration = HAL_GetTick() - key_press_start;
          if (press_duration >= LONG_PRESS_TIME) {
            key_long_press = 1;
            long_press_triggered = 1; // 标记长按已触发，避免重复
            anxia = 1; // 触发按键事件,直接Clear,而不是等待释放(失去长按作用)
          }
        }

        return 1;
      }
    }
  }

  return 0;
}

/**
 * @brief  扫描 4x4 矩阵键盘 (非阻塞式)
 * @note   调用此函数检测按键,建议在主循环中周期调用
 */
void KEY_Scan(void) {
  uint8_t key_found = 0;

  // 扫描所有行
  for (uint8_t row = 0; row < 4; row++) {
    if (KEY_Scan_Row(row)) {
      key_found = 1;
      break;
    }
  }

  // 如果之前有按键按下，但现在没有检测到
  if (key_pressed && !key_found) {
    // 检查按键是否真的释放了
    uint8_t row = last_key_index / 4;
    uint8_t col = last_key_index % 4;

    if (!Is_Key_Still_Pressed(row, col)) {
      // 按键释放 - 在释放时触发事件
      anxia = 1; // 标记有按键事件

      // 如果没有触发长按，则为短按
      if (!long_press_triggered) {
        key_long_press = 0;
      }
      // 如果已触发长按，key_long_press 保持为 1

      key_pressed = 0;
      key_processed = 0;
      long_press_triggered = 0;

      // 释放后延迟，防止抖动
      delay_ms(50);
    }
  }

  // 如果没有找到按键且没有按键按下状态，清除事件标志
  if (!key_found && !key_pressed && key_processed) {
    anxia = 0;
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
 * @brief  检查是否长按
 * @retval 1: 长按, 0: 短按
 */
uint8_t Key_IsLongPress(void) { return key_long_press; }

/**
 * @brief  清除按键状态
 */
void Key_Clear(void) {
  anxia = 0;
  key = 0;
  key_long_press = 0;
  key_processed = 1; // 标记已处理
}