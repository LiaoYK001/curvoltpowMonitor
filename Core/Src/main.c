/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Key4x4.h"
#include "delay.h"
#include "ina260.h"
#include "oled.h"
#include "stdio.h"
#include "string.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t last_oled_update = 0;              // 上次 OLED 更新时间计数
const uint32_t OLED_UPDATE_INTERVAL = 1000; // OLED 更新间隔 1000ms = 1s

// 输入状态机(用于处理按键输入设定电压和电流)
typedef enum {    // enum是定义枚举类型的关键字
  INPUT_IDLE = 0, // 空闲状态,从0开始
  INPUT_VOLTAGE,  // 输入电压中INPUT_VOLTAGE = 1
  INPUT_CURRENT   // 输入电流中INPUT_CURRENT = 2
} InputState_t;

InputState_t input_state = INPUT_IDLE; // 当前输入状态
char input_buffer[10] = {0};           // 输入缓冲区
uint8_t input_index = 0;               // 输入位置

float SetVoltage = 0.00f; // 设定电压 (V)
float SetCurrent = 0.00f; // 设定电流 (A)

// 旋转编码器相关变量
uint16_t encoder_last_cnt = 1000;
const float ENCODER_STEP = 0.05f;
const float MAX_VOLTAGE = 16.0f;
const float MAX_CURRENT = 5.0f;
uint8_t input_mode = 0; // 0=键盘模式, 1=编码器模式
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Process_Key_Input(char key);
static void Clear_Input_Buffer(void);
static void Display_Input_Prompt(void);
static uint8_t Validate_And_Set_Value(void);
static void Process_Encoder(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  清空输入缓冲区
 */
static void Clear_Input_Buffer(void) {
  // memset函数用于将一块内存空间全部设置为某个值，这里是将输入缓冲区清零
  // 函数用于将一段内存区域设置为指定的值，通常用于初始化数组或结构体。
  // 它按字节赋值，意味着*每个字节*都会被设置为指定的值。
  // 当使用 memset 对非字符类型的数组进行赋值时，需要特别注意赋值的内容。
  // 例如，对于 int 类型的数组，只有当赋值为 -1 或 0 时才能得到预期的结果。
  // 这是因为 -1 和 0 在二进制中每一位都相同，所以每个字节都会被正确地赋值。
  memset(input_buffer, 0, sizeof(input_buffer));
  input_index = 0;
  input_mode = 0; // 模式为键盘输入模式或编码器输入模式 0=键盘模式, 1=编码器模式
}

/**
 * @brief  显示输入提示
 */
static void Display_Input_Prompt(void) {
  char prompt[32];

  if (input_mode == 1) {
    // 使用编码器时,提示在第5行显示
    if (input_state == INPUT_VOLTAGE) {
      sprintf(prompt, "[ROTARY] V:%.2fV      ", SetVoltage);
    } else if (input_state == INPUT_CURRENT) {
      sprintf(prompt, "[ROTARY] I:%.2fA      ", SetCurrent);
    }
  } else {
    // 使用键盘输入时,显示输入缓冲区
    if (input_state == INPUT_VOLTAGE) {
      sprintf(prompt, "SetV:%s#          ", input_buffer);
    } else if (input_state == INPUT_CURRENT) {
      sprintf(prompt, "SetI:%s#          ", input_buffer);
    }
  }
  OLED_ShowString(0, 54, (uint8_t *)prompt, 8, 1);
  OLED_Refresh();
}

/**
 * @brief  处理旋转编码器输入
 */
static void Process_Encoder(void) {
  // 获取当前编码器计数值
  uint16_t encoder_cnt = __HAL_TIM_GET_COUNTER(&htim1);

  // 计算计数差值
  int16_t cnt_diff = (int16_t)(encoder_cnt - encoder_last_cnt);

  // 如果计数值变化
  if (cnt_diff != 0) {
    // 切换到编码器模式,清空键盘缓冲区
    if (input_mode == 0) {
      input_mode = 1;
      memset(input_buffer, 0, sizeof(input_buffer));
      input_index = 0;
    }

    // 根据当前模式调节电压或电流
    if (input_state == INPUT_VOLTAGE) {
      // 调节电压模式
      SetVoltage += cnt_diff * ENCODER_STEP;

      // 限制范围 0.00 - 16.00 V
      if (SetVoltage < 0.0f) {
        SetVoltage = 0.0f;
      } else if (SetVoltage > MAX_VOLTAGE) {
        SetVoltage = MAX_VOLTAGE;
      }

    } else if (input_state == INPUT_CURRENT) {
      // 调节电流模式
      SetCurrent += cnt_diff * ENCODER_STEP;

      // 限制范围 0.00 - 5.00 A
      if (SetCurrent < 0.0f) {
        SetCurrent = 0.0f;
      } else if (SetCurrent > MAX_CURRENT) {
        SetCurrent = MAX_CURRENT;
      }
    }

    // 更新上次计数值
    encoder_last_cnt = encoder_cnt;

    // 显示提示信息
    Display_Input_Prompt();
  }
}

/**
 * @brief  验证并设置数值 简要说明
 * @retval 1: 成功, 0: 失败 返回说明
 */
static uint8_t Validate_And_Set_Value(void) {
  // atof函数用于将字符串转换为浮点数 ascii to floating point numbers
  // 这里将输入缓冲区的字符串转换为浮点数来和范围进行比较
  float value = atof(input_buffer);
  char msg[32];

  if (input_state == INPUT_VOLTAGE) {
    // 验证范围 0.00 - 16.00 V
    if (value < 0.0f || value > MAX_VOLTAGE) {
      sprintf(msg, "  Vset ERR:0-16V     ");
      OLED_ShowString(0, 54, (uint8_t *)msg, 8, 1);
      OLED_Refresh();
      HAL_Delay(2000);
      return 0;
    }
    // 若验证通过,设置变量
    SetVoltage = value;
    sprintf(msg, "  SetV:%.2fV OK     ", SetVoltage);

  } else if (input_state == INPUT_CURRENT) {
    // 验证范围 0.00 - 5.00 A
    if (value < 0.0f || value > MAX_CURRENT) {
      sprintf(msg, "  Iset ERR:0-5A     ");
      OLED_ShowString(0, 54, (uint8_t *)msg, 8, 1);
      OLED_Refresh();
      HAL_Delay(2000);
      return 0;
    }
    // 若验证通过,设置变量
    SetCurrent = value;
    sprintf(msg, "  SetI:%.2fA OK     ", SetCurrent);
  }
  // 展示成功信息和设定值
  OLED_ShowString(0, 54, (uint8_t *)msg, 8, 1);
  OLED_Refresh();
  HAL_Delay(1500);
  // 1: 成功
  return 1;
}

/**
 * @brief  处理按键输入
 * @param  key: 按键字符
 */
static void Process_Key_Input(char key) {
  switch (input_state) {
    // 如果在空闲状态按下时候:
  case INPUT_IDLE:
    if (key == 'A') {
      // 开始输入电压
      input_state = INPUT_VOLTAGE;
      Clear_Input_Buffer();
      Display_Input_Prompt();
    } else if (key == 'B') {
      // 开始输入电流
      input_state = INPUT_CURRENT;
      Clear_Input_Buffer();
      Display_Input_Prompt();
    } else if (key == 'C') {
      // 取消/清除设定值 Clear
      SetVoltage = 0.0f;
      SetCurrent = 0.0f;
      OLED_ShowString(0, 54, (uint8_t *)"    --- Clear OK ---    ", 8, 1);
      OLED_Refresh();
      HAL_Delay(1000);
    }
    break;

    // 如果在输入电压或电流状态按下时候:
  case INPUT_VOLTAGE:
  case INPUT_CURRENT:
    if (key == 'E') {
      // 确认输入 Enter
      if (input_mode == 1) {
        // 如果是用编码器调节的,直接确认
        char msg[32];
        if (input_state == INPUT_VOLTAGE) {
          sprintf(msg, "  SetV:%.2fV OK     ", SetVoltage);
        } else {
          sprintf(msg, "  SetI:%.2fA OK     ", SetCurrent);
        }
        OLED_ShowString(0, 54, (uint8_t *)msg, 8, 1);
        OLED_Refresh();
        HAL_Delay(1500);
        // 返回空闲状态
        input_state = INPUT_IDLE;
        Clear_Input_Buffer();

      } else if (input_index > 0) {
        // 进行验证和设置
        if (Validate_And_Set_Value()) {
          // 验证成功,返回空闲状态
          input_state = INPUT_IDLE;
          // 清空输入缓冲区
          Clear_Input_Buffer();
        } else {
          // 验证失败,重新输入
          // 清空输入缓冲区
          Clear_Input_Buffer();
          // 显示输入提示
          Display_Input_Prompt();
        }
      }
    } else if (key == '*') {
      // 退格/取消
      // 返回空闲状态
      input_state = INPUT_IDLE;
      // 清空输入缓冲区
      Clear_Input_Buffer();
      OLED_ShowString(0, 54, (uint8_t *)"      --- Cancel ---       ", 8, 1);
      OLED_Refresh();
      HAL_Delay(1000);

    } else if ((key >= '0' && key <= '9') || key == '.') {
      // 切换到键盘模式
      if (input_mode == 1) {
        input_mode = 0;
        memset(input_buffer, 0, sizeof(input_buffer));
        input_index = 0;
      }
      // 数字或小数点 - 只有在未使用编码器时才接受键盘输入
      if (input_index < 8) { // 限制输入长度
        // 检查小数点重复
        if (key == '.') {
          // 只允许一个小数点,for循环遍历当前输入缓冲区
          for (uint8_t i = 0; i < input_index; i++) {
            if (input_buffer[i] == '.') {
              return; // 已有小数点,忽略
            }
          }
        }
        // 添加到输入缓冲区
        input_buffer[input_index++] = key;
        // (回到)显示输入提示
        Display_Input_Prompt();
      }
    }
    break;
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // u8 t = ' ';
  delay_init();
  HAL_Delay(30); // for OLED screen initial
  OLED_Init();
  OLED_ColorTurn(0);   // 0正常显示，1 反色显示
  OLED_DisplayTurn(1); // 0正常显示 1 屏幕翻转显示
  // OLED部分代码参考取自中景园电子ZJY096I0400WG11技术资料(SSD1315)
  KEY_4x4_Init();                                 // 初始化4x4矩阵键盘
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // 启动编码器接口
  htim1.Instance->CNT = 100;
  encoder_last_cnt = 100;

  HAL_Delay(20);
  uint8_t codehyz[10] = "23211319";
  uint8_t codelyk[10] = "23211326";
  uint8_t codecyh[10] = "23211288";
  OLED_Clear();
  OLED_Refresh();
  OLED_CN(28, 6, 6, 12, 1, 1);
  OLED_CN(0, 20, 4, 12, 1, 2);
  OLED_ShowString(50, 20, codehyz, 12, 1);
  OLED_CN(0, 36, 4, 12, 1, 3);
  OLED_ShowString(50, 36, codelyk, 12, 1);
  OLED_CN(0, 52, 4, 12, 1, 4);
  OLED_ShowString(50, 52, codecyh, 12, 1);
  OLED_Refresh();
  HAL_Delay(3000);
  OLED_Clear();

  // 初始化 INA260 引用和参考: https://github.com/xupenghu/ina260
  int ret = ina260_init_default(INA260_SLAVE_ADDRESS);

  OLED_Clear();
  OLED_CN(28, 6, 6, 12, 1, 1);
  if (ret == INA_STATUS_OK) {
    OLED_ShowString(0, 24, (uint8_t *)"INA260 OK", 16, 1);

    // 配置工作模式
    // operating_type: iotPower (default) - 同时测电流和电压
    // operating_mode: iomContinuous (default) - 连续测量
    // current_ctime: ictConvert1p1ms (default) - 电流转换时间 1.1ms
    // voltage_ctime: ivtConvert1p1ms (default) - 电压转换时间 1.1ms
    // sample_size: issSample512 - 512次采样取平均值
    // 总采样时间 = 采样次数 * 转换时间
    ina260_set_config(iotPower, iomContinuous, ictConvert1p1ms, ictConvert1p1ms,
                      issSample512);

  } else {
    OLED_ShowString(0, 24, (uint8_t *)"INA260 ERR", 16, 1);
  }
  OLED_Refresh();
  HAL_Delay(2000);

  // LED7显示
  HAL_GPIO_WritePin(a7led_GPIO_Port, a7led_Pin, GPIO_PIN_SET);
  // uint8_t mes[30] = "";

  SystemCoreClockUpdate(); // 确保 SystemCoreClock 正确
  DWT_Delay_Init();        // 微秒档位初始化 delay.c

  OLED_Clear();
  OLED_CN(28, 6, 6, 12, 1, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // 获取当前时间戳
    uint32_t current_time = HAL_GetTick();

    // 高频率扫描键盘
    KEY_Scan();

    if (Key_IsPressed()) {
      //
      uint16_t key_val = Key_Read();
      char key = (char)key_val;
      // 处理按键输入
      Process_Key_Input(key);
      // 清除按键状态
      Key_Clear();
    }
    // 处理旋转编码器 - 在电压/电流调节模式时生效
    if (input_state == INPUT_VOLTAGE || input_state == INPUT_CURRENT) {
      Process_Encoder();
    }
    // 定时更新 INA260 和 OLED 显示
    // while是 20ms 循环一次，等待 1s 更新一次OLED显示
    if ((current_time - last_oled_update) >= OLED_UPDATE_INTERVAL) {
      last_oled_update = current_time;

      // 读取 INA260 数据
      float voltage_mv = 0;
      float current_ma = 0;
      float power_mw = 0;
      char buffer[32];

      // 读取并显示电压
      if (ina260_get_voltage(&voltage_mv) == INA_STATUS_OK) {
        if (voltage_mv > 1000) {
          sprintf(buffer, "SV:%.2fV V:%.2fV   ", SetVoltage,
                  voltage_mv / 1000.0f);
        } else {
          sprintf(buffer, "SV:%.2fV V:%.0fmV   ", SetVoltage, voltage_mv);
        }
        OLED_ShowString(0, 18, (uint8_t *)buffer, 12, 1);
      }

      // 读取并显示电流
      if (ina260_get_current(&current_ma) == INA_STATUS_OK) {
        if (fabs(current_ma) > 1000) {
          sprintf(buffer, "SI:%.2fA I:%.2fA   ", SetCurrent,
                  current_ma / 1000.0f);
        } else {
          sprintf(buffer, "SI:%.2fA I:%.0fmA   ", SetCurrent, current_ma);
        }
        OLED_ShowString(0, 18 + 14, (uint8_t *)buffer, 12, 1);
      }

      // 计算并显示功率
      power_mw = (voltage_mv * current_ma) / 1000.0f;
      if (power_mw > 1000) {
        sprintf(buffer, " P:%.3fW    ", power_mw / 1000.0f);
      } else {
        sprintf(buffer, " P:%.3fmW    ", power_mw);
      }
      OLED_ShowString(0, 45, (uint8_t *)buffer, 8, 1);

      // 只在非输入状态清除提示行
      if (input_state == INPUT_IDLE) {
        OLED_ShowString(0, 54, (uint8_t *)"                          ", 8, 1);
      }

      OLED_Refresh();
      HAL_GPIO_TogglePin(a7led_GPIO_Port, a7led_Pin);
    }

    HAL_Delay(20);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
