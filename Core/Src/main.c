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
#include "i2c.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Key4x4.h"
#include "dac60501.h"
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
float SetCurrent = 1.00f; // 设定电流 (A)

// 旋转编码器相关变量
uint16_t V_encoder_last_cnt = 1000;        // 上次电压编码器计数值
uint16_t I_encoder_last_cnt = 1000;        // 上次电流编码器计数值
const float ENCODER_STEP_FINE = 0.01f;     // 编码器精调步进
const float ENCODER_STEP_COARSE_V = 0.50f; // 电压粗调步进
const float ENCODER_STEP_COARSE_I = 0.10f; // 电流粗调步进
uint8_t encoder_v_active = 0;              // 电压编码器激活标志
uint8_t encoder_i_active = 0;              // 电流编码器激活标志
uint32_t encoder_v_last_time = 0;          // 电压编码器上次活动时间
uint32_t encoder_i_last_time = 0;          // 电流编码器上次活动时间
const uint32_t ENCODER_TIMEOUT = 2000;     // 编码器提示超时时间(ms)

// 粗调/精调模式标志
uint8_t voltage_coarse_mode = 0; // 电压粗调模式标志
uint8_t current_coarse_mode = 0; // 电流粗调模式标志

// DAC相关常量
// const float DAC_VREF = 2.5f;     // DAC内部参考电压 2.5V
// const float DAC_VOUT_MAX = 2.5f; // DAC最大输出电压 2.5V

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Process_Key_Input(char key);
static void Clear_Input_Buffer(void);
static void Display_Input_Prompt(void);
static uint8_t Validate_And_Set_Value(void);
static void Process_Encoder(void);
static int Update_DAC_Outputs(void); // 添加函数声明
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
}

/**
 * @brief  显示输入提示
 */
static void Display_Input_Prompt(void) {
  char prompt[32];
  uint32_t current_time = HAL_GetTick();

  // 优先显示键盘输入状态
  if (input_state == INPUT_VOLTAGE) {
    sprintf(prompt, "SetV:%s#          ", input_buffer);
    OLED_ShowString(0, 54, (uint8_t *)prompt, 8, 1);
  } else if (input_state == INPUT_CURRENT) {
    sprintf(prompt, "SetI:%s#          ", input_buffer);
    OLED_ShowString(0, 54, (uint8_t *)prompt, 8, 1);
  }
  // 显示编码器调节状态（如果有激活）
  else {
    // 检查电压编码器是否在超时时间内活动
    if (encoder_v_active &&
        (current_time - encoder_v_last_time) < ENCODER_TIMEOUT) {
      // 显示粗调/精调模式
      if (voltage_coarse_mode) {
        sprintf(prompt, "[COARSE] V:%.2fV  ", SetVoltage);
      } else {
        sprintf(prompt, "[FINE] V:%.2fV    ", SetVoltage);
      }
      OLED_ShowString(0, 54, (uint8_t *)prompt, 8, 1);
    }
    // 检查电流编码器是否在超时时间内活动
    else if (encoder_i_active &&
             (current_time - encoder_i_last_time) < ENCODER_TIMEOUT) {
      // 显示粗调/精调模式
      if (current_coarse_mode) {
        sprintf(prompt, "[COARSE] I:%.2fA  ", SetCurrent);
      } else {
        sprintf(prompt, "[FINE] I:%.2fA    ", SetCurrent);
      }
      OLED_ShowString(0, 54, (uint8_t *)prompt, 8, 1);
    }
    // 超时后清除提示
    else {
      encoder_v_active = 0;
      encoder_i_active = 0;
      // 不在输入状态时清空提示行
      if (input_state == INPUT_IDLE) {
        OLED_ShowString(0, 54, (uint8_t *)"                          ", 8, 1);
      }
    }
  }

  OLED_Refresh();
}

/**
 * @brief  处理旋转编码器输入（随时可调）
 */
static void Process_Encoder(void) {
  uint32_t current_time = HAL_GetTick();

  // ========== 检测电压编码器按钮状态（粗调/精调切换）==========
  // 低电平 = 按下 = 粗调模式
  if (HAL_GPIO_ReadPin(Coarse_Detector_V_GPIO_Port, Coarse_Detector_V_Pin) ==
      GPIO_PIN_RESET) {
    voltage_coarse_mode = 1;
  } else {
    voltage_coarse_mode = 0;
  }

  // ========== 检测电流编码器按钮状态（粗调/精调切换）==========
  // 低电平 = 按下 = 粗调模式
  if (HAL_GPIO_ReadPin(Coarse_Detector_I_GPIO_Port, Coarse_Detector_I_Pin) ==
      GPIO_PIN_RESET) {
    current_coarse_mode = 1;
  } else {
    current_coarse_mode = 0;
  }

  // ========== 处理电压编码器 (TIM1) ==========
  uint16_t V_encoder_cnt = __HAL_TIM_GET_COUNTER(&htim1);
  int16_t V_cnt_diff =
      ((int16_t)V_encoder_cnt - (int16_t)V_encoder_last_cnt) / 2;

  // 限制单次调整幅度,防止异常跳变
  if (abs(V_cnt_diff) > 0 && abs(V_cnt_diff) <= 100) {
    // 根据粗调/精调模式选择步进值
    float v_step =
        voltage_coarse_mode ? ENCODER_STEP_COARSE_V : ENCODER_STEP_FINE;

    // 计算新电压值
    float new_voltage = SetVoltage + V_cnt_diff * v_step;

    // 限制范围 0.00 - MAX_VOLTAGE
    if (new_voltage < 0.0f) {
      SetVoltage = 0.0f;
    } else if (new_voltage > MAX_VOLTAGE) {
      SetVoltage = MAX_VOLTAGE;
    } else {
      SetVoltage = new_voltage;
    }

    // 标记电压编码器活动
    encoder_v_active = 1;
    encoder_v_last_time = current_time;
    encoder_i_active = 0; // 清除电流编码器提示

    // 更新上次计数值
    V_encoder_last_cnt = V_encoder_cnt;
  }

  // ========== 处理电流编码器 (TIM4) ==========
  uint16_t I_encoder_cnt = __HAL_TIM_GET_COUNTER(&htim4);
  int16_t I_cnt_diff =
      ((int16_t)I_encoder_cnt - (int16_t)I_encoder_last_cnt) / 2;

  // 限制单次调整幅度,防止异常跳变
  if (abs(I_cnt_diff) > 0 && abs(I_cnt_diff) <= 100) {
    // 根据粗调/精调模式选择步进值
    float i_step =
        current_coarse_mode ? ENCODER_STEP_COARSE_I : ENCODER_STEP_FINE;

    // 计算新电流值
    float new_current = SetCurrent + I_cnt_diff * i_step;

    // 限制范围 0.00 - MAX_CURRENT
    if (new_current < 0.0f) {
      SetCurrent = 0.0f;
    } else if (new_current > MAX_CURRENT) {
      SetCurrent = MAX_CURRENT;
    } else {
      SetCurrent = new_current;
    }

    // 标记电流编码器活动
    encoder_i_active = 1;
    encoder_i_last_time = current_time;
    encoder_v_active = 0; // 清除电压编码器提示

    // 更新上次计数值
    I_encoder_last_cnt = I_encoder_cnt;
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
      sprintf(msg, "  Iset ERR:0-1A     ");
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
      encoder_v_active = 0; // 清除编码器状态
      encoder_i_active = 0;
      Display_Input_Prompt();
    } else if (key == 'B') {
      // 开始输入电流
      input_state = INPUT_CURRENT;
      Clear_Input_Buffer();
      encoder_v_active = 0; // 清除编码器状态
      encoder_i_active = 0;
      Display_Input_Prompt();
    } else if (key == 'C') {
      // 取消/清除设定值 Clear
      SetVoltage = 0.0f;
      SetCurrent = 0.0f;
      OLED_ShowString(0, 54, (uint8_t *)"  --- Clear OK ---       ", 8, 1);
      OLED_Refresh();
      HAL_Delay(1000);
    }
    break;

    // 如果在输入电压或电流状态按下时候:
  case INPUT_VOLTAGE:
  case INPUT_CURRENT:
    if (key == 'E') {
      if (input_index > 0) {
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
      // 取消提示
      OLED_ShowString(0, 54, (uint8_t *)"  --- Cancel ---       ", 8, 1);
      OLED_Refresh();
      HAL_Delay(1000);

    } else if ((key >= '0' && key <= '9') || key == '.') {
      if (input_index < 8) {
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

/**
 * @brief  更新所有DAC输出(电压和电流)
 * @retval 0: 全部成功, -1: 电压DAC失败, -2: 电流DAC失败, -3: 全部失败
 */
static int Update_DAC_Outputs(void) {
  int ret_v = DAC60501_SetVoltageOutput(SetVoltage);
  int ret_i = DAC60501_SetCurrentOutput(SetCurrent);

  // 返回错误码
  if (ret_v != 0 && ret_i != 0)
    return -3; // 全部失败
  if (ret_v != 0)
    return -1; // 电压DAC失败
  if (ret_i != 0)
    return -2; // 电流DAC失败
  return 0;    // 全部成功
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // u8 t = ' ';
  delay_init();
  HAL_Delay(30); // for OLED screen initial
  OLED_Init();
  OLED_ColorTurn(0);   // 0正常显示，1 反色显示
  OLED_DisplayTurn(1); // 0正常显示 1 屏幕翻转显示
  // OLED部分代码参考取自中景园电子ZJY096I0400WG11技术资料(SSD1315)
  KEY_4x4_Init();                                 // 初始化4x4矩阵键盘
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // 启动电压编码器
  htim1.Instance->CNT = 1000;
  V_encoder_last_cnt = 1000;

  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 启动电流编码器
  htim4.Instance->CNT = 1000;
  I_encoder_last_cnt = 1000;

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
  HAL_Delay(1500);

  // ============ DAC60501 开机自检 ============
  OLED_Clear();
  OLED_CN(28, 6, 6, 12, 1, 1); // 显示标题
  OLED_ShowString(0, 20, (uint8_t *)"DAC60501 Diagnostic", 8, 1);
  OLED_Refresh();
  HAL_Delay(1000);
  
  //PB9 置高
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, CC_LED_Pin, GPIO_PIN_SET); // CC LED ON   
  HAL_GPIO_WritePin(GPIOA, CV_LED_Pin, GPIO_PIN_SET); // CV LED ON
  char dac_msg[32];
  uint8_t diag_line = 30; // 诊断信息起始行

  // ========== 步骤1: 初始化所有DAC ==========
  OLED_ShowString(0, diag_line, (uint8_t *)"1.Init DACs...       ", 8, 1);
  OLED_Refresh();
  HAL_Delay(300);

  DAC60501_Init_All(); // 初始化两个DAC
  HAL_Delay(1000);

  OLED_ShowString(0, diag_line, (uint8_t *)"1.Init DACs...OK       ", 8, 1);
  OLED_Refresh();
  HAL_Delay(500);

  // ========== 步骤2: 读取设备ID ==========
  // ========== 检查电压控制DAC (0x48) ==========
  diag_line += 10;
  OLED_ShowString(0, diag_line, (uint8_t *)"2.Voltage DAC...       ", 8, 1);
  OLED_Refresh();
  HAL_Delay(300);

  uint16_t dac_v_id = DAC60501_ReadDeviceId_Addr(DAC_VOLTAGE_ADDR);
  if (dac_v_id != 0xFFFF && dac_v_id != 0x0000) {
    sprintf(dac_msg, "2.V-DAC:0x%04X OK       ", dac_v_id);
  } else {
    sprintf(dac_msg, "2.V-DAC:0x%04X ERR       ", dac_v_id);
  }
  OLED_ShowString(0, diag_line, (uint8_t *)dac_msg, 8, 1);
  OLED_Refresh();
  HAL_Delay(500);

  // ========== 检查电流控制DAC (0x49) ==========
  diag_line += 10;
  OLED_ShowString(0, diag_line, (uint8_t *)"3.Current DAC...       ", 8, 1);
  OLED_Refresh();
  HAL_Delay(300);

  uint16_t dac_i_id = DAC60501_ReadDeviceId_Addr(DAC_CURRENT_ADDR);
  if (dac_i_id != 0xFFFF && dac_i_id != 0x0000) {
    sprintf(dac_msg, "3.I-DAC:0x%04X OK       ", dac_i_id);
  } else {
    sprintf(dac_msg, "3.I-DAC:0x%04X ERR       ", dac_i_id);
  }
  OLED_ShowString(0, diag_line, (uint8_t *)dac_msg, 8, 1);
  OLED_Refresh();
  HAL_Delay(2000);
  // ========== 下一页 ==========
  OLED_Clear();
  diag_line = 0;
  HAL_Delay(500);
  // ========== 步骤3: 读取STATUS寄存器 ==========
  diag_line += 10;
  OLED_ShowString(0, diag_line, (uint8_t *)"3.Read STATUS...       ", 8, 1);
  OLED_Refresh();
  HAL_Delay(300);

  // 修改: 读取两个DAC的STATUS
  uint16_t status_v = DAC60501_ReadStatus_Addr(DAC_VOLTAGE_ADDR);
  sprintf(dac_msg, "3.V-STA:0x%04X       ", status_v);
  OLED_ShowString(0, diag_line, (uint8_t *)dac_msg, 8, 1);
  OLED_Refresh();
  HAL_Delay(500);
  // ========== 下一页 ==========
  OLED_Clear();
  diag_line = 0;
  HAL_Delay(500);
  // ========== 步骤4: 设置GAIN寄存器 ==========
  diag_line += 10;
  OLED_ShowString(0, diag_line, (uint8_t *)"4.Set GAIN...       ", 8, 1);
  OLED_Refresh();
  HAL_Delay(300);

  // 修改: 设置两个DAC的GAIN
  int gain_ret_v = DAC60501_SetGain_Addr(DAC_VOLTAGE_ADDR, 1, 1);
  int gain_ret_i = DAC60501_SetGain_Addr(DAC_CURRENT_ADDR, 1, 1);

  if (gain_ret_v == 0 && gain_ret_i == 0) {
    OLED_ShowString(0, diag_line, (uint8_t *)"4.GAIN Set OK       ", 8, 1);
  } else {
    sprintf(dac_msg, "4.GAIN Err V:%d I:%d", gain_ret_v, gain_ret_i);
    OLED_ShowString(0, diag_line, (uint8_t *)dac_msg, 8, 1);
  }
  OLED_Refresh();
  HAL_Delay(500);

  // ========== 步骤5: 检查REF-ALARM ==========
  diag_line += 10;
  OLED_ShowString(0, diag_line, (uint8_t *)"5.Check REF...       ", 8, 1);
  OLED_Refresh();
  HAL_Delay(300);

  // 修改: 检查两个DAC的REF-ALARM
  int alarm_v = DAC60501_RefAlarm_Addr(DAC_VOLTAGE_ADDR);
  int alarm_i = DAC60501_RefAlarm_Addr(DAC_CURRENT_ADDR);

  if (alarm_v == 0 && alarm_i == 0) {
    OLED_ShowString(0, diag_line, (uint8_t *)"5.REF OK       ", 8, 1);
  } else {
    sprintf(dac_msg, "5.REF V:%d I:%d       ", alarm_v, alarm_i);
    OLED_ShowString(0, diag_line, (uint8_t *)dac_msg, 8, 1);
  }
  OLED_Refresh();
  HAL_Delay(500);
  // ========== 下一页 ==========
  OLED_Clear();
  diag_line = 0;
  HAL_Delay(500);
  // ========== 步骤6: 测试电压输出(增强版) ==========
  diag_line += 10;
  OLED_ShowString(0, diag_line, (uint8_t *)"6.Test Output...", 8, 1);
  OLED_Refresh();
  HAL_Delay(500);

  // 先读取CONFIG寄存器
  I2C_WriteByte_Addr(DAC_VOLTAGE_ADDR, CONFIG, 0x0000); // 复位V CONFIG寄存器
  I2C_WriteByte_Addr(DAC_CURRENT_ADDR, CONFIG, 0x0000); // 复位I CONFIG寄存器
  uint16_t V_config_reg = I2C_ReadByte_Addr(DAC_VOLTAGE_ADDR, CONFIG);
  uint16_t I_config_reg = I2C_ReadByte_Addr(DAC_CURRENT_ADDR, CONFIG);
  sprintf(dac_msg, "CFG:V-%04X,I-%04X", V_config_reg, I_config_reg);
  OLED_ShowString(0, diag_line + 10, (uint8_t *)dac_msg, 8, 1);
  OLED_Refresh();
  HAL_Delay(1000);

  // 清屏,开始详细测试
  OLED_Clear();
  diag_line = 18;

  // // 测试序列: 0V -> 1.25V -> 2.5V
  // float test_voltages[] = {0.0f, 1.25f, 2.5f};
  // uint8_t test_passed = 1;

  // for (int i = 0; i < 3; i++) {
  //   // 显示当前测试电压
  //   sprintf(dac_msg, "Test %d: %.2fV", i + 1, test_voltages[i]);
  //   OLED_ShowString(0, diag_line, (uint8_t *)dac_msg, 8, 1);
  //   OLED_Refresh();
  //   HAL_Delay(300);

  //   // 计算DAC码值
  //   uint16_t dac_code = (uint16_t)((test_voltages[i] / DAC_VREF) * 4095.0f);
  //   uint16_t dac_data = dac_code << 4; // 左对齐到16-bit

  //   // 写入DAC_DATA寄存器
  //   I2C_WriteByte(DAC_DATA, dac_data);
  //   HAL_Delay(100); // 等待DAC稳定

  //   // 回读DAC_DATA寄存器验证
  //   uint16_t readback = I2C_ReadByte(DAC_DATA);
  //   sprintf(dac_msg, "W:0x%04X R:0x%04X", dac_data, readback);
  //   OLED_ShowString(0, diag_line + 10, (uint8_t *)dac_msg, 8, 1);
  //   OLED_Refresh();

  //   // 验证写入是否成功
  //   if (readback != dac_data) {
  //     test_passed = 0;
  //     sprintf(dac_msg, "Readback FAIL!");
  //     OLED_ShowString(0, diag_line + 20, (uint8_t *)dac_msg, 8, 1);
  //     OLED_Refresh();
  //     HAL_Delay(2000);
  //     break;
  //   }

  //   // 再次检查STATUS寄存器
  //   uint16_t status_now = DAC60501_ReadStatus();
  //   sprintf(dac_msg, "STATUS:0x%04X", status_now);
  //   OLED_ShowString(0, diag_line + 20, (uint8_t *)dac_msg, 8, 1);
  //   OLED_Refresh();

  //   // 检查REF-ALARM位
  //   if (status_now & STATUS_REF_ALARM) {
  //     test_passed = 0;
  //     OLED_ShowString(0, diag_line + 30, (uint8_t *)"REF ALARM!", 8, 1);
  //     OLED_Refresh();
  //     HAL_Delay(2000);
  //     break;
  //   }

  //   // 显示"请用万用表测量"提示
  //   OLED_ShowString(0, diag_line + 30, (uint8_t *)"Measure VOUT now!", 8, 1);
  //   OLED_Refresh();
  //   HAL_Delay(5000); // 保持5秒供测量

  //   // 清除测试信息,准备下一个
  //   if (i < 2) {
  //     OLED_Clear();
  //     diag_line = 18;
  //   }
  // }

  // 恢复0V输出 - 修改为两个DAC
  DAC60501_SetVoltageOutput(0.0f);
  DAC60501_SetCurrentOutput(0.0f);

  // ========== 最终结果 ==========
  OLED_Clear();
  diag_line = 0;

  // ========== 额外诊断: 检查SYNC寄存器  ==========
  diag_line += 10;
  uint16_t sync_reg_v = I2C_ReadByte_Addr(DAC_VOLTAGE_ADDR, SYNC);
  uint16_t sync_reg_i = I2C_ReadByte_Addr(DAC_CURRENT_ADDR, SYNC);
  sprintf(dac_msg, "SYN-V:%04X-I:%04X", sync_reg_v, sync_reg_i);
  OLED_ShowString(0, diag_line, (uint8_t *)dac_msg, 8, 1);
  OLED_Refresh();
  HAL_Delay(1000);

  // ========== 额外诊断: 检查TRIGGER寄存器  ==========
  diag_line += 10;
  uint16_t trigger_reg_v = I2C_ReadByte_Addr(DAC_VOLTAGE_ADDR, TRIGGER);
  uint16_t trigger_reg_i = I2C_ReadByte_Addr(DAC_CURRENT_ADDR, TRIGGER);
  sprintf(dac_msg, "TRIG-V:%04X-I:%04X", trigger_reg_v, trigger_reg_i);
  OLED_ShowString(0, diag_line, (uint8_t *)dac_msg, 8, 1);
  OLED_Refresh();
  HAL_Delay(1000);

  // ========== 诊断总结 - 修改判断条件 ==========
  diag_line += 10;
  if (dac_v_id != 0xFFFF && dac_i_id != 0xFFFF && gain_ret_v == 0 &&
      gain_ret_i == 0 && alarm_v == 0 && alarm_i == 0) {
    OLED_ShowString(0, diag_line, (uint8_t *)"=> All Tests PASS", 8, 1);
  } else {
    OLED_ShowString(0, diag_line, (uint8_t *)"=> Tests FAILED!", 8, 1);
  }

  OLED_Refresh();
  HAL_Delay(5000); // 显示诊断结果5秒
  // ============ DAC诊断结束 ============

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

    // on或off按键控制 (非阻塞去抖动)
    static uint32_t last_switch_time = 0;
    static uint8_t switch_state = GPIO_PIN_SET;

    uint8_t current_switch = HAL_GPIO_ReadPin(on_off_switch_GPIO_Port, on_off_switch_Pin);
    if (current_switch != switch_state) {
      if (current_time - last_switch_time > 20) {  // 20ms debounce
        switch_state = current_switch;
        if (switch_state == GPIO_PIN_RESET) {
          HAL_GPIO_TogglePin(switch_vcc_GPIO_Port, switch_vcc_Pin);
        }
        last_switch_time = current_time;
      }
    }
    if (Key_IsPressed()) {
      uint16_t key_val = Key_Read();
      char key = (char)key_val;
      // 处理按键输入
      Process_Key_Input(key);
      // 清除按键状态
      Key_Clear();
    }

    // ========== 随时处理旋转编码器（无论任何状态）==========
    Process_Encoder();

    // 更新编码器提示显示（每50ms更新一次）
    static uint32_t last_encoder_display = 0;
    if ((current_time - last_encoder_display) >= 50) {
      last_encoder_display = current_time;
      if (encoder_v_active || encoder_i_active) {
        Display_Input_Prompt();
      }
    }

    // 旋转编码器激活显示指示
    char voltage_coarse_mode_prompt[4] = "";
    char current_coarse_mode_prompt[4] = "";

    if (encoder_v_active) {
      if (voltage_coarse_mode == 0) {
        sprintf(voltage_coarse_mode_prompt, "*");
      } else {
        sprintf(voltage_coarse_mode_prompt, "**");
      }
    }

    if (encoder_i_active) {
      if (current_coarse_mode == 0) {
        sprintf(current_coarse_mode_prompt, "*");
      } else {
        sprintf(current_coarse_mode_prompt, "**");
      }
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
          sprintf(buffer, "%sSV:%.2fV V:%.2fV   ", voltage_coarse_mode_prompt,
                  SetVoltage, voltage_mv / 1000.0f);
        } else {
          sprintf(buffer, "%sSV:%.2fV V:%.0fmV   ", voltage_coarse_mode_prompt,
                  SetVoltage, voltage_mv);
        }
        OLED_ShowString(0, 18, (uint8_t *)buffer, 12, 1);
      }

      // 读取并显示电流
      if (ina260_get_current(&current_ma) == INA_STATUS_OK) {
        if (fabs(current_ma) > 1000) {
          sprintf(buffer, "%sSI:%.2fA I:%.2fA   ", current_coarse_mode_prompt,
                  SetCurrent, current_ma / 1000.0f);
        } else {
          sprintf(buffer, "%sSI:%.2fA I:%.0fmA   ", current_coarse_mode_prompt,
                  SetCurrent, current_ma);
        }
        OLED_ShowString(0, 18 + 10, (uint8_t *)buffer, 12, 1);
      }

      // 计算并显示功率
      power_mw = (voltage_mv * current_ma) / 1000.0f;
      if (power_mw > 1000) {
        sprintf(buffer, "P:%.3fW              ", power_mw / 1000.0f);
      } else {
        sprintf(buffer, "P:%.3fmW              ", power_mw);
      }
      OLED_ShowString(0, 18 + 10 * 2, (uint8_t *)buffer, 12, 1);

      // 只在非输入状态且编码器未激活时清除提示行
      if (input_state == INPUT_IDLE && !encoder_v_active && !encoder_i_active) {
        OLED_ShowString(0, 54, (uint8_t *)"                          ", 8, 1);
      }

      // 更新DAC输出
      int dac_ret = Update_DAC_Outputs();
      if (dac_ret == -1) {
        OLED_ShowString(0, 54, (uint8_t *)"V-DAC ERR!       ", 8, 1);
      } else if (dac_ret == -2) {
        OLED_ShowString(0, 54, (uint8_t *)"I-DAC ERR!       ", 8, 1);
      } else if (dac_ret == -3) {
        OLED_ShowString(0, 54, (uint8_t *)"DACs ERR!       ", 8, 1);
      }
      // ======================================

      // ======================================

      OLED_Refresh();
      HAL_GPIO_TogglePin(a7led_GPIO_Port, a7led_Pin);
    }

    HAL_Delay(20);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
