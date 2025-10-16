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
#include "adc.h"
#include "gpio.h"
#include "i2c.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "ina260.h"
#include "oled.h"
#include "stdio.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// void MX_TIM1_Init(void) { htim1.Init.Prescaler = 144 - 1; }
// 已经通过CubeMX在tim.c修改

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // u8 t = ' ';
  delay_init();
  HAL_Delay(30); // for OLED screen initial
  OLED_Init();
  OLED_ColorTurn(0);   // 0正常显示，1 反色显示
  OLED_DisplayTurn(1); // 0正常显示 1 屏幕翻转显示
  // 取自中景园电子ZJY096I0400WG11技术资料(SSD1315)
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
  if (ret == INA_STATUS_OK) {
    OLED_ShowString(0, 0, (uint8_t *)"INA260 OK", 16, 1);

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
    OLED_ShowString(0, 0, (uint8_t *)"INA260 ERR", 16, 1);
  }
  OLED_Refresh();
  HAL_Delay(2000);

  // LED7显示
  HAL_GPIO_WritePin(a7led_GPIO_Port, a7led_Pin, GPIO_PIN_SET);
  // uint8_t mes[30] = "";

  SystemCoreClockUpdate(); // 确保 SystemCoreClock 正确
  DWT_Delay_Init();        // 微秒档位初始化 delay.c

  OLED_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // 读取 INA260 数据 引用和参考: https://github.com/xupenghu/ina260
    float voltage_mv = 0;
    float current_ma = 0;
    float power_mw = 0;
    char buffer[32];

    // 读取并显示电压
    if (ina260_get_voltage(&voltage_mv) == INA_STATUS_OK) {
      if (voltage_mv > 1000) {
        sprintf(buffer, " V:%.2fV    ", voltage_mv / 1000.0f); // 大于1V转换为V
      } else {
        sprintf(buffer, " V:%.2fmV    ", voltage_mv);
      }
      OLED_ShowString(0, 0, (uint8_t *)buffer, 16, 1);
    }

    // 读取并显示电流
    if (ina260_get_current(&current_ma) == INA_STATUS_OK) {
      if (fabs(current_ma) > 1000) { // INA260测量电流可能为负值，取绝对值判断
        sprintf(buffer, " I:%.2fA    ", current_ma / 1000.0f); // 大于1A转换为A
      } else {
        sprintf(buffer, " I:%.2fmA    ", current_ma);
      }
      OLED_ShowString(0, 18, (uint8_t *)buffer, 16, 1);
    }

    // 读取并显示功率
    // if (ina260_get_power(&power_mw) == INA_STATUS_OK) {
    //   sprintf(buffer, " P:%.2fmW   ", power_mw);
    //   OLED_ShowString(0, 36, (uint8_t *)buffer, 16, 1);
    // }

    // 更高位数计算功率
    power_mw = (voltage_mv * current_ma) / 1000.0f; // mW
    if (power_mw > 1000) {
      sprintf(buffer, " P:%.3fW    ", power_mw / 1000.0f); // 大于1W转换为W
    } else {
      sprintf(buffer, " P:%.3fmW    ", power_mw);
    }
    OLED_ShowString(0, 36, (uint8_t *)buffer, 16, 1);

    OLED_Refresh();
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(a7led_GPIO_Port, a7led_Pin);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
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
