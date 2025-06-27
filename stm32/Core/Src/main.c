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
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_slave.h"
#include "status.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_TX_BUF_SIZE 32
#define I2C_RX_BUF_SIZE 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define GPIO_SET(GPIOx, PinMask) ((GPIOx)->BSRR = (PinMask))
#define GPIO_CLR(GPIOx, PinMask) ((GPIOx)->BRR = (PinMask))
// #define GPIO_CLR(GPIOx, PinMask) ((GPIOx)->BSRR = ((uint32_t)(PinMask) <<
// 16))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void update_LED(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int rx_busy_counter = 0;

volatile uint8_t  last_error       = 0;
volatile uint16_t led_mode_mask    = 0;
volatile uint16_t custom_leds_data = 0;               // custom led value
volatile uint16_t threshold_flag   = 0;               // threshold result
volatile uint16_t thresholds[16]   = {0};             // led threshold
volatile uint16_t leds_data = 0, last_leds_data = 0;  // led value
volatile uint16_t mux_data[16] = {0};                 // adc value

volatile uint8_t  mux_index      = 0;  // current mux index
volatile uint32_t adc_dma_buffer = 0;  // current adc value

static uint32_t         last_update_time = 0;
static volatile uint8_t old_mux_index    = 0;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_ADC_Start_DMA(&hadc, (uint32_t *)&adc_dma_buffer, 1) != HAL_OK) {
    Error_Handler();
  }
  if (I2C_begin() != HAL_OK) {
    Error_Handler();
  }

  update_LED();
  set_default_status();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_update_time >= 1) {
      if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_RX_LISTEN) {
        rx_busy_counter++;

        if (rx_busy_counter > I2C_RX_BUSY_CNTR) {
          HAL_I2C_DisableListen_IT(&hi2c1);
          HAL_I2C_DeInit(&hi2c1);
          HAL_I2C_Init(&hi2c1);
          HAL_I2C_EnableListen_IT(&hi2c1);
          rx_busy_counter = 0;
        }
      } else rx_busy_counter = 0;

      if (last_leds_data != leds_data) {
        update_LED();
      }
    }

    if (old_mux_index == mux_index) {
      mux_index = (mux_index + 1) % 16;

      // eq: GPIOA->BSRR = (0x0f << (16 + 1)) | ((mux_index & 0x0f) << 1);
      GPIOA->BSRR = 0x1e0000 | ((mux_index & 0x0f) << 1);

      for (int i = 0; i < 10; i++) __NOP();

      HAL_ADC_Start_DMA(&hadc, (uint32_t *)&adc_dma_buffer, 1);
    }

    update_status();
    HAL_IWDG_Refresh(&hiwdg);
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
  RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType        = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14 | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState              = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State            = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue   = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState              = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState          = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource         = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL            = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV            = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    uint16_t value      = adc_dma_buffer;
    mux_data[mux_index] = value;

    uint16_t mask = (1 << mux_index);
    if (value > thresholds[mux_index]) threshold_flag |= mask;
    else threshold_flag &= ~mask;

    if (led_mode_mask & mask) {
      if (custom_leds_data & mask) leds_data |= mask;
      else leds_data &= ~mask;
    } else {
      if (threshold_flag & mask) leds_data |= mask;
      else leds_data &= ~mask;
    }

    old_mux_index = mux_index;
  }
}

/** to display leds (HC595) */
void update_LED() {
  GPIO_CLR(L_CK_GPIO_Port, L_CK_Pin);
  for (uint8_t i = 0; i < 16; i++) {
    if (leds_data & (1 << i)) GPIO_SET(LEDS_GPIO_Port, LEDS_Pin);
    else GPIO_CLR(LEDS_GPIO_Port, LEDS_Pin);

    GPIO_SET(D_CK_GPIO_Port, D_CK_Pin);
    GPIO_CLR(D_CK_GPIO_Port, D_CK_Pin);
  }
  GPIO_SET(L_CK_GPIO_Port, L_CK_Pin);
  GPIO_CLR(L_CK_GPIO_Port, L_CK_Pin);

  last_leds_data = leds_data;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();  // disable all interrupts

  set_status(0, STATUS_MAX_TIME);  // keep led on
  update_status();

  NVIC_SystemReset();

  // if reset not work
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
