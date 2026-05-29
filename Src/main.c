/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — ADC dual-channel with double buffering,
  *                   moving average filter, and outlier rejection.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdint.h>

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>   /* abs() */
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */

typedef struct __attribute__((packed)) {
  uint8_t  start_delimiter;
  uint16_t adc0;
  uint16_t adc1;
} uart_packet_t;

/* USER CODE END PTD */

/* USER CODE BEGIN PD */

/* Number of timer-triggered ADC pairs captured into the DMA ring.
   Must be even so the half/full callbacks split cleanly. */
#define SAMPLES        1000          /* total ADC *pairs* in DMA buffer        */
#define HALF           (SAMPLES / 2) /* samples per processing half            */

/* Moving-average sliding window (per channel) */
#define WINDOW_SIZE    3

/* Outlier gate: replace sample when |sample - mean| > mean * THRESHOLD / 100 */
#define OUTLIER_THRESHOLD  85

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* ── DMA write buffer ────────────────────────────────────────────────────────
   ADC is configured for 2 regular channels (CH6 rank-1, CH8 rank-2).
   Each TIM1 TRGO trigger causes one full scan → 2 uint16 values written.
   With SAMPLES=1000 pairs the DMA buffer holds 2000 uint16 words total.
   DMA circular mode fills it continuously; the half/full callbacks fire at
   index [0] and index [SAMPLES] respectively. */
volatile uint16_t dma_buffer[SAMPLES * 2]; /* interleaved: [adc0, adc1, adc0, adc1, ...] */

/* Flags set by ISR, cleared by main loop */
volatile int first_half_ready  = 0;
volatile int second_half_ready = 0;

/* ── Processing buffers (one set per channel, per half) ──────────────────── */
/* Channel 0 */
volatile uint16_t proc_buf_ch0_1[HALF], proc_buf_ch0_2[HALF];
volatile uint16_t out_buf_ch0_1[HALF],  out_buf_ch0_2[HALF];

/* Channel 1 */
volatile uint16_t proc_buf_ch1_1[HALF], proc_buf_ch1_2[HALF];
volatile uint16_t out_buf_ch1_1[HALF],  out_buf_ch1_2[HALF];

/* Moving-average state (reset before each half) */
volatile uint16_t window_ch0[WINDOW_SIZE];
volatile uint16_t window_ch1[WINDOW_SIZE];
volatile uint32_t sum_ch0 = 0, sum_ch1 = 0;
volatile int      win_idx = 0;

/* UART */
volatile uint8_t uart_tx_busy = 0;

uart_packet_t uart_tx = {
  .start_delimiter = 0xAA,
  .adc0 = 0,
  .adc1 = 0
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void process_half(
    const volatile uint16_t *raw_interleaved, /* pointer into dma_buffer        */
    volatile uint16_t *proc_ch0,              /* deinterleaved + shift staging  */
    volatile uint16_t *proc_ch1,
    volatile uint16_t *out_ch0,               /* filtered output                */
    volatile uint16_t *out_ch1);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/**
 * @brief  Run the full processing pipeline on one half-buffer.
 *
 * Steps per channel:
 *   1. De-interleave from DMA buffer
 *   2. Left-shift << 4  (align 12-bit result to MSB of uint16)
 *   3. Warm-up window fill with simple cumulative average
 *   4. For remaining samples:
 *        a. Outlier rejection  (replace spike with current window mean)
 *        b. Moving average update
 */
static void process_half(
    const volatile uint16_t *raw,
    volatile uint16_t *proc_ch0, volatile uint16_t *proc_ch1,
    volatile uint16_t *out_ch0,  volatile uint16_t *out_ch1)
{
  /* Reset moving-average state */
  sum_ch0 = 0; sum_ch1 = 0; win_idx = 0;

  /* ── 1. De-interleave --------------------------------------------------- */
  for (int i = 0; i < HALF; i++) {
    proc_ch0[i] = raw[i * 2];       /* even words = CH6  */
    proc_ch1[i] = raw[i * 2 + 1];   /* odd  words = CH8  */
  }

  /* ── 2–4. Shift + warm-up window --------------------------------------- */
  for (int i = 0; i < WINDOW_SIZE; i++) {
    proc_ch0[i] = proc_ch0[i] << 4;
    proc_ch1[i] = proc_ch1[i] << 4;

    window_ch0[i] = proc_ch0[i];
    window_ch1[i] = proc_ch1[i];

    sum_ch0 += window_ch0[i];
    sum_ch1 += window_ch1[i];

    /* Simple cumulative average during warm-up */
    out_ch0[i] = (uint16_t)(sum_ch0 / (i + 1));
    out_ch1[i] = (uint16_t)(sum_ch1 / (i + 1));
  }

  /* ── 3–4. Steady-state: outlier rejection + moving average -------------- */
  for (int i = WINDOW_SIZE; i < HALF; i++) {
    proc_ch0[i] = proc_ch0[i] << 4;
    proc_ch1[i] = proc_ch1[i] << 4;

    uint16_t s0 = proc_ch0[i];
    uint16_t s1 = proc_ch1[i];

    uint16_t mean0 = (uint16_t)(sum_ch0 / WINDOW_SIZE);
    uint16_t mean1 = (uint16_t)(sum_ch1 / WINDOW_SIZE);

    /* Outlier gate — replace spike with current window mean */
    uint16_t thr0 = (uint16_t)((mean0 * OUTLIER_THRESHOLD) / 100);
    uint16_t thr1 = (uint16_t)((mean1 * OUTLIER_THRESHOLD) / 100);

    if (abs((int)s0 - (int)mean0) > thr0) s0 = mean0;
    if (abs((int)s1 - (int)mean1) > thr1) s1 = mean1;

    /* Sliding window update */
    sum_ch0 -= window_ch0[win_idx];
    sum_ch1 -= window_ch1[win_idx];

    window_ch0[win_idx] = s0;
    window_ch1[win_idx] = s1;

    sum_ch0 += s0;
    sum_ch1 += s1;

    out_ch0[i] = (uint16_t)(sum_ch0 / WINDOW_SIZE);
    out_ch1[i] = (uint16_t)(sum_ch1 / WINDOW_SIZE);

    win_idx = (win_idx + 1 == WINDOW_SIZE) ? 0 : win_idx + 1;
  }
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

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_TIM_Base_Start(&htim1);

  /* Start DMA circular capture — 2 channels × SAMPLES triggers = 2*SAMPLES words */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_buffer, SAMPLES * 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ── First half ready ------------------------------------------------- */
    if (first_half_ready) {
      first_half_ready = 0;

      process_half(
        &dma_buffer[0],          /* first HALF pairs start at index 0 */
        proc_buf_ch0_1, proc_buf_ch1_1,
        out_buf_ch0_1,  out_buf_ch1_1);

      /* Transmit one representative sample per processed half.
         Extend to a loop / DMA scatter if you need all HALF samples. */
      if (!uart_tx_busy) {
        uart_tx.adc0 = out_buf_ch0_1[HALF - 1];  /* last filtered sample */
        uart_tx.adc1 = out_buf_ch1_1[HALF - 1];
        uart_tx_busy = 1;
        HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&uart_tx, sizeof(uart_tx));
      }
    }

    /* ── Second half ready ------------------------------------------------ */
    if (second_half_ready) {
      second_half_ready = 0;

      process_half(
        &dma_buffer[HALF * 2],   /* second HALF pairs start at index HALF*2 */
        proc_buf_ch0_2, proc_buf_ch1_2,
        out_buf_ch0_2,  out_buf_ch1_2);

      if (!uart_tx_busy) {
        uart_tx.adc0 = out_buf_ch0_2[HALF - 1];
        uart_tx.adc1 = out_buf_ch1_2[HALF - 1];
        uart_tx_busy = 1;
        HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&uart_tx, sizeof(uart_tx));
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    Error_Handler();

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState            = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 1;
  RCC_OscInitStruct.PLL.PLLN            = 16;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    Error_Handler();

  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler         = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution             = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign              = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode           = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection           = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait       = DISABLE;
  hadc1.Init.ContinuousConvMode     = DISABLE;
  hadc1.Init.NbrOfConversion        = 2;
  hadc1.Init.DiscontinuousConvMode  = DISABLE;
  hadc1.Init.ExternalTrigConv       = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge   = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests  = ENABLE;
  hadc1.Init.Overrun                = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode       = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    Error_Handler();

  sConfig.Channel      = ADC_CHANNEL_6;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();

  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank    = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();
}

/**
  * @brief TIM1 Initialization Function
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};

  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 0;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = 3200;
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    Error_Handler();

  sMasterConfig.MasterOutputTrigger  = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    Error_Handler();
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance            = USART2;
  huart2.Init.BaudRate       = 115200;
  huart2.Init.WordLength     = UART_WORDLENGTH_8B;
  huart2.Init.StopBits       = UART_STOPBITS_1;
  huart2.Init.Parity         = UART_PARITY_NONE;
  huart2.Init.Mode           = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling   = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
    Error_Handler();
}

/**
  * @brief Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin   = LD3_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/**
 * @brief  ADC DMA half-complete — first 500 pairs are ready.
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
  first_half_ready = 1;
}

/**
 * @brief  ADC DMA complete — second 500 pairs are ready.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
  second_half_ready = 1;
}

/**
 * @brief  UART DMA TX complete — bus is free for next packet.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
    uart_tx_busy = 0;
}

/* USER CODE END 4 */

/**
  * @brief  Error handler
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif