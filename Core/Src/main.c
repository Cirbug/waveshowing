/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t armed;
  uint32_t last_cross_tick;
  uint32_t frequency_hz;
} AdcFreqMeter;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOFA_SEND_PERIOD_MS 2U
#define LCD_REFRESH_PERIOD_MS 100U
#define LCD_WAVE_POINTS 304U
#define LCD_WAVE_LEFT 8U
#define LCD_WAVE_WIDTH LCD_WAVE_POINTS
#define LCD_ADC1_TOP 80U
#define LCD_ADC2_TOP 168U
#define LCD_WAVE_HEIGHT 56U
#define ADC_MAX_CODE 4095U
#define TIM5_COUNTER_HZ 1000000U
#define FREQ_TIMEOUT_MS 1000U
#define ADC_FREQ_LOW_THRESHOLD 1900U
#define ADC_FREQ_HIGH_THRESHOLD 2200U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t adc1_value = 0U;
static uint16_t adc2_value = 0U;
static uint16_t lcd_adc1_wave[LCD_WAVE_POINTS];
static uint16_t lcd_adc2_wave[LCD_WAVE_POINTS];
static uint16_t lcd_wave_index = 0U;
static uint16_t lcd_wave_count = 0U;
static volatile uint32_t frequency_hz = 0U;
static volatile uint32_t frequency_last_capture_tick = 0U;
static uint32_t tim5_last_capture = 0U;
static uint8_t tim5_capture_ready = 0U;
static AdcFreqMeter adc1_freq_meter = {1U, 0U, 0U};
static AdcFreqMeter adc2_freq_meter = {1U, 0U, 0U};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void AdcDacVofa_Start(void);
static uint16_t ReadAdcValue(ADC_HandleTypeDef *hadc, uint16_t previous_value);
static uint32_t AdcFreq_Update(AdcFreqMeter *meter, uint16_t sample, uint32_t now);
static void UpdateDacOutputs(uint16_t value1, uint16_t value2);
static void Vofa_SendSamples(uint16_t value1, uint16_t value2, uint32_t pa0_freq, uint32_t adc1_freq, uint32_t adc2_freq);
static void LcdDisplay_Init(void);
static void LcdDisplay_Update(uint32_t now, uint16_t value1, uint16_t value2, uint32_t pa0_freq, uint32_t adc1_freq, uint32_t adc2_freq);
static void LcdPushSamples(uint16_t value1, uint16_t value2);
static void LcdDrawStatic(void);
static void LcdDrawValues(uint16_t value1, uint16_t value2, uint32_t pa0_freq, uint32_t adc1_freq, uint32_t adc2_freq);
static void LcdDrawFrequency(uint16_t x, uint16_t y, uint32_t freq);
static void LcdDrawWaveFrame(uint16_t top, const char *label, uint16_t color);
static void LcdDrawWaveform(uint16_t top, const uint16_t *samples, uint16_t color);
static uint16_t LcdAdcToY(uint16_t value, uint16_t top);
void App_Init(void);
void App_TaskStep(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_DAC_Init();
  MX_FSMC_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void App_Init(void)
{
  LcdDisplay_Init();
  AdcDacVofa_Start();

  if (HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

void App_TaskStep(void)
{
  static uint32_t last_vofa_tick = 0U;
  uint32_t now = HAL_GetTick();
  uint32_t pa0_freq = frequency_hz;

  if ((now - frequency_last_capture_tick) > FREQ_TIMEOUT_MS)
  {
    pa0_freq = 0U;
    frequency_hz = 0U;
  }

  adc1_value = ReadAdcValue(&hadc1, adc1_value);
  adc2_value = ReadAdcValue(&hadc2, adc2_value);
  uint32_t adc1_freq = AdcFreq_Update(&adc1_freq_meter, adc1_value, now);
  uint32_t adc2_freq = AdcFreq_Update(&adc2_freq_meter, adc2_value, now);

  UpdateDacOutputs(adc1_value, adc2_value);
  LcdDisplay_Update(now, adc1_value, adc2_value, pa0_freq, adc1_freq, adc2_freq);

  if ((now - last_vofa_tick) >= VOFA_SEND_PERIOD_MS)
  {
    last_vofa_tick = now;
    LcdPushSamples(adc1_value, adc2_value);
    Vofa_SendSamples(adc1_value, adc2_value, pa0_freq, adc1_freq, adc2_freq);
  }
}

static void AdcDacVofa_Start(void)
{
  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static uint16_t ReadAdcValue(ADC_HandleTypeDef *hadc, uint16_t previous_value)
{
  (void)HAL_ADC_Stop(hadc);
  __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);

  if (HAL_ADC_Start(hadc) != HAL_OK)
  {
    return previous_value;
  }

  if (HAL_ADC_PollForConversion(hadc, 10U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(hadc);
    return previous_value;
  }

  uint16_t value = (uint16_t)HAL_ADC_GetValue(hadc);
  (void)HAL_ADC_Stop(hadc);

  return value;
}

static uint32_t AdcFreq_Update(AdcFreqMeter *meter, uint16_t sample, uint32_t now)
{
  if (sample <= ADC_FREQ_LOW_THRESHOLD)
  {
    meter->armed = 1U;
  }

  if ((meter->armed != 0U) && (sample >= ADC_FREQ_HIGH_THRESHOLD))
  {
    if (meter->last_cross_tick != 0U)
    {
      uint32_t period_ms = now - meter->last_cross_tick;

      if (period_ms != 0U)
      {
        meter->frequency_hz = (1000U + (period_ms / 2U)) / period_ms;
      }
    }

    meter->last_cross_tick = now;
    meter->armed = 0U;
  }

  if ((meter->last_cross_tick == 0U) || ((now - meter->last_cross_tick) > FREQ_TIMEOUT_MS))
  {
    meter->frequency_hz = 0U;
  }

  return meter->frequency_hz;
}

static void UpdateDacOutputs(uint16_t value1, uint16_t value2)
{
  if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, value2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Vofa_SendSamples(uint16_t value1, uint16_t value2, uint32_t pa0_freq, uint32_t adc1_freq, uint32_t adc2_freq)
{
  char tx_buf[64];
  int len = snprintf(tx_buf, sizeof(tx_buf), "%u,%u,%lu,%lu,%lu\r\n",
                     value1,
                     value2,
                     (unsigned long)pa0_freq,
                     (unsigned long)adc1_freq,
                     (unsigned long)adc2_freq);

  if (len > 0)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)len, 10U);
  }
}

static void LcdDisplay_Init(void)
{
  LCD_Init();
  LcdDrawStatic();
  LcdDrawValues(0U, 0U, 0U, 0U, 0U);
}

static void LcdDisplay_Update(uint32_t now, uint16_t value1, uint16_t value2, uint32_t pa0_freq, uint32_t adc1_freq, uint32_t adc2_freq)
{
  static uint32_t last_lcd_tick = 0U;

  if ((now - last_lcd_tick) < LCD_REFRESH_PERIOD_MS)
  {
    return;
  }

  last_lcd_tick = now;
  LcdDrawValues(value1, value2, pa0_freq, adc1_freq, adc2_freq);
  LcdDrawWaveform(LCD_ADC1_TOP, lcd_adc1_wave, BLUE);
  LcdDrawWaveform(LCD_ADC2_TOP, lcd_adc2_wave, RED);
}

static void LcdPushSamples(uint16_t value1, uint16_t value2)
{
  lcd_adc1_wave[lcd_wave_index] = value1;
  lcd_adc2_wave[lcd_wave_index] = value2;

  lcd_wave_index++;
  if (lcd_wave_index >= LCD_WAVE_POINTS)
  {
    lcd_wave_index = 0U;
  }

  if (lcd_wave_count < LCD_WAVE_POINTS)
  {
    lcd_wave_count++;
  }
}

static void LcdDrawStatic(void)
{
  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
  LCD_Clear(WHITE);
  LCD_ShowString(8, 4, 220, 16, 16, (uint8_t *)"ADC DAC VOFA LCD");
  LcdDrawWaveFrame(LCD_ADC1_TOP, "ADC1 PA2 -> DAC1 PA4", BLUE);
  LcdDrawWaveFrame(LCD_ADC2_TOP, "ADC2 PA3 -> DAC2 PA5", RED);
}

static void LcdDrawValues(uint16_t value1, uint16_t value2, uint32_t pa0_freq, uint32_t adc1_freq, uint32_t adc2_freq)
{
  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;

  LCD_Fill(0, 22, 319, 63, WHITE);

  LCD_ShowString(8, 22, 18, 12, 12, (uint8_t *)"A1");
  LCD_ShowNum(30, 22, value1, 4, 12);
  LCD_ShowString(76, 22, 18, 12, 12, (uint8_t *)"F1");
  LcdDrawFrequency(96, 22, adc1_freq);

  LCD_ShowString(8, 36, 18, 12, 12, (uint8_t *)"A2");
  LCD_ShowNum(30, 36, value2, 4, 12);
  LCD_ShowString(76, 36, 18, 12, 12, (uint8_t *)"F2");
  LcdDrawFrequency(96, 36, adc2_freq);

  LCD_ShowString(8, 50, 18, 12, 12, (uint8_t *)"P0");
  LcdDrawFrequency(30, 50, pa0_freq);
}

static void LcdDrawFrequency(uint16_t x, uint16_t y, uint32_t freq)
{
  if (freq < 100000U)
  {
    LCD_ShowNum(x, y, freq, 5, 12);
    LCD_ShowString(x + 34U, y, 18, 12, 12, (uint8_t *)"Hz");
  }
  else
  {
    LCD_ShowNum(x, y, freq / 1000U, 5, 12);
    LCD_ShowString(x + 34U, y, 24, 12, 12, (uint8_t *)"kHz");
  }
}

static void LcdDrawWaveFrame(uint16_t top, const char *label, uint16_t color)
{
  uint16_t bottom = top + LCD_WAVE_HEIGHT - 1U;
  uint16_t right = LCD_WAVE_LEFT + LCD_WAVE_WIDTH - 1U;
  uint16_t mid = top + LCD_WAVE_HEIGHT / 2U;

  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
  LCD_ShowString(LCD_WAVE_LEFT, top - 14U, 180, 12, 12, (uint8_t *)label);

  POINT_COLOR = GRAY;
  LCD_DrawRectangle(LCD_WAVE_LEFT, top, right, bottom);
  LCD_DrawLine(LCD_WAVE_LEFT, mid, right, mid);

  POINT_COLOR = color;
}

static void LcdDrawWaveform(uint16_t top, const uint16_t *samples, uint16_t color)
{
  uint16_t bottom = top + LCD_WAVE_HEIGHT - 1U;
  uint16_t right = LCD_WAVE_LEFT + LCD_WAVE_WIDTH - 1U;
  uint16_t start = 0U;
  uint16_t last_x = LCD_WAVE_LEFT;
  uint16_t last_y;

  LCD_Fill(LCD_WAVE_LEFT + 1U, top + 1U, right - 1U, bottom - 1U, WHITE);
  LcdDrawWaveFrame(top, top == LCD_ADC1_TOP ? "ADC1 PA2 -> DAC1 PA4" : "ADC2 PA3 -> DAC2 PA5", color);

  if (lcd_wave_count < 2U)
  {
    return;
  }

  if (lcd_wave_count >= LCD_WAVE_POINTS)
  {
    start = lcd_wave_index;
  }

  last_y = LcdAdcToY(samples[start], top);
  POINT_COLOR = color;

  for (uint16_t i = 1U; i < lcd_wave_count; ++i)
  {
    uint16_t sample_index = (uint16_t)((start + i) % LCD_WAVE_POINTS);
    uint16_t x = (uint16_t)(LCD_WAVE_LEFT + i);
    uint16_t y = LcdAdcToY(samples[sample_index], top);

    LCD_DrawLine(last_x, last_y, x, y);
    last_x = x;
    last_y = y;
  }
}

static uint16_t LcdAdcToY(uint16_t value, uint16_t top)
{
  uint32_t y_span = LCD_WAVE_HEIGHT - 3U;
  uint32_t y = top + 1U + y_span - ((uint32_t)value * y_span) / ADC_MAX_CODE;

  return (uint16_t)y;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM5) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    if (tim5_capture_ready != 0U)
    {
      uint32_t delta = capture - tim5_last_capture;

      if (delta != 0U)
      {
        frequency_hz = TIM5_COUNTER_HZ / delta;
        frequency_last_capture_tick = HAL_GetTick();
      }
    }
    else
    {
      tim5_capture_ready = 1U;
      frequency_last_capture_tick = HAL_GetTick();
    }

    tim5_last_capture = capture;
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
