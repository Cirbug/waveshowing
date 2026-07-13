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
#include "touch.h"
#include "scalar_kalman.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t armed;              /* 已经低于下阈值后才允许检测下一次上升沿，防止同一个波形被重复计数 */
  uint32_t last_cross_tick;   /* 上一次 ADC 波形越过高阈值的系统 tick，单位 ms */
  uint32_t frequency_hz;      /* 根据两次越过阈值的时间差估算出的频率 */
} AdcFreqMeter;

typedef enum
{
  UI_PAGE_HOME = 0,
  UI_PAGE_SINGLE,
  UI_PAGE_DOUBLE
} UiPage;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOFA_SEND_PERIOD_MS 2U          /* VOFA 串口发送周期，2ms 约等于 500Hz 刷新 */
#define LCD_REFRESH_PERIOD_MS 100U      /* LCD 刷新比较慢，100ms 刷一次可减少闪烁和占用 */
#define UI_NAV_TOP 200U                 /* 底部导航栏顶部，屏幕为 320x240 */
#define UI_NAV_BOTTOM 239U
#define UI_NAV_HOME_RIGHT 106U
#define UI_NAV_SINGLE_RIGHT 213U
#define TIM5_COUNTER_HZ 1000000U        /* TIM5 计数频率：84MHz / 84 = 1MHz，1 个计数 = 1us */
#define FREQ_TIMEOUT_MS 1000U           /* 超过 1 秒没有新边沿/过阈值，就认为频率为 0 */
#define ADC_FREQ_LOW_THRESHOLD 1900U    /* ADC 软件测频低阈值，用来重新“武装”检测 */
#define ADC_FREQ_HIGH_THRESHOLD 2200U   /* ADC 软件测频高阈值，超过它认为出现一次上升穿越 */

#define PA1_PERIOD_MIN_COUNT 5U         /* 过滤过短的干扰脉冲，最高约 200kHz */
#define PA1_PERIOD_MAX_COUNT 5000U      /* 过滤过长的异常周期，最低约 200Hz */
#define ADC_KALMAN_Q 4.0f               /* ADC 卡尔曼过程噪声，越大响应越快 */
#define ADC_KALMAN_R 25.0f              /* ADC 卡尔曼测量噪声，越大滤波越强 */
#define PA1_KALMAN_Q 100.0f             /* PA1 频率卡尔曼过程噪声 */
#define PA1_KALMAN_R 2500.0f            /* PA1 频率卡尔曼测量噪声 */
#define LENGTH_SCALE_X1000 21461918UL   /* 新标定曲线系数，内部长度单位为 0.001m */
#define LENGTH_OFFSET_X1000 253UL       /* 新标定曲线偏移量 0.253m */
#define LENGTH_MAX_X10 500U             /* 最大测量长度 50.0m */
#define LENGTH_INVALID_X10 0xFFFFU      /* 没有频率输入时的无效长度 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t adc1_value = 0U;                         /* ADC1 当前采样值，PA2 */
static uint16_t adc2_value = 0U;                         /* ADC2 当前采样值，PA3 */
static volatile uint32_t frequency_hz = 0U;              /* PA1/TIM5 输入捕获测得的频率 */
static volatile uint32_t frequency_last_capture_tick = 0U; /* PA1 最近一次捕获到边沿的 tick */
static uint32_t tim5_last_capture = 0U;                  /* TIM5 上一次捕获寄存器值 */
static uint8_t tim5_capture_ready = 0U;                  /* 第一次捕获只记录基准，第二次开始才可计算周期 */
static scalar_kalman_t adc1_kalman;                      /* ADC1 独立卡尔曼滤波器 */
static scalar_kalman_t adc2_kalman;                      /* ADC2 独立卡尔曼滤波器 */
static scalar_kalman_t pa1_freq_kalman;                  /* PA1 频率独立卡尔曼滤波器 */
static uint8_t adc1_kalman_ready = 0U;
static uint8_t adc2_kalman_ready = 0U;
static volatile uint8_t pa1_kalman_ready = 0U;
static AdcFreqMeter adc1_freq_meter = {1U, 0U, 0U};      /* ADC1 软件测频状态 */
static AdcFreqMeter adc2_freq_meter = {1U, 0U, 0U};      /* ADC2 软件测频状态 */
static TouchState touch_state = {0U, 0U, 0U, 0U, 0U};
static UiPage current_page = UI_PAGE_HOME;
static uint8_t page_dirty = 1U;                          /* 切页后需要重画静态内容 */
static uint8_t touch_was_pressed = 0U;                   /* 用于检测一次新的按下动作 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void AdcDacVofa_Start(void);
static uint16_t ReadAdcValue(ADC_HandleTypeDef *hadc, uint16_t previous_value);
static uint16_t FilterAdcValue(scalar_kalman_t *kalman, uint8_t *ready, uint16_t sample);
static uint32_t AdcFreq_Update(AdcFreqMeter *meter, uint16_t sample, uint32_t now);
static void UpdateDacOutputs(uint16_t value1, uint16_t value2);
static void Vofa_SendSamples(uint16_t value1, uint16_t value2, uint32_t pa1_freq, uint32_t adc1_freq, uint32_t adc2_freq);
static void Ui_Init(void);
static void Ui_Update(uint32_t now, uint16_t value1, uint16_t value2, uint32_t pa1_freq);
static void Ui_HandleTouch(const TouchState *state);
static void Ui_DrawPageStatic(void);
static void Ui_DrawNavigation(void);
static void Ui_DrawSingleValue(uint32_t pa1_freq);
static void Ui_DrawDoubleValues(uint16_t value1, uint16_t value2);
static void Ui_DrawLargeFrequency(uint16_t x, uint16_t y, uint32_t freq);
static void Ui_DrawSignedValue(uint16_t x, uint16_t y, int32_t value, uint8_t size);
static uint16_t CalculateLengthX10(uint32_t frequency);
static void Ui_DrawLength(uint16_t x, uint16_t y, uint16_t length_x10);
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
  /* FreeRTOS 任务启动后先初始化 LCD、DAC，再打开 PA1 的 TIM5 输入捕获中断 */
  scalar_kalman_init(&adc1_kalman, 1.0f, 1.0f, ADC_KALMAN_Q, ADC_KALMAN_R);
  scalar_kalman_init(&adc2_kalman, 1.0f, 1.0f, ADC_KALMAN_Q, ADC_KALMAN_R);
  scalar_kalman_init(&pa1_freq_kalman, 1.0f, 1.0f, PA1_KALMAN_Q, PA1_KALMAN_R);

  Ui_Init();
  Touch_Init();
  AdcDacVofa_Start();

  if (HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void App_TaskStep(void)
{
  static uint32_t last_vofa_tick = 0U;
  static uint32_t last_touch_tick = 0U;
  uint32_t now = HAL_GetTick();
  uint32_t pa1_freq = frequency_hz;

  /* PA1 如果 1 秒没有捕获到新边沿，说明输入信号停止或频率太低，显示为 0Hz */
  if ((now - frequency_last_capture_tick) > FREQ_TIMEOUT_MS)
  {
    pa1_freq = 0U;
    frequency_hz = 0U;
    pa1_kalman_ready = 0U;
  }

  /* ADC1、ADC2 分别采样并使用独立卡尔曼滤波器，避免两个通道相互影响。 */
  uint16_t adc1_sample = ReadAdcValue(&hadc1, adc1_value);
  uint16_t adc2_sample = ReadAdcValue(&hadc2, adc2_value);
  adc1_value = FilterAdcValue(&adc1_kalman, &adc1_kalman_ready, adc1_sample);
  adc2_value = FilterAdcValue(&adc2_kalman, &adc2_kalman_ready, adc2_sample);

  /* 用 ADC 采样值做软件测频。这个方法适合低频，采样间隔越稳定越准 */
  uint32_t adc1_freq = AdcFreq_Update(&adc1_freq_meter, adc1_value, now);
  uint32_t adc2_freq = AdcFreq_Update(&adc2_freq_meter, adc2_value, now);

  if ((now - last_touch_tick) >= 20U)
  {
    last_touch_tick = now;
    (void)Touch_Scan(&touch_state);
    Ui_HandleTouch(&touch_state);
  }

  /* DAC 输出跟随 ADC：ADC1 -> DAC1(PA4)，ADC2 -> DAC2(PA5) */
  UpdateDacOutputs(adc1_value, adc2_value);

  /* LCD 不需要每 1ms 都刷，函数内部会按 LCD_REFRESH_PERIOD_MS 限速 */
  Ui_Update(now, adc1_value, adc2_value, pa1_freq);

  if ((now - last_vofa_tick) >= VOFA_SEND_PERIOD_MS)
  {
    last_vofa_tick = now;
    Vofa_SendSamples(adc1_value, adc2_value, pa1_freq, adc1_freq, adc2_freq);
  }
}

static void AdcDacVofa_Start(void)
{
  /* DAC 必须先启动，后面 HAL_DAC_SetValue 写入的值才会真正输出到引脚 */
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
  /* 单次轮询读取 ADC。失败时返回上一次值，避免屏幕/输出突然跳到 0 */
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

static uint16_t FilterAdcValue(scalar_kalman_t *kalman, uint8_t *ready, uint16_t sample)
{
  float filtered;

  /* 第一次采样直接作为初始状态，避免滤波器从 0 缓慢爬升。 */
  if (*ready == 0U)
  {
    kalman->x = (float)sample;
    kalman->P = kalman->R;
    *ready = 1U;
    return sample;
  }

  filtered = scalar_kalman(kalman, (float)sample);
  if (filtered <= 0.0f)
  {
    return 0U;
  }

  if (filtered >= 4095.0f)
  {
    return 4095U;
  }

  return (uint16_t)(filtered + 0.5f);
}

static uint32_t AdcFreq_Update(AdcFreqMeter *meter, uint16_t sample, uint32_t now)
{
  /* 先低于低阈值，再高于高阈值，才算一次新的上升穿越；这样能过滤阈值附近的小抖动 */
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
        /* period_ms 是一个周期的毫秒数，频率约等于 1000 / period_ms，加入半周期用于四舍五入 */
        meter->frequency_hz = (1000U + (period_ms / 2U)) / period_ms;
      }
    }

    /* 记录这次穿越时间，并等待波形重新下降到低阈值后再检测下一次 */
    meter->last_cross_tick = now;
    meter->armed = 0U;
  }

  /* 长时间没有再次穿越阈值，就把频率清零 */
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

static void Vofa_SendSamples(uint16_t value1, uint16_t value2, uint32_t pa1_freq, uint32_t adc1_freq, uint32_t adc2_freq)
{
  char tx_buf[64];

  /* VOFA 以逗号分隔多通道数据：ADC1, ADC2, PA1频率, ADC1频率, ADC2频率 */
  int len = snprintf(tx_buf, sizeof(tx_buf), "%u,%u,%lu,%lu,%lu\r\n",
                     value1,
                     value2,
                     (unsigned long)pa1_freq,
                     (unsigned long)adc1_freq,
                     (unsigned long)adc2_freq);

  if (len > 0)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)len, 10U);
  }
}

static void Ui_Init(void)
{
  LCD_Init();
  current_page = UI_PAGE_HOME;
  page_dirty = 1U;
  Ui_DrawPageStatic();
  page_dirty = 0U;
}

static void Ui_Update(uint32_t now, uint16_t value1, uint16_t value2, uint32_t pa1_freq)
{
  static uint32_t last_lcd_tick = 0U;
  uint8_t page_redrawn = 0U;

  if (page_dirty != 0U)
  {
    Ui_DrawPageStatic();
    page_dirty = 0U;
    page_redrawn = 1U;
  }

  if ((page_redrawn == 0U) && ((now - last_lcd_tick) < LCD_REFRESH_PERIOD_MS))
  {
    return;
  }

  last_lcd_tick = now;

  if (current_page == UI_PAGE_SINGLE)
  {
    Ui_DrawSingleValue(pa1_freq);
  }
  else if (current_page == UI_PAGE_DOUBLE)
  {
    Ui_DrawDoubleValues(value1, value2);
  }
}

static void Ui_HandleTouch(const TouchState *state)
{
  UiPage new_page = current_page;

  if (state == NULL)
  {
    return;
  }

  /* 只响应一次新的按下动作，避免手指按住时重复切页 */
  if ((state->pressed != 0U) && (touch_was_pressed == 0U) && (state->y >= UI_NAV_TOP))
  {
    if (state->x <= UI_NAV_HOME_RIGHT)
    {
      new_page = UI_PAGE_HOME;
    }
    else if (state->x <= UI_NAV_SINGLE_RIGHT)
    {
      new_page = UI_PAGE_SINGLE;
    }
    else
    {
      new_page = UI_PAGE_DOUBLE;
    }

    if (new_page != current_page)
    {
      current_page = new_page;
      page_dirty = 1U;
    }
  }

  touch_was_pressed = state->pressed;
}

static void Ui_DrawPageStatic(void)
{
  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
  LCD_Clear(WHITE);

  if (current_page == UI_PAGE_SINGLE)
  {
    LCD_ShowString(94, 18, 132, 24, 24, (uint8_t *)"SINGLE END");
    LCD_ShowString(104, 70, 112, 16, 16, (uint8_t *)"P1 FREQUENCY");
    POINT_COLOR = BLUE;
    LCD_DrawRectangle(38, 96, 281, 145);
    POINT_COLOR = BLACK;
    LCD_ShowString(70, 164, 72, 16, 16, (uint8_t *)"LENGTH :");
  }
  else if (current_page == UI_PAGE_DOUBLE)
  {
    LCD_ShowString(94, 12, 132, 24, 24, (uint8_t *)"DOUBLE END");
    LCD_ShowString(52, 62, 72, 24, 24, (uint8_t *)"ADC1 :");
    LCD_ShowString(52, 105, 72, 24, 24, (uint8_t *)"ADC2 :");
    LCD_ShowString(52, 148, 72, 24, 24, (uint8_t *)"DIFF :");
  }
  /* 首页内容区按需求暂时留空。 */

  Ui_DrawNavigation();
}

static void Ui_DrawNavigation(void)
{
  /* 当前页面使用蓝底白字，其余页面使用白底黑字。 */
  if (current_page == UI_PAGE_HOME)
  {
    LCD_Fill(0, UI_NAV_TOP, UI_NAV_HOME_RIGHT, UI_NAV_BOTTOM, BLUE);
  }
  else if (current_page == UI_PAGE_SINGLE)
  {
    LCD_Fill(UI_NAV_HOME_RIGHT + 1U, UI_NAV_TOP, UI_NAV_SINGLE_RIGHT, UI_NAV_BOTTOM, BLUE);
  }
  else
  {
    LCD_Fill(UI_NAV_SINGLE_RIGHT + 1U, UI_NAV_TOP, 319, UI_NAV_BOTTOM, BLUE);
  }

  POINT_COLOR = GRAY;
  LCD_DrawRectangle(0, UI_NAV_TOP, 319, UI_NAV_BOTTOM);
  LCD_DrawLine(UI_NAV_HOME_RIGHT, UI_NAV_TOP, UI_NAV_HOME_RIGHT, UI_NAV_BOTTOM);
  LCD_DrawLine(UI_NAV_SINGLE_RIGHT, UI_NAV_TOP, UI_NAV_SINGLE_RIGHT, UI_NAV_BOTTOM);

  POINT_COLOR = (current_page == UI_PAGE_HOME) ? WHITE : BLACK;
  BACK_COLOR = (current_page == UI_PAGE_HOME) ? BLUE : WHITE;
  LCD_ShowString(37, 212, 40, 16, 16, (uint8_t *)"HOME");

  POINT_COLOR = (current_page == UI_PAGE_SINGLE) ? WHITE : BLACK;
  BACK_COLOR = (current_page == UI_PAGE_SINGLE) ? BLUE : WHITE;
  LCD_ShowString(132, 212, 48, 16, 16, (uint8_t *)"SINGLE");

  POINT_COLOR = (current_page == UI_PAGE_DOUBLE) ? WHITE : BLACK;
  BACK_COLOR = (current_page == UI_PAGE_DOUBLE) ? BLUE : WHITE;
  LCD_ShowString(239, 212, 48, 16, 16, (uint8_t *)"DOUBLE");

  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
}

static void Ui_DrawSingleValue(uint32_t pa1_freq)
{
  uint16_t length_x10 = CalculateLengthX10(pa1_freq);

  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
  LCD_Fill(40, 98, 279, 143, WHITE);
  Ui_DrawLargeFrequency(50, 108, pa1_freq);
  LCD_Fill(150, 154, 279, 190, WHITE);
  Ui_DrawLength(154, 158, length_x10);
}

/*
 * 根据新标定点计算长度：6.4kHz=3.1m，12.24kHz=1.5m。
 * 反比例曲线为 L=21461.918/f-0.253，内部使用 0.001m 避免浮点运算。
 */
static uint16_t CalculateLengthX10(uint32_t frequency)
{
  uint32_t reciprocal_x1000;
  uint32_t length_x1000;

  if (frequency == 0U)
  {
    return LENGTH_INVALID_X10;
  }

  reciprocal_x1000 = LENGTH_SCALE_X1000 / frequency;
  if (reciprocal_x1000 <= LENGTH_OFFSET_X1000)
  {
    return 0U;
  }

  length_x1000 = reciprocal_x1000 - LENGTH_OFFSET_X1000;
  if (length_x1000 >= ((uint32_t)LENGTH_MAX_X10 * 100U))
  {
    return LENGTH_MAX_X10;
  }

  /* 从 0.001m 四舍五入到屏幕显示使用的 0.1m。 */
  return (uint16_t)((length_x1000 + 50U) / 100U);
}

static void Ui_DrawLength(uint16_t x, uint16_t y, uint16_t length_x10)
{
  if (length_x10 == LENGTH_INVALID_X10)
  {
    LCD_ShowString(x, y, 84, 24, 24, (uint8_t *)"--.- m");
    return;
  }

  LCD_ShowNum(x, y, length_x10 / 10U, 2, 24);
  LCD_ShowString(x + 24U, y, 12, 24, 24, (uint8_t *)".");
  LCD_ShowNum(x + 36U, y, length_x10 % 10U, 1, 24);
  LCD_ShowString(x + 52U, y, 12, 24, 24, (uint8_t *)"m");
}

static void Ui_DrawDoubleValues(uint16_t value1, uint16_t value2)
{
  int32_t difference = (int32_t)value1 - (int32_t)value2;

  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
  LCD_Fill(136, 60, 270, 178, WHITE);
  LCD_ShowNum(150, 62, value1, 4, 24);
  LCD_ShowNum(150, 105, value2, 4, 24);
  Ui_DrawSignedValue(150, 148, difference, 24U);
}

static void Ui_DrawLargeFrequency(uint16_t x, uint16_t y, uint32_t freq)
{
  if (freq < 1000000U)
  {
    LCD_ShowNum(x, y, freq, 7, 24);
    LCD_ShowString(x + 104U, y, 24, 24, 24, (uint8_t *)"Hz");
  }
  else
  {
    LCD_ShowNum(x + 20U, y, freq / 1000U, 5, 24);
    LCD_ShowString(x + 104U, y, 36, 24, 24, (uint8_t *)"kHz");
  }
}

static void Ui_DrawSignedValue(uint16_t x, uint16_t y, int32_t value, uint8_t size)
{
  uint32_t abs_value;
  uint16_t character_width = (uint16_t)(size / 2U);

  if (value < 0)
  {
    LCD_ShowString(x, y, character_width, size, size, (uint8_t *)"-");
    abs_value = (uint32_t)(-value);
  }
  else
  {
    LCD_ShowString(x, y, character_width, size, size, (uint8_t *)"+");
    abs_value = (uint32_t)value;
  }

  LCD_ShowNum(x + character_width + 4U, y, abs_value, 4, size);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* PA1 接到 TIM5_CH2。每来一个上升沿，硬件会把当前计数值锁存到捕获寄存器 */
  if ((htim->Instance == TIM5) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
  {
    uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    if (tim5_capture_ready != 0U)
    {
      /* 两次捕获值的差就是一个周期的计数。TIM5 是 32 位，uint32_t 自然溢出也能得到正确差值 */
      uint32_t delta = capture - tim5_last_capture;

      if ((delta >= PA1_PERIOD_MIN_COUNT) && (delta <= PA1_PERIOD_MAX_COUNT))
      {
        uint32_t raw_frequency = (TIM5_COUNTER_HZ + (delta / 2U)) / delta;
        float filtered_frequency;

        /* 第一个有效频率直接作为卡尔曼初值，避免从 0Hz 缓慢上升。 */
        if (pa1_kalman_ready == 0U)
        {
          pa1_freq_kalman.x = (float)raw_frequency;
          pa1_freq_kalman.P = pa1_freq_kalman.R;
          pa1_kalman_ready = 1U;
          filtered_frequency = (float)raw_frequency;
        }
        else
        {
          filtered_frequency = scalar_kalman(&pa1_freq_kalman, (float)raw_frequency);
        }

        if (filtered_frequency > 0.0f)
        {
          frequency_hz = (uint32_t)(filtered_frequency + 0.5f);
        }

        frequency_last_capture_tick = HAL_GetTick();
      }
    }
    else
    {
      /* 第一次捕获还没有上一次数据，只能作为基准保存 */
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
