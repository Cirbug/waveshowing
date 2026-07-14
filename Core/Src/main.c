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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "touch.h"
#include "scalar_kalman.h"
#include "spi_flash.h"
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
  UI_PAGE_DOUBLE,
  UI_PAGE_CALIBRATION
} UiPage;

typedef struct
{
  uint8_t valid;
  uint8_t frequency_valid;
  uint32_t frequency_hz;
  int32_t difference;
} UiMeasurementResult;

#define CALIBRATION_MAX_POINTS 5U

typedef struct
{
  uint32_t point_count;
  float length_m[CALIBRATION_MAX_POINTS];
  float resistance_ohm[CALIBRATION_MAX_POINTS];
  float resistance_per_meter;
  float zero_resistance;
} DoubleCalibrationData;

typedef struct
{
  uint32_t point_count;
  float length_m[CALIBRATION_MAX_POINTS];
  uint32_t frequency_hz[CALIBRATION_MAX_POINTS];
  float inverse_period_slope;
  float offset_m;
} SingleCalibrationData;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  SingleCalibrationData single;
  DoubleCalibrationData double_end;
  uint32_t checksum;
} CalibrationStorage;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOFA_SEND_PERIOD_MS 2U          /* VOFA 串口发送周期，2ms 约等于 500Hz 刷新 */
#define UI_NAV_TOP 200U                 /* 底部导航栏顶部，屏幕为 320x240 */
#define UI_NAV_BOTTOM 239U
#define UI_NAV_HOME_RIGHT 79U
#define UI_NAV_SINGLE_RIGHT 159U
#define UI_NAV_DOUBLE_RIGHT 239U
#define UI_RUN_LEFT 244U
#define UI_RUN_TOP 8U
#define UI_RUN_RIGHT 311U
#define UI_RUN_BOTTOM 42U
#define RUN_DURATION_MS 4000U           /* 每次按 RUN 后采集 4 秒 */
#define RUN_SAMPLE_PERIOD_MS 20U        /* 每 20ms 保存一个样本 */
#define RUN_MAX_SAMPLES 200U            /* 4 秒最多保存 200 组样本 */
#define CALIBRATION_MAGIC 0x43414C32UL   /* Flash 校准数据标志：CAL2 */
#define CALIBRATION_VERSION 2UL
#define CALIBRATION_FLASH_ADDRESS 0x00FFF000UL /* NM25Q128 最后一个 4KB 扇区 */
#define CAL_KEYPAD_LEFT 140U
#define CAL_KEYPAD_TOP 48U
#define CAL_KEY_WIDTH 60U
#define CAL_KEY_HEIGHT 37U
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
#define DIFF_VOLTAGE_SCALE 2.0f         /* 差分 ADC 值乘 2 后作为电压采样值，单位 mV */
#define MEASURE_CURRENT_MA 330.0f       /* 双端测量使用的恒定电流，单位 mA */
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
static uint8_t measurement_running = 0U;
static UiPage measurement_page = UI_PAGE_HOME;
static uint32_t measurement_start_tick = 0U;
static uint32_t measurement_last_sample_tick = 0U;
static uint32_t frequency_samples[RUN_MAX_SAMPLES];
static int32_t difference_samples[RUN_MAX_SAMPLES];
static uint16_t frequency_sample_count = 0U;
static uint16_t difference_sample_count = 0U;
static UiMeasurementResult measurement_results[4];
static DoubleCalibrationData double_calibration;
static SingleCalibrationData single_calibration;
static char calibration_value_input[10];
static uint8_t calibration_value_input_length = 0U;
static char calibration_length_input[8];
static uint8_t calibration_input_length = 0U;
static uint8_t calibration_active_field = 0U;             /* 0频率/电阻输入，1线长输入 */
static uint8_t calibration_status = 0U;                  /* 0空闲，1已添加，2已保存，3输入错误，4数据不足 */
static uint8_t spi_flash_ready = 0U;
static uint16_t spi_flash_id = 0U;
static uint8_t calibration_mode = 0U;                    /* 0单端校准，1双端校准 */
static uint8_t calibration_selected_point = 0U;

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
static void Ui_DrawRunButton(void);
static void Ui_DrawHomeValues(const UiMeasurementResult *result);
static void Ui_DrawCalibrationPage(void);
static void Ui_DrawCalibrationValues(void);
static void Ui_HandleCalibrationTouch(uint16_t x, uint16_t y);
static void Ui_DrawSingleValue(uint32_t pa1_freq);
static void Ui_DrawDoubleValue(int32_t difference);
static void Ui_DrawLargeFrequency(uint16_t x, uint16_t y, uint32_t freq);
static void Ui_DrawSignedValue(uint16_t x, uint16_t y, int32_t value, uint8_t size);
static uint16_t CalculateLengthX10(uint32_t frequency);
static void Ui_DrawLength(uint16_t x, uint16_t y, uint16_t length_x10);
static void Measurement_Start(uint32_t now);
static void Measurement_Update(uint32_t now, uint32_t pa1_freq, uint16_t value1, uint16_t value2);
static uint32_t MedianU32(uint32_t *values, uint16_t count);
static int32_t MedianS32(int32_t *values, uint16_t count);
static float DifferenceToResistance(int32_t difference);
static uint8_t Calibration_Fit(void);
static uint8_t SingleCalibration_Fit(void);
static float Calibration_ParseText(const char *text, uint8_t text_length);
static float Calibration_ParseValue(void);
static float Calibration_ParseLength(void);
static void Calibration_AppendKey(char key);
static uint8_t Calibration_AddPoint(void);
static void Calibration_DeletePoint(void);
static void Calibration_SelectPoint(int8_t direction);
static uint8_t Calibration_Save(void);
static void Calibration_Load(void);
static uint32_t Calibration_Checksum(const CalibrationStorage *data);
static uint8_t Calibration_GetLength(float resistance_ohm, float *length_m);
static uint8_t SingleCalibration_GetLength(uint32_t frequency_hz, float *length_m);
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
  MX_SPI1_Init();
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
  Calibration_Load();

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

  /* RUN 测量按固定周期收集滤波后的数据，满 4 秒后计算中位数。 */
  Measurement_Update(now, pa1_freq, adc1_value, adc2_value);

  if ((now - last_touch_tick) >= 20U)
  {
    last_touch_tick = now;
    (void)Touch_Scan(&touch_state);
    Ui_HandleTouch(&touch_state);
  }

  /* DAC 输出跟随 ADC：ADC1 -> DAC1(PA4)，ADC2 -> DAC2(PA5) */
  UpdateDacOutputs(adc1_value, adc2_value);

  /* LCD 只在页面状态改变时重画，避免数值区域持续刷新产生频闪。 */
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

static void Measurement_Start(uint32_t now)
{
  measurement_page = current_page;
  measurement_start_tick = now;
  measurement_last_sample_tick = now - RUN_SAMPLE_PERIOD_MS;
  frequency_sample_count = 0U;
  difference_sample_count = 0U;
  measurement_results[measurement_page].valid = 0U;
  measurement_results[measurement_page].frequency_valid = 0U;
  measurement_running = 1U;
  page_dirty = 1U;
}

static void Measurement_Update(uint32_t now, uint32_t pa1_freq, uint16_t value1, uint16_t value2)
{
  UiMeasurementResult *result;

  if (measurement_running == 0U)
  {
    return;
  }

  if ((now - measurement_last_sample_tick) >= RUN_SAMPLE_PERIOD_MS)
  {
    measurement_last_sample_tick = now;

    if (difference_sample_count < RUN_MAX_SAMPLES)
    {
      difference_samples[difference_sample_count++] = (int32_t)value1 - (int32_t)value2;
    }

    if ((pa1_freq != 0U) && (frequency_sample_count < RUN_MAX_SAMPLES))
    {
      frequency_samples[frequency_sample_count++] = pa1_freq;
    }
  }

  if ((now - measurement_start_tick) < RUN_DURATION_MS)
  {
    return;
  }

  result = &measurement_results[measurement_page];
  result->frequency_valid = (frequency_sample_count != 0U) ? 1U : 0U;
  result->frequency_hz = (frequency_sample_count != 0U) ? MedianU32(frequency_samples, frequency_sample_count) : 0U;
  result->difference = (difference_sample_count != 0U) ? MedianS32(difference_samples, difference_sample_count) : 0;

  if (measurement_page == UI_PAGE_SINGLE)
  {
    result->valid = result->frequency_valid;
  }
  else
  {
    result->valid = (difference_sample_count != 0U) ? 1U : 0U;
  }

  measurement_running = 0U;
  page_dirty = 1U;
}

static uint32_t MedianU32(uint32_t *values, uint16_t count)
{
  for (uint16_t i = 1U; i < count; ++i)
  {
    uint32_t key = values[i];
    uint16_t j = i;

    while ((j > 0U) && (values[j - 1U] > key))
    {
      values[j] = values[j - 1U];
      --j;
    }

    values[j] = key;
  }

  if ((count & 1U) != 0U)
  {
    return values[count / 2U];
  }

  return (uint32_t)(((uint64_t)values[(count / 2U) - 1U] + values[count / 2U]) / 2U);
}

static int32_t MedianS32(int32_t *values, uint16_t count)
{
  for (uint16_t i = 1U; i < count; ++i)
  {
    int32_t key = values[i];
    uint16_t j = i;

    while ((j > 0U) && (values[j - 1U] > key))
    {
      values[j] = values[j - 1U];
      --j;
    }

    values[j] = key;
  }

  if ((count & 1U) != 0U)
  {
    return values[count / 2U];
  }

  return (int32_t)(((int64_t)values[(count / 2U) - 1U] + values[count / 2U]) / 2);
}

static float DifferenceToResistance(int32_t difference)
{
  uint32_t absolute_difference = (difference < 0) ? (uint32_t)(-difference) : (uint32_t)difference;
  return ((float)absolute_difference * DIFF_VOLTAGE_SCALE) / MEASURE_CURRENT_MA;
}

static uint8_t Calibration_Fit(void)
{
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_xx = 0.0f;
  float sum_xy = 0.0f;
  float denominator;
  float count = (float)double_calibration.point_count;

  if (double_calibration.point_count < 2U)
  {
    return 0U;
  }

  for (uint32_t i = 0U; i < double_calibration.point_count; ++i)
  {
    float x = double_calibration.length_m[i];
    float y = double_calibration.resistance_ohm[i];
    sum_x += x;
    sum_y += y;
    sum_xx += x * x;
    sum_xy += x * y;
  }

  denominator = count * sum_xx - sum_x * sum_x;
  if ((denominator > -0.0001f) && (denominator < 0.0001f))
  {
    return 0U;
  }

  double_calibration.resistance_per_meter = (count * sum_xy - sum_x * sum_y) / denominator;
  double_calibration.zero_resistance = (sum_y - double_calibration.resistance_per_meter * sum_x) / count;

  return (double_calibration.resistance_per_meter > 0.000001f) ? 1U : 0U;
}

static uint8_t SingleCalibration_Fit(void)
{
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_xx = 0.0f;
  float sum_xy = 0.0f;
  float denominator;
  float count = (float)single_calibration.point_count;

  if (single_calibration.point_count < 2U)
  {
    return 0U;
  }

  for (uint32_t i = 0U; i < single_calibration.point_count; ++i)
  {
    float x = 1000000.0f / (float)single_calibration.frequency_hz[i];
    float y = single_calibration.length_m[i];
    sum_x += x;
    sum_y += y;
    sum_xx += x * x;
    sum_xy += x * y;
  }

  denominator = count * sum_xx - sum_x * sum_x;
  if ((denominator > -0.0001f) && (denominator < 0.0001f))
  {
    return 0U;
  }

  single_calibration.inverse_period_slope = (count * sum_xy - sum_x * sum_y) / denominator;
  single_calibration.offset_m = (sum_y - single_calibration.inverse_period_slope * sum_x) / count;

  return (single_calibration.inverse_period_slope > 0.000001f) ? 1U : 0U;
}

static float Calibration_ParseText(const char *text, uint8_t text_length)
{
  float value = 0.0f;
  float decimal_scale = 0.1f;
  uint8_t after_decimal = 0U;

  for (uint8_t i = 0U; i < text_length; ++i)
  {
    char character = text[i];

    if (character == '.')
    {
      after_decimal = 1U;
    }
    else if ((character >= '0') && (character <= '9'))
    {
      if (after_decimal == 0U)
      {
        value = value * 10.0f + (float)(character - '0');
      }
      else
      {
        value += (float)(character - '0') * decimal_scale;
        decimal_scale *= 0.1f;
      }
    }
  }

  return value;
}

static float Calibration_ParseValue(void)
{
  return Calibration_ParseText(calibration_value_input, calibration_value_input_length);
}

static float Calibration_ParseLength(void)
{
  return Calibration_ParseText(calibration_length_input, calibration_input_length);
}

static void Calibration_AppendKey(char key)
{
  char *input = (calibration_active_field == 0U) ? calibration_value_input : calibration_length_input;
  uint8_t *input_length = (calibration_active_field == 0U) ?
                          &calibration_value_input_length : &calibration_input_length;
  uint8_t input_capacity = (calibration_active_field == 0U) ?
                           (uint8_t)sizeof(calibration_value_input) : (uint8_t)sizeof(calibration_length_input);
  uint8_t has_decimal = 0U;

  calibration_status = 0U;

  if (key == '<')
  {
    if (*input_length != 0U)
    {
      (*input_length)--;
      input[*input_length] = '\0';
    }
    return;
  }

  if (key == '.')
  {
    for (uint8_t i = 0U; i < *input_length; ++i)
    {
      if (input[i] == '.')
      {
        has_decimal = 1U;
      }
    }

    if (has_decimal != 0U)
    {
      return;
    }

    if (*input_length == 0U)
    {
      input[(*input_length)++] = '0';
    }
  }

  if (*input_length < (input_capacity - 1U))
  {
    input[(*input_length)++] = key;
    input[*input_length] = '\0';
  }
}

static uint8_t Calibration_AddPoint(void)
{
  float measured_value = Calibration_ParseValue();
  float length_m = Calibration_ParseLength();

  if ((measured_value <= 0.0f) || (length_m <= 0.0f) || (length_m > 1000.0f))
  {
    return 0U;
  }

  if (calibration_mode == 0U)
  {
    if ((measured_value > 1000000.0f) || (single_calibration.point_count >= CALIBRATION_MAX_POINTS))
    {
      return 0U;
    }

    single_calibration.length_m[single_calibration.point_count] = length_m;
    single_calibration.frequency_hz[single_calibration.point_count] = (uint32_t)(measured_value + 0.5f);
    single_calibration.point_count++;

    calibration_selected_point = (uint8_t)(single_calibration.point_count - 1U);
  }
  else
  {
    if ((measured_value > 10000.0f) || (double_calibration.point_count >= CALIBRATION_MAX_POINTS))
    {
      return 0U;
    }

    double_calibration.length_m[double_calibration.point_count] = length_m;
    double_calibration.resistance_ohm[double_calibration.point_count] = measured_value;
    double_calibration.point_count++;

    calibration_selected_point = (uint8_t)(double_calibration.point_count - 1U);
  }

  calibration_value_input_length = 0U;
  calibration_value_input[0] = '\0';
  calibration_input_length = 0U;
  calibration_length_input[0] = '\0';
  return 1U;
}

static void Calibration_DeletePoint(void)
{
  if (calibration_mode == 0U)
  {
    if (single_calibration.point_count == 0U)
    {
      return;
    }

    for (uint32_t i = calibration_selected_point; i + 1U < single_calibration.point_count; ++i)
    {
      single_calibration.length_m[i] = single_calibration.length_m[i + 1U];
      single_calibration.frequency_hz[i] = single_calibration.frequency_hz[i + 1U];
    }
    single_calibration.point_count--;
    if (single_calibration.point_count >= 2U)
    {
      (void)SingleCalibration_Fit();
    }
  }
  else
  {
    if (double_calibration.point_count == 0U)
    {
      return;
    }

    for (uint32_t i = calibration_selected_point; i + 1U < double_calibration.point_count; ++i)
    {
      double_calibration.length_m[i] = double_calibration.length_m[i + 1U];
      double_calibration.resistance_ohm[i] = double_calibration.resistance_ohm[i + 1U];
    }
    double_calibration.point_count--;
    if (double_calibration.point_count >= 2U)
    {
      (void)Calibration_Fit();
    }
  }

  {
    uint32_t count = (calibration_mode == 0U) ? single_calibration.point_count : double_calibration.point_count;
    if (count == 0U)
    {
      calibration_selected_point = 0U;
    }
    else if (calibration_selected_point >= count)
    {
      calibration_selected_point = (uint8_t)(count - 1U);
    }
  }
}

static void Calibration_SelectPoint(int8_t direction)
{
  uint32_t count = (calibration_mode == 0U) ? single_calibration.point_count : double_calibration.point_count;

  if (count == 0U)
  {
    calibration_selected_point = 0U;
  }
  else if (direction < 0)
  {
    calibration_selected_point = (calibration_selected_point == 0U) ?
                                 (uint8_t)(count - 1U) : (uint8_t)(calibration_selected_point - 1U);
  }
  else
  {
    calibration_selected_point = (uint8_t)((calibration_selected_point + 1U) % count);
  }
}

static uint32_t Calibration_Checksum(const CalibrationStorage *data)
{
  const uint32_t *words = (const uint32_t *)data;
  uint32_t checksum = 2166136261UL;
  uint32_t word_count = (sizeof(CalibrationStorage) / sizeof(uint32_t)) - 1U;

  for (uint32_t i = 0U; i < word_count; ++i)
  {
    checksum ^= words[i];
    checksum *= 16777619UL;
  }

  return checksum;
}

static uint8_t Calibration_Save(void)
{
  CalibrationStorage storage = {0};
  CalibrationStorage verify = {0};

  if (spi_flash_ready == 0U)
  {
    return 0U;
  }

  if ((single_calibration.point_count >= 2U) && (SingleCalibration_Fit() == 0U))
  {
    single_calibration.inverse_period_slope = 0.0f;
    single_calibration.offset_m = 0.0f;
  }

  if ((double_calibration.point_count >= 2U) && (Calibration_Fit() == 0U))
  {
    double_calibration.resistance_per_meter = 0.0f;
    double_calibration.zero_resistance = 0.0f;
  }

  storage.magic = CALIBRATION_MAGIC;
  storage.version = CALIBRATION_VERSION;
  storage.single = single_calibration;
  storage.double_end = double_calibration;
  storage.checksum = Calibration_Checksum(&storage);

  if (SpiFlash_WriteSector(CALIBRATION_FLASH_ADDRESS, (const uint8_t *)&storage,
                           (uint16_t)sizeof(storage)) == 0U)
  {
    return 0U;
  }

  if (SpiFlash_Read(CALIBRATION_FLASH_ADDRESS, (uint8_t *)&verify, (uint16_t)sizeof(verify)) == 0U)
  {
    return 0U;
  }

  return ((verify.magic == CALIBRATION_MAGIC) &&
          (verify.checksum == Calibration_Checksum(&verify))) ? 1U : 0U;
}

static void Calibration_Load(void)
{
  CalibrationStorage stored = {0};

  spi_flash_ready = SpiFlash_Init();
  if (spi_flash_ready != 0U)
  {
    spi_flash_id = SpiFlash_ReadId();
    spi_flash_ready = ((spi_flash_id != 0U) && (spi_flash_id != 0xFFFFU)) ? 1U : 0U;
  }

  if ((spi_flash_ready == 0U) ||
      (SpiFlash_Read(CALIBRATION_FLASH_ADDRESS, (uint8_t *)&stored, (uint16_t)sizeof(stored)) == 0U))
  {
    return;
  }

  if ((stored.magic == CALIBRATION_MAGIC) &&
      (stored.version == CALIBRATION_VERSION) &&
      (stored.single.point_count <= CALIBRATION_MAX_POINTS) &&
      (stored.double_end.point_count <= CALIBRATION_MAX_POINTS) &&
      (stored.checksum == Calibration_Checksum(&stored)))
  {
    single_calibration = stored.single;
    double_calibration = stored.double_end;
  }
}

static uint8_t Calibration_GetLength(float resistance_ohm, float *length_m)
{
  if ((length_m == NULL) || (double_calibration.point_count < 2U) ||
      (double_calibration.resistance_per_meter <= 0.000001f))
  {
    return 0U;
  }

  *length_m = (resistance_ohm - double_calibration.zero_resistance) /
              double_calibration.resistance_per_meter;

  if (*length_m < 0.0f)
  {
    *length_m = 0.0f;
  }
  else if (*length_m > 1000.0f)
  {
    *length_m = 1000.0f;
  }

  return 1U;
}

static uint8_t SingleCalibration_GetLength(uint32_t frequency_hz, float *length_m)
{
  if ((length_m == NULL) || (frequency_hz == 0U) || (single_calibration.point_count < 2U) ||
      (single_calibration.inverse_period_slope <= 0.000001f))
  {
    return 0U;
  }

  *length_m = single_calibration.inverse_period_slope * (1000000.0f / (float)frequency_hz) +
              single_calibration.offset_m;
  if (*length_m < 0.0f)
  {
    *length_m = 0.0f;
  }
  else if (*length_m > 1000.0f)
  {
    *length_m = 1000.0f;
  }

  return 1U;
}

static void Ui_Init(void)
{
  LCD_Init();
  current_page = UI_PAGE_HOME;
  page_dirty = 1U;
}

static void Ui_Update(uint32_t now, uint16_t value1, uint16_t value2, uint32_t pa1_freq)
{
  const UiMeasurementResult *result;

  (void)now;
  (void)value1;
  (void)value2;
  (void)pa1_freq;

  /* 数值只在切页、开始测量或测量完成时重画，避免周期性清屏造成频闪。 */
  if (page_dirty == 0U)
  {
    return;
  }

  Ui_DrawPageStatic();
  page_dirty = 0U;

  if (measurement_running != 0U)
  {
    return;
  }

  result = &measurement_results[current_page];

  if (current_page == UI_PAGE_HOME)
  {
    Ui_DrawHomeValues(result);
  }
  else if (current_page == UI_PAGE_SINGLE)
  {
    Ui_DrawSingleValue(result->frequency_valid != 0U ? result->frequency_hz : 0U);
  }
  else if (current_page == UI_PAGE_DOUBLE)
  {
    Ui_DrawDoubleValue(result->valid != 0U ? result->difference : 0);
  }
}

static void Ui_HandleTouch(const TouchState *state)
{
  UiPage new_page = current_page;

  if (state == NULL)
  {
    return;
  }

  /* 只响应一次新的按下动作，避免手指按住时重复触发。 */
  if ((state->pressed != 0U) && (touch_was_pressed == 0U))
  {
    if ((current_page != UI_PAGE_CALIBRATION) &&
        (state->x >= UI_RUN_LEFT) && (state->x <= UI_RUN_RIGHT) &&
        (state->y >= UI_RUN_TOP) && (state->y <= UI_RUN_BOTTOM))
    {
      if (measurement_running == 0U)
      {
        Measurement_Start(HAL_GetTick());
      }
    }
    else if ((measurement_running == 0U) && (current_page == UI_PAGE_CALIBRATION) && (state->y < UI_NAV_TOP))
    {
      Ui_HandleCalibrationTouch(state->x, state->y);
    }
    else if ((measurement_running == 0U) && (state->y >= UI_NAV_TOP))
    {
      if (state->x <= UI_NAV_HOME_RIGHT)
      {
        new_page = UI_PAGE_HOME;
      }
      else if (state->x <= UI_NAV_SINGLE_RIGHT)
      {
        new_page = UI_PAGE_SINGLE;
      }
      else if (state->x <= UI_NAV_DOUBLE_RIGHT)
      {
        new_page = UI_PAGE_DOUBLE;
      }
      else
      {
        new_page = UI_PAGE_CALIBRATION;
      }

      if (new_page != current_page)
      {
        current_page = new_page;
        page_dirty = 1U;
      }
    }
  }

  touch_was_pressed = state->pressed;
}

static void Ui_DrawPageStatic(void)
{
  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
  LCD_Clear(WHITE);

  if (current_page == UI_PAGE_HOME)
  {
    LCD_ShowString(70, 12, 144, 24, 24, (uint8_t *)"MEASUREMENT");
    LCD_ShowString(52, 54, 72, 16, 16, (uint8_t *)"FREQ :");
    LCD_ShowString(52, 88, 72, 16, 16, (uint8_t *)"LENGTH:");
    LCD_ShowString(52, 122, 72, 16, 16, (uint8_t *)"DIFF :");
    LCD_ShowString(52, 156, 48, 16, 16, (uint8_t *)"RES :");
  }
  else if (current_page == UI_PAGE_SINGLE)
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
    LCD_ShowString(52, 52, 48, 16, 16, (uint8_t *)"DIFF:");
    LCD_ShowString(52, 84, 48, 16, 16, (uint8_t *)"VOLT:");
    LCD_ShowString(52, 116, 40, 16, 16, (uint8_t *)"RES:");
    LCD_ShowString(52, 148, 56, 16, 16, (uint8_t *)"LENGTH:");
  }
  else
  {
    Ui_DrawCalibrationPage();
  }

  if (current_page != UI_PAGE_CALIBRATION)
  {
    Ui_DrawRunButton();
  }
  Ui_DrawNavigation();
}

static void Ui_DrawRunButton(void)
{
  uint8_t is_running_page = (measurement_running != 0U) && (measurement_page == current_page);

  LCD_Fill(UI_RUN_LEFT, UI_RUN_TOP, UI_RUN_RIGHT, UI_RUN_BOTTOM, is_running_page != 0U ? RED : BLUE);
  POINT_COLOR = WHITE;
  BACK_COLOR = is_running_page != 0U ? RED : BLUE;

  if (is_running_page != 0U)
  {
    LCD_ShowString(252, 17, 48, 16, 16, (uint8_t *)"WAIT");
  }
  else
  {
    LCD_ShowString(262, 17, 24, 16, 16, (uint8_t *)"RUN");
  }

  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
}

static void Ui_DrawCalibrationPage(void)
{
  static const char keys[4][3] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'.', '0', '<'}
  };

  LCD_Fill(8, 8, 62, 30, calibration_mode == 0U ? BLUE : WHITE);
  POINT_COLOR = calibration_mode == 0U ? WHITE : BLACK;
  BACK_COLOR = calibration_mode == 0U ? BLUE : WHITE;
  LCD_ShowString(15, 12, 40, 12, 12, (uint8_t *)"S-CAL");
  LCD_Fill(68, 8, 128, 30, calibration_mode != 0U ? BLUE : WHITE);
  POINT_COLOR = calibration_mode != 0U ? WHITE : BLACK;
  BACK_COLOR = calibration_mode != 0U ? BLUE : WHITE;
  LCD_ShowString(78, 12, 40, 12, 12, (uint8_t *)"D-CAL");
  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
  LCD_ShowString(145, 10, 48, 12, 12,
                 (uint8_t *)(spi_flash_ready != 0U ? "FL OK" : "FL ERR"));
  LCD_ShowString(194, 10, 12, 12, 12, (uint8_t *)"I");
  LCD_ShowNum(208, 10, spi_flash_id, 5, 12);

  LCD_DrawRectangle(8, 140, 45, 164);
  LCD_ShowString(18, 146, 18, 12, 12, (uint8_t *)"ADD");
  LCD_DrawRectangle(49, 140, 86, 164);
  LCD_ShowString(59, 146, 18, 12, 12, (uint8_t *)"DEL");
  LCD_DrawRectangle(90, 140, 128, 164);
  LCD_ShowString(96, 146, 30, 12, 12, (uint8_t *)"SAVE");
  LCD_DrawRectangle(8, 170, 45, 194);
  LCD_ShowString(12, 176, 30, 12, 12, (uint8_t *)"PREV");
  LCD_DrawRectangle(49, 170, 86, 194);
  LCD_ShowString(53, 176, 30, 12, 12, (uint8_t *)"NEXT");
  LCD_DrawRectangle(90, 170, 128, 194);
  LCD_ShowString(100, 176, 18, 12, 12, (uint8_t *)"CLR");

  for (uint8_t row = 0U; row < 4U; ++row)
  {
    for (uint8_t col = 0U; col < 3U; ++col)
    {
      uint16_t left = CAL_KEYPAD_LEFT + (uint16_t)col * CAL_KEY_WIDTH;
      uint16_t top = CAL_KEYPAD_TOP + (uint16_t)row * CAL_KEY_HEIGHT;
      char label[2] = {keys[row][col], '\0'};

      LCD_DrawRectangle(left, top, left + CAL_KEY_WIDTH - 1U, top + CAL_KEY_HEIGHT - 1U);
      LCD_ShowString(left + 25U, top + 10U, 12, 16, 16, (uint8_t *)label);
    }
  }

  Ui_DrawCalibrationValues();
}

static void Ui_DrawCalibrationValues(void)
{
  uint32_t point_count = (calibration_mode == 0U) ? single_calibration.point_count : double_calibration.point_count;
  uint32_t value_x100;

  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
  LCD_Fill(8, 32, 136, 136, WHITE);

  POINT_COLOR = (calibration_active_field == 0U) ? BLUE : GRAY;
  LCD_DrawRectangle(5, 32, 132, 54);
  POINT_COLOR = (calibration_active_field == 1U) ? BLUE : GRAY;
  LCD_DrawRectangle(5, 56, 132, 78);
  POINT_COLOR = BLACK;

  if (calibration_mode == 0U)
  {
    LCD_ShowString(8, 34, 16, 16, 16, (uint8_t *)"F:");
  }
  else
  {
    LCD_ShowString(8, 34, 16, 16, 16, (uint8_t *)"R:");
  }

  if (calibration_value_input_length != 0U)
  {
    LCD_ShowString(28, 34, 80, 16, 16, (uint8_t *)calibration_value_input);
    LCD_ShowString(104, 34, 24, 16, 16,
                   (uint8_t *)(calibration_mode == 0U ? "Hz" : "ohm"));
  }
  else
  {
    LCD_ShowString(28, 34, 72, 16, 16,
                   (uint8_t *)(calibration_mode == 0U ? "---Hz" : "--ohm"));
  }

  LCD_ShowString(8, 58, 16, 16, 16, (uint8_t *)"L:");
  if (calibration_input_length != 0U)
  {
    LCD_ShowString(28, 58, 96, 16, 16, (uint8_t *)calibration_length_input);
    LCD_ShowString(112, 58, 8, 16, 16, (uint8_t *)"m");
  }
  else
  {
    LCD_ShowString(28, 58, 48, 16, 16, (uint8_t *)"--.-m");
  }

  LCD_ShowString(8, 82, 16, 16, 16, (uint8_t *)"P:");
  if (point_count != 0U)
  {
    LCD_ShowNum(28, 82, calibration_selected_point + 1U, 1, 16);
    LCD_ShowString(40, 82, 8, 16, 16, (uint8_t *)"/");
    LCD_ShowNum(48, 82, point_count, 1, 16);

    if (calibration_mode == 0U)
    {
      LCD_ShowString(8, 104, 18, 12, 12, (uint8_t *)"PF:");
      LCD_ShowNum(28, 104, single_calibration.frequency_hz[calibration_selected_point], 6, 12);
      LCD_ShowString(8, 120, 18, 12, 12, (uint8_t *)"PL:");
      value_x100 = (uint32_t)(single_calibration.length_m[calibration_selected_point] * 100.0f + 0.5f);
    }
    else
    {
      LCD_ShowString(8, 104, 18, 12, 12, (uint8_t *)"PR:");
      value_x100 = (uint32_t)(double_calibration.resistance_ohm[calibration_selected_point] * 100.0f + 0.5f);
      LCD_ShowNum(28, 104, value_x100 / 100U, 2, 12);
      LCD_ShowString(40, 104, 6, 12, 12, (uint8_t *)".");
      LCD_ShowxNum(46, 104, value_x100 % 100U, 2, 12, 0x80U);
      LCD_ShowString(8, 120, 18, 12, 12, (uint8_t *)"PL:");
      value_x100 = (uint32_t)(double_calibration.length_m[calibration_selected_point] * 100.0f + 0.5f);
    }

    LCD_ShowNum(28, 120, value_x100 / 100U, 3, 12);
    LCD_ShowString(46, 120, 6, 12, 12, (uint8_t *)".");
    LCD_ShowxNum(52, 120, value_x100 % 100U, 2, 12, 0x80U);
  }
  else
  {
    LCD_ShowString(28, 82, 18, 16, 16, (uint8_t *)"0/0");
  }

  if (calibration_status == 1U)
  {
    LCD_ShowString(92, 120, 36, 12, 12, (uint8_t *)"ADDED");
  }
  else if (calibration_status == 2U)
  {
    LCD_ShowString(92, 120, 36, 12, 12, (uint8_t *)"SAVED");
  }
  else if (calibration_status == 3U)
  {
    LCD_ShowString(92, 120, 24, 12, 12, (uint8_t *)"ERR");
  }
  else if (calibration_status == 4U)
  {
    LCD_ShowString(92, 120, 24, 12, 12, (uint8_t *)"NEED");
  }
  else if (calibration_status == 5U)
  {
    LCD_ShowString(92, 120, 30, 12, 12, (uint8_t *)"FLASH");
  }
}

static void Ui_HandleCalibrationTouch(uint16_t x, uint16_t y)
{
  static const char keys[4][3] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'.', '0', '<'}
  };

  if ((y >= 8U) && (y <= 30U) && (x >= 8U) && (x <= 128U))
  {
    calibration_mode = (x <= 62U) ? 0U : 1U;
    calibration_selected_point = 0U;
    calibration_active_field = 0U;
    calibration_value_input_length = 0U;
    calibration_value_input[0] = '\0';
    calibration_input_length = 0U;
    calibration_length_input[0] = '\0';
    calibration_status = 0U;
    page_dirty = 1U;
  }
  else if ((x >= 5U) && (x <= 132U) && (y >= 32U) && (y <= 54U))
  {
    calibration_active_field = 0U;
    Ui_DrawCalibrationValues();
  }
  else if ((x >= 5U) && (x <= 132U) && (y >= 56U) && (y <= 78U))
  {
    calibration_active_field = 1U;
    Ui_DrawCalibrationValues();
  }
  else if ((x >= CAL_KEYPAD_LEFT) && (y >= CAL_KEYPAD_TOP) && (y < 196U))
  {
    uint8_t col = (uint8_t)((x - CAL_KEYPAD_LEFT) / CAL_KEY_WIDTH);
    uint8_t row = (uint8_t)((y - CAL_KEYPAD_TOP) / CAL_KEY_HEIGHT);

    if ((col < 3U) && (row < 4U))
    {
      Calibration_AppendKey(keys[row][col]);
      Ui_DrawCalibrationValues();
    }
  }
  else if ((x >= 8U) && (x <= 45U) && (y >= 140U) && (y <= 164U))
  {
    if (Calibration_AddPoint() != 0U)
    {
      calibration_value_input_length = 0U;
      calibration_value_input[0] = '\0';
      calibration_input_length = 0U;
      calibration_length_input[0] = '\0';
      calibration_status = 1U;
    }
    else
    {
      calibration_status = 3U;
    }
    Ui_DrawCalibrationValues();
  }
  else if ((x >= 49U) && (x <= 86U) && (y >= 140U) && (y <= 164U))
  {
    Calibration_DeletePoint();
    calibration_status = 0U;
    Ui_DrawCalibrationValues();
  }
  else if ((x >= 90U) && (x <= 128U) && (y >= 140U) && (y <= 164U))
  {
    uint32_t count = (calibration_mode == 0U) ? single_calibration.point_count : double_calibration.point_count;
    if (count == 0U)
    {
      calibration_status = 4U;
    }
    else if (spi_flash_ready == 0U)
    {
      calibration_status = 5U;
    }
    else
    {
      calibration_status = Calibration_Save() != 0U ? 2U : 3U;
    }
    Ui_DrawCalibrationValues();
  }
  else if ((x >= 8U) && (x <= 45U) && (y >= 170U) && (y <= 194U))
  {
    Calibration_SelectPoint(-1);
    Ui_DrawCalibrationValues();
  }
  else if ((x >= 49U) && (x <= 86U) && (y >= 170U) && (y <= 194U))
  {
    Calibration_SelectPoint(1);
    Ui_DrawCalibrationValues();
  }
  else if ((x >= 90U) && (x <= 128U) && (y >= 170U) && (y <= 194U))
  {
    if (calibration_mode == 0U)
    {
      single_calibration.point_count = 0U;
      single_calibration.inverse_period_slope = 0.0f;
      single_calibration.offset_m = 0.0f;
    }
    else
    {
      double_calibration.point_count = 0U;
      double_calibration.resistance_per_meter = 0.0f;
      double_calibration.zero_resistance = 0.0f;
    }
    calibration_selected_point = 0U;
    calibration_value_input_length = 0U;
    calibration_value_input[0] = '\0';
    calibration_input_length = 0U;
    calibration_length_input[0] = '\0';
    calibration_status = 0U;
    Ui_DrawCalibrationValues();
  }
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
  else if (current_page == UI_PAGE_DOUBLE)
  {
    LCD_Fill(UI_NAV_SINGLE_RIGHT + 1U, UI_NAV_TOP, UI_NAV_DOUBLE_RIGHT, UI_NAV_BOTTOM, BLUE);
  }
  else
  {
    LCD_Fill(UI_NAV_DOUBLE_RIGHT + 1U, UI_NAV_TOP, 319, UI_NAV_BOTTOM, BLUE);
  }

  POINT_COLOR = GRAY;
  LCD_DrawRectangle(0, UI_NAV_TOP, 319, UI_NAV_BOTTOM);
  LCD_DrawLine(UI_NAV_HOME_RIGHT, UI_NAV_TOP, UI_NAV_HOME_RIGHT, UI_NAV_BOTTOM);
  LCD_DrawLine(UI_NAV_SINGLE_RIGHT, UI_NAV_TOP, UI_NAV_SINGLE_RIGHT, UI_NAV_BOTTOM);
  LCD_DrawLine(UI_NAV_DOUBLE_RIGHT, UI_NAV_TOP, UI_NAV_DOUBLE_RIGHT, UI_NAV_BOTTOM);

  POINT_COLOR = (current_page == UI_PAGE_HOME) ? WHITE : BLACK;
  BACK_COLOR = (current_page == UI_PAGE_HOME) ? BLUE : WHITE;
  LCD_ShowString(20, 212, 40, 16, 16, (uint8_t *)"HOME");

  POINT_COLOR = (current_page == UI_PAGE_SINGLE) ? WHITE : BLACK;
  BACK_COLOR = (current_page == UI_PAGE_SINGLE) ? BLUE : WHITE;
  LCD_ShowString(94, 212, 48, 16, 16, (uint8_t *)"SINGLE");

  POINT_COLOR = (current_page == UI_PAGE_DOUBLE) ? WHITE : BLACK;
  BACK_COLOR = (current_page == UI_PAGE_DOUBLE) ? BLUE : WHITE;
  LCD_ShowString(166, 212, 48, 16, 16, (uint8_t *)"DOUBLE");

  POINT_COLOR = (current_page == UI_PAGE_CALIBRATION) ? WHITE : BLACK;
  BACK_COLOR = (current_page == UI_PAGE_CALIBRATION) ? BLUE : WHITE;
  LCD_ShowString(268, 212, 24, 16, 16, (uint8_t *)"CAL");

  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
}

static void Ui_DrawHomeValues(const UiMeasurementResult *result)
{
  uint16_t length_x10;
  uint32_t absolute_difference;
  uint32_t resistance_x100;

  if ((result == NULL) || (result->valid == 0U))
  {
    return;
  }

  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;

  if (result->frequency_valid != 0U)
  {
    LCD_ShowNum(130, 54, result->frequency_hz, 6, 16);
    LCD_ShowString(182, 54, 16, 16, 16, (uint8_t *)"Hz");

    length_x10 = CalculateLengthX10(result->frequency_hz);
    LCD_ShowNum(130, 88, length_x10 / 10U, 2, 16);
    LCD_ShowString(146, 88, 8, 16, 16, (uint8_t *)".");
    LCD_ShowNum(154, 88, length_x10 % 10U, 1, 16);
    LCD_ShowString(166, 88, 8, 16, 16, (uint8_t *)"m");
  }
  else
  {
    LCD_ShowString(130, 54, 72, 16, 16, (uint8_t *)"--- Hz");
    LCD_ShowString(130, 88, 48, 16, 16, (uint8_t *)"--.-m");
  }

  Ui_DrawSignedValue(130, 122, result->difference, 16U);

  absolute_difference = (result->difference < 0) ? (uint32_t)(-result->difference) : (uint32_t)result->difference;
  resistance_x100 = (uint32_t)(((float)absolute_difference * DIFF_VOLTAGE_SCALE / MEASURE_CURRENT_MA) * 100.0f + 0.5f);
  LCD_ShowNum(130, 156, resistance_x100 / 100U, 2, 16);
  LCD_ShowString(146, 156, 8, 16, 16, (uint8_t *)".");
  LCD_ShowxNum(154, 156, resistance_x100 % 100U, 2, 16, 0x80U);
  LCD_ShowString(174, 156, 24, 16, 16, (uint8_t *)"ohm");
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
  float calibrated_length_m;
  uint32_t reciprocal_x1000;
  uint32_t length_x1000;

  if (frequency == 0U)
  {
    return LENGTH_INVALID_X10;
  }

  if (SingleCalibration_GetLength(frequency, &calibrated_length_m) != 0U)
  {
    uint32_t calibrated_x10 = (uint32_t)(calibrated_length_m * 10.0f + 0.5f);
    return (calibrated_x10 > LENGTH_MAX_X10) ? LENGTH_MAX_X10 : (uint16_t)calibrated_x10;
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

static void Ui_DrawDoubleValue(int32_t difference)
{
  uint32_t absolute_difference = (difference < 0) ? (uint32_t)(-difference) : (uint32_t)difference;
  float voltage_mv = (float)absolute_difference * DIFF_VOLTAGE_SCALE;
  float resistance_ohm = DifferenceToResistance(difference);
  float line_length_m = 0.0f;
  uint32_t voltage_display = (uint32_t)(voltage_mv + 0.5f);
  uint32_t resistance_x100 = (uint32_t)(resistance_ohm * 100.0f + 0.5f);
  uint32_t length_x10;

  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;

  /* DIFF 保留正负号，表示 ADC1 相对于 ADC2 的极性。 */
  Ui_DrawSignedValue(130, 52, difference, 16U);

  /* mV/mA 的数值关系等同于欧姆，因此可直接计算电阻。 */
  LCD_ShowNum(130, 84, voltage_display, 4, 16);
  LCD_ShowString(170, 84, 16, 16, 16, (uint8_t *)"mV");

  LCD_ShowNum(130, 116, resistance_x100 / 100U, 2, 16);
  LCD_ShowString(146, 116, 8, 16, 16, (uint8_t *)".");
  LCD_ShowxNum(154, 116, resistance_x100 % 100U, 2, 16, 0x80U);
  LCD_ShowString(174, 116, 24, 16, 16, (uint8_t *)"ohm");

  if (Calibration_GetLength(resistance_ohm, &line_length_m) != 0U)
  {
    length_x10 = (uint32_t)(line_length_m * 10.0f + 0.5f);
    LCD_ShowNum(130, 148, length_x10 / 10U, 4, 16);
    LCD_ShowString(162, 148, 8, 16, 16, (uint8_t *)".");
    LCD_ShowNum(170, 148, length_x10 % 10U, 1, 16);
    LCD_ShowString(182, 148, 8, 16, 16, (uint8_t *)"m");
  }
  else
  {
    LCD_ShowString(130, 148, 48, 16, 16, (uint8_t *)"--.-m");
  }
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
