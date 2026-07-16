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
#include "cable_test_manager.h"
#include "run_measurement.h"
#include "calibration_model.h"
#include "double_end_measurement.h"
#include "measurement_math.h"
#include "ad9954.h"
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

typedef enum
{
  CAL_AUTO_NONE = 0,
  CAL_AUTO_ZERO,
  CAL_AUTO_REFERENCE
} CalibrationAutoAction;

typedef struct
{
  uint8_t valid;
  uint8_t frequency_valid;
  uint32_t frequency_hz;
  int32_t difference;
  DoubleEndMeasurementResult double_end;
} UiMeasurementResult;

/* v2-v4使用的旧校准数组固定为5组，升级后用于迁移原有Flash数据。 */
#define CALIBRATION_LEGACY_MAX_POINTS 5U

typedef struct
{
  uint32_t point_count;
  float length_m[CALIBRATION_LEGACY_MAX_POINTS];
  float resistance_ohm[CALIBRATION_LEGACY_MAX_POINTS];
  float resistance_per_meter;
  float zero_resistance;
} LegacyDoubleCalibrationData;

typedef struct
{
  uint32_t point_count;
  float length_m[CALIBRATION_LEGACY_MAX_POINTS];
  uint32_t frequency_hz[CALIBRATION_LEGACY_MAX_POINTS];
  float inverse_period_slope;
  float offset_m;
} LegacySingleCalibrationData;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  SingleCalibrationData single;
  DoubleCalibrationData utp;
  DoubleCalibrationData sftp;
  DoubleFieldCalibrationData utp_field;
  DoubleFieldCalibrationData sftp_field;
  uint32_t checksum;
} CalibrationStorage;

/* 版本4：基础拟合与现场校准已经分开，但每种拟合最多保存5组。 */
typedef struct
{
  uint32_t magic;
  uint32_t version;
  LegacySingleCalibrationData single;
  LegacyDoubleCalibrationData utp;
  LegacyDoubleCalibrationData sftp;
  DoubleFieldCalibrationData utp_field;
  DoubleFieldCalibrationData sftp_field;
  uint32_t checksum;
} CalibrationStorageV4;

/* 版本3：已有UTP/SFTP基础拟合，但现场校准仍直接覆盖K/B。 */
typedef struct
{
  uint32_t magic;
  uint32_t version;
  LegacySingleCalibrationData single;
  LegacyDoubleCalibrationData utp;
  LegacyDoubleCalibrationData sftp;
  uint32_t checksum;
} CalibrationStorageV3;

/* 旧版 Flash 格式，用于升级时保留原来的一套双端校准数据。 */
typedef struct
{
  uint32_t magic;
  uint32_t version;
  LegacySingleCalibrationData single;
  LegacyDoubleCalibrationData double_end;
  uint32_t checksum;
} CalibrationStorageV2;

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
#define CALIBRATION_MAGIC 0x43414C32UL   /* Flash 校准数据标志：CAL2 */
#define CALIBRATION_VERSION 5UL
#define CALIBRATION_VERSION_PREVIOUS 4UL
#define CALIBRATION_VERSION_V3 3UL
#define CALIBRATION_VERSION_LEGACY 2UL
#define CALIBRATION_FLASH_ADDRESS 0x00FFF000UL /* NM25Q128 最后一个 4KB 扇区 */
#define UTP_REFERENCE_LENGTH_M 50.0f  /* 现场 UTP 参考网线长度 */
#define SFTP_REFERENCE_LENGTH_M 1.0f  /* 现场 SFTP 参考网线长度 */
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
static uint16_t adc3_input_value = 0U;                   /* ADC3_IN，PC0/ADC3_CH10 */
static uint16_t adc3_output_value = 0U;                  /* ADC3_OUT，PC1/ADC3_CH11 */
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
static volatile uint32_t adc1_frequency_hz = 0U;
static volatile uint32_t adc2_frequency_hz = 0U;
static TouchState touch_state = {0U, 0U, 0U, 0U, 0U};
static UiPage current_page = UI_PAGE_HOME;
static uint8_t page_dirty = 1U;                          /* 切页后需要重画静态内容 */
static uint8_t touch_was_pressed = 0U;                   /* 用于检测一次新的按下动作 */
static uint8_t measurement_running = 0U;
static UiPage measurement_page = UI_PAGE_HOME;
static RunMeasurement run_measurement;
static UiMeasurementResult measurement_results[4];
static DoubleCalibrationData double_calibrations[2];     /* 0=UTP，1=SFTP */
static DoubleFieldCalibrationData double_field_calibrations[2]; /* 独立现场校准参数 */
static SingleCalibrationData single_calibration;
static char calibration_value_input[10];
static uint8_t calibration_value_input_length = 0U;
static char calibration_length_input[8];
static uint8_t calibration_input_length = 0U;
static uint8_t calibration_active_field = 0U;             /* 0频率/电阻输入，1线长输入 */
static uint8_t calibration_status = 0U;                  /* 0空闲，1添加，2保存，3错误，4不足，5Flash，6零点，7参考完成，8等待，9重置 */
static uint8_t spi_flash_ready = 0U;
static uint16_t spi_flash_id = 0U;
static uint8_t calibration_mode = 0U;                    /* 0单端，1=UTP，2=SFTP */
static uint8_t calibration_view = 0U;                    /* 0多点数据管理，1现场参考线校准 */
static uint8_t calibration_selected_point = 0U;
static CalibrationAutoAction calibration_auto_action = CAL_AUTO_NONE;
static volatile uint8_t cable_scan_pending = 0U;
static CableTestManager cable_test_manager;
static CableTestResult cable_test_result;
static volatile uint8_t storage_save_pending = 0U;
static volatile uint8_t storage_save_busy = 0U;
static uint8_t storage_save_success_status = 2U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void AdcDacVofa_Start(void);
static uint16_t ReadAdcValue(ADC_HandleTypeDef *hadc, uint16_t previous_value);
static uint8_t ReadAdc3Pair(uint16_t *input_value, uint16_t *output_value);
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
static void Ui_DrawFieldCalibrationPage(void);
static void Ui_HandleCalibrationTouch(uint16_t x, uint16_t y);
static void Ui_HandleFieldCalibrationTouch(uint16_t x, uint16_t y);
static void Ui_DrawSingleValue(uint32_t pa1_freq);
static void Ui_DrawDoubleValue(const UiMeasurementResult *result);
static void Ui_DrawLargeFrequency(uint16_t x, uint16_t y, uint32_t freq);
static void Ui_DrawSignedValue(uint16_t x, uint16_t y, int32_t value, uint8_t size);
static uint16_t CalculateLengthX10(uint32_t frequency);
static void Ui_DrawLength(uint16_t x, uint16_t y, uint16_t length_x10);
static void Measurement_Start(uint32_t now);
static void Measurement_Update(uint32_t now, uint32_t pa1_freq, uint16_t value1, uint16_t value2);
static DoubleCalibrationData *Calibration_GetActiveDouble(void);
static const DoubleCalibrationData *Calibration_GetDoubleByShield(uint8_t shielded);
static DoubleFieldCalibrationData *Calibration_GetActiveField(void);
static const DoubleFieldCalibrationData *Calibration_GetFieldByShield(uint8_t shielded);
static float Calibration_ParseText(const char *text, uint8_t text_length);
static float Calibration_ParseValue(void);
static float Calibration_ParseLength(void);
static void Calibration_AppendKey(char key);
static uint8_t Calibration_AddPoint(void);
static void Calibration_DeletePoint(void);
static void Calibration_SelectPoint(int8_t direction);
static uint8_t Calibration_SaveNow(void);
static void Calibration_RequestSave(uint8_t success_status);
static void Calibration_Load(void);
static uint32_t Calibration_Checksum(const CalibrationStorage *data);
static uint32_t Calibration_ChecksumV4(const CalibrationStorageV4 *data);
static uint32_t Calibration_ChecksumV3(const CalibrationStorageV3 *data);
static uint32_t Calibration_ChecksumV2(const CalibrationStorageV2 *data);
static void Calibration_ImportLegacySingle(const LegacySingleCalibrationData *source,
                                           SingleCalibrationData *destination);
static void Calibration_ImportLegacyDouble(const LegacyDoubleCalibrationData *source,
                                           DoubleCalibrationData *destination);
static void Calibration_AutoStart(CalibrationAutoAction action);
static void Calibration_AutoComplete(int32_t difference);
void App_Init(void);
void App_SamplingTaskStep(void);
void App_MeasurementTaskStep(void);
void App_UiTaskStep(void);
void App_VofaTaskStep(void);
void App_StorageTaskStep(void);

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
  MX_ADC3_Init();
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

  /* LCD和触摸优先初始化，后续外设异常时屏幕也能先正常点亮。 */
  Ui_Init();
  Touch_Init();
  Calibration_Load();

  /* 频率直接填写Hz，幅度直接填写十进制mVpp。 */
  if ((Ad9954_Init() == 0U) ||
      (Ad9954_SetOutput(30000000UL, AD9954_MAX_AMPLITUDE_SCALE) == 0U))
  {
    Error_Handler();
  }

  AdcDacVofa_Start();

  if (HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void App_SamplingTaskStep(void)
{
  uint32_t now = HAL_GetTick();

  /* PA1 如果 1 秒没有捕获到新边沿，说明输入信号停止或频率太低，显示为 0Hz */
  if ((now - frequency_last_capture_tick) > FREQ_TIMEOUT_MS)
  {
    frequency_hz = 0U;
    pa1_kalman_ready = 0U;
  }

  /* ADC1、ADC2 分别采样并使用独立卡尔曼滤波器，避免两个通道相互影响。 */
  uint16_t adc1_sample = ReadAdcValue(&hadc1, adc1_value);
  uint16_t adc2_sample = ReadAdcValue(&hadc2, adc2_value);
  adc1_value = FilterAdcValue(&adc1_kalman, &adc1_kalman_ready, adc1_sample);
  adc2_value = FilterAdcValue(&adc2_kalman, &adc2_kalman_ready, adc2_sample);

  /* ADC3只在双端RUN期间读取，避免空闲时占用最高优先级采样任务。 */
  if ((measurement_running != 0U) && (measurement_page == UI_PAGE_DOUBLE))
  {
    (void)ReadAdc3Pair(&adc3_input_value, &adc3_output_value);
  }

  /* 用 ADC 采样值做软件测频。这个方法适合低频，采样间隔越稳定越准 */
  adc1_frequency_hz = AdcFreq_Update(&adc1_freq_meter, adc1_value, now);
  adc2_frequency_hz = AdcFreq_Update(&adc2_freq_meter, adc2_value, now);

  /* DAC 输出跟随 ADC：ADC1 -> DAC1(PA4)，ADC2 -> DAC2(PA5) */
  UpdateDacOutputs(adc1_value, adc2_value);
}

void App_MeasurementTaskStep(void)
{
  uint32_t now = HAL_GetTick();

  if (cable_scan_pending != 0U)
  {
    cable_scan_pending = 0U;
    CableTestManager_Start(&cable_test_manager, now);
  }

  if (cable_test_manager.running != 0U)
  {
    (void)CableTestManager_Process(&cable_test_manager, now, &cable_test_result);
  }

  /* RUN 测量按固定周期收集滤波后的数据，满 4 秒后计算中位数。 */
  Measurement_Update(now, frequency_hz, adc1_value, adc2_value);
}

void App_UiTaskStep(void)
{
  uint32_t now = HAL_GetTick();

  (void)Touch_Scan(&touch_state);
  Ui_HandleTouch(&touch_state);

  /* LCD 只由界面任务操作，避免多个任务同时访问 FSMC 和触摸驱动。 */
  Ui_Update(now, adc1_value, adc2_value, frequency_hz);
}

void App_VofaTaskStep(void)
{
  Vofa_SendSamples(adc1_value,
                   adc2_value,
                   frequency_hz,
                   adc1_frequency_hz,
                   adc2_frequency_hz);
}

void App_StorageTaskStep(void)
{
  uint8_t success_status;

  if (storage_save_pending == 0U)
  {
    return;
  }

  storage_save_busy = 1U;
  storage_save_pending = 0U;
  success_status = storage_save_success_status;
  calibration_status = (Calibration_SaveNow() != 0U) ? success_status :
                       ((spi_flash_ready == 0U) ? 5U : 3U);
  storage_save_busy = 0U;
  page_dirty = 1U;
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

static uint8_t ReadAdc3Pair(uint16_t *input_value, uint16_t *output_value)
{
  uint16_t input_sample;
  uint16_t output_sample;

  if ((input_value == NULL) || (output_value == NULL))
  {
    return 0U;
  }

  (void)HAL_ADC_Stop(&hadc3);
  __HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_OVR | ADC_FLAG_EOC);

  if (HAL_ADC_Start(&hadc3) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ADC_PollForConversion(&hadc3, 10U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc3);
    return 0U;
  }
  input_sample = (uint16_t)HAL_ADC_GetValue(&hadc3);

  if (HAL_ADC_PollForConversion(&hadc3, 10U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc3);
    return 0U;
  }
  output_sample = (uint16_t)HAL_ADC_GetValue(&hadc3);
  (void)HAL_ADC_Stop(&hadc3);

  *input_value = input_sample;
  *output_value = output_sample;
  return 1U;
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
  RunMeasurement_Start(&run_measurement, (uint8_t)measurement_page, now);
  measurement_results[measurement_page].valid = 0U;
  measurement_results[measurement_page].frequency_valid = 0U;
  measurement_results[measurement_page].double_end = (DoubleEndMeasurementResult){0};
  measurement_running = 1U;
  page_dirty = 1U;

  /* 线序扫描交给独立测量任务，避免阻塞触摸、LCD、ADC和VOFA任务。 */
  if (measurement_page == UI_PAGE_DOUBLE)
  {
    adc3_input_value = 0U;
    adc3_output_value = 0U;
    cable_test_manager = (CableTestManager){0};
    cable_test_result = (CableTestResult){0};
    cable_scan_pending = 1U;
  }
}

static void Measurement_Update(uint32_t now, uint32_t pa1_freq, uint16_t value1, uint16_t value2)
{
  RunMeasurementResult run_result = {0};
  UiMeasurementResult *result;

  if (measurement_running == 0U)
  {
    return;
  }

  if (RunMeasurement_Update(&run_measurement,
                            now,
                            pa1_freq,
                            value1,
                            value2,
                            adc3_input_value,
                            adc3_output_value,
                            &run_result) == 0U)
  {
    return;
  }

  result = &measurement_results[measurement_page];
  result->frequency_valid = run_result.frequency_valid;
  result->frequency_hz = run_result.frequency_hz;
  result->difference = run_result.difference;

  if (measurement_page == UI_PAGE_SINGLE)
  {
    result->valid = result->frequency_valid;
  }
  else if (measurement_page == UI_PAGE_DOUBLE)
  {
    result->valid = run_result.valid;
    DoubleEndMeasurement_Calculate(result->difference,
                                   &cable_test_result,
                                   Calibration_GetDoubleByShield(cable_test_result.shielded),
                                   Calibration_GetFieldByShield(cable_test_result.shielded),
                                   run_result.adc3_input,
                                   run_result.adc3_output,
                                   &result->double_end);
  }
  else
  {
    result->valid = run_result.valid;
  }

  if ((measurement_page == UI_PAGE_CALIBRATION) &&
           (calibration_auto_action != CAL_AUTO_NONE))
  {
    Calibration_AutoComplete(result->difference);
  }

  measurement_running = 0U;
  page_dirty = 1U;
}

static DoubleCalibrationData *Calibration_GetActiveDouble(void)
{
  if (calibration_mode == 1U)
  {
    return &double_calibrations[0];
  }
  if (calibration_mode == 2U)
  {
    return &double_calibrations[1];
  }

  return NULL;
}

static const DoubleCalibrationData *Calibration_GetDoubleByShield(uint8_t shielded)
{
  return &double_calibrations[(shielded != 0U) ? 1U : 0U];
}

static DoubleFieldCalibrationData *Calibration_GetActiveField(void)
{
  if (calibration_mode == 1U)
  {
    return &double_field_calibrations[0];
  }
  if (calibration_mode == 2U)
  {
    return &double_field_calibrations[1];
  }

  return NULL;
}

static const DoubleFieldCalibrationData *Calibration_GetFieldByShield(uint8_t shielded)
{
  return &double_field_calibrations[(shielded != 0U) ? 1U : 0U];
}

static void Calibration_AutoStart(CalibrationAutoAction action)
{
  if ((calibration_mode == 0U) || (action == CAL_AUTO_NONE) ||
      (measurement_running != 0U))
  {
    calibration_status = 3U;
    page_dirty = 1U;
    return;
  }

  calibration_auto_action = action;
  calibration_status = 8U;
  Measurement_Start(HAL_GetTick());
}

static void Calibration_AutoComplete(int32_t difference)
{
  DoubleCalibrationData *calibration = Calibration_GetActiveDouble();
  DoubleFieldCalibrationData *field_calibration = Calibration_GetActiveField();
  float measured_resistance;
  float reference_length;

  if ((calibration == NULL) || (field_calibration == NULL) ||
      (calibration_mode < 1U) || (calibration_mode > 2U))
  {
    calibration_status = 3U;
    calibration_auto_action = CAL_AUTO_NONE;
    return;
  }

  measured_resistance = MeasurementMath_DifferenceToResistance(difference,
                                                               DIFF_VOLTAGE_SCALE,
                                                               MEASURE_CURRENT_MA);

  if (calibration_auto_action == CAL_AUTO_ZERO)
  {
    /* ZERO只记录现场短接值，不再修改基础拟合的K/B。 */
    CalibrationModel_SetFieldZero(field_calibration, measured_resistance);
    Calibration_RequestSave(6U);
  }
  else if (calibration_auto_action == CAL_AUTO_REFERENCE)
  {
    reference_length = (calibration_mode == 1U) ?
                       UTP_REFERENCE_LENGTH_M : SFTP_REFERENCE_LENGTH_M;

    if (CalibrationModel_ApplyFieldReference(calibration,
                                             field_calibration,
                                             measured_resistance,
                                             reference_length) == 0U)
    {
      calibration_status = 4U;
      calibration_auto_action = CAL_AUTO_NONE;
      return;
    }

    Calibration_RequestSave(7U);
  }

  calibration_auto_action = CAL_AUTO_NONE;
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
  DoubleCalibrationData *double_calibration = Calibration_GetActiveDouble();
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
    if (single_calibration.point_count >= 2U)
    {
      (void)CalibrationModel_FitSingle(&single_calibration);
    }
  }
  else
  {
    if ((double_calibration == NULL) || (measured_value > 10000.0f) ||
        (double_calibration->point_count >= CALIBRATION_MAX_POINTS))
    {
      return 0U;
    }

    double_calibration->length_m[double_calibration->point_count] = length_m;
    double_calibration->resistance_ohm[double_calibration->point_count] = measured_value;
    double_calibration->point_count++;

    calibration_selected_point = (uint8_t)(double_calibration->point_count - 1U);
    if (double_calibration->point_count >= 2U)
    {
      (void)CalibrationModel_FitDouble(double_calibration);
    }

    /* 基础拟合点发生变化后，原来的现场修正参数已经不再匹配。 */
    CalibrationModel_ResetField(Calibration_GetActiveField());
  }

  calibration_value_input_length = 0U;
  calibration_value_input[0] = '\0';
  calibration_input_length = 0U;
  calibration_length_input[0] = '\0';
  return 1U;
}

static void Calibration_DeletePoint(void)
{
  DoubleCalibrationData *double_calibration = Calibration_GetActiveDouble();

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
      (void)CalibrationModel_FitSingle(&single_calibration);
    }
  }
  else
  {
    if ((double_calibration == NULL) || (double_calibration->point_count == 0U))
    {
      return;
    }

    for (uint32_t i = calibration_selected_point; i + 1U < double_calibration->point_count; ++i)
    {
      double_calibration->length_m[i] = double_calibration->length_m[i + 1U];
      double_calibration->resistance_ohm[i] = double_calibration->resistance_ohm[i + 1U];
    }
    double_calibration->point_count--;
    if (double_calibration->point_count >= 2U)
    {
      (void)CalibrationModel_FitDouble(double_calibration);
    }
    else
    {
      double_calibration->resistance_per_meter = 0.0f;
      double_calibration->zero_resistance = 0.0f;
    }

    /* 删除基础拟合点后，必须重新进行现场参考线校准。 */
    CalibrationModel_ResetField(Calibration_GetActiveField());
  }

  {
    uint32_t count = (calibration_mode == 0U) ? single_calibration.point_count :
                     ((double_calibration != NULL) ? double_calibration->point_count : 0U);
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
  DoubleCalibrationData *double_calibration = Calibration_GetActiveDouble();
  uint32_t count = (calibration_mode == 0U) ? single_calibration.point_count :
                   ((double_calibration != NULL) ? double_calibration->point_count : 0U);

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

static uint32_t Calibration_ChecksumV4(const CalibrationStorageV4 *data)
{
  const uint32_t *words = (const uint32_t *)data;
  uint32_t checksum = 2166136261UL;
  uint32_t word_count = (sizeof(CalibrationStorageV4) / sizeof(uint32_t)) - 1U;

  for (uint32_t i = 0U; i < word_count; ++i)
  {
    checksum ^= words[i];
    checksum *= 16777619UL;
  }

  return checksum;
}

static uint32_t Calibration_ChecksumV3(const CalibrationStorageV3 *data)
{
  const uint32_t *words = (const uint32_t *)data;
  uint32_t checksum = 2166136261UL;
  uint32_t word_count = (sizeof(CalibrationStorageV3) / sizeof(uint32_t)) - 1U;

  for (uint32_t i = 0U; i < word_count; ++i)
  {
    checksum ^= words[i];
    checksum *= 16777619UL;
  }

  return checksum;
}

static uint32_t Calibration_ChecksumV2(const CalibrationStorageV2 *data)
{
  const uint32_t *words = (const uint32_t *)data;
  uint32_t checksum = 2166136261UL;
  uint32_t word_count = (sizeof(CalibrationStorageV2) / sizeof(uint32_t)) - 1U;

  for (uint32_t i = 0U; i < word_count; ++i)
  {
    checksum ^= words[i];
    checksum *= 16777619UL;
  }

  return checksum;
}

static void Calibration_ImportLegacySingle(const LegacySingleCalibrationData *source,
                                           SingleCalibrationData *destination)
{
  if ((source == NULL) || (destination == NULL) ||
      (source->point_count > CALIBRATION_LEGACY_MAX_POINTS))
  {
    return;
  }

  *destination = (SingleCalibrationData){0};
  destination->point_count = source->point_count;
  destination->inverse_period_slope = source->inverse_period_slope;
  destination->offset_m = source->offset_m;

  for (uint32_t i = 0U; i < source->point_count; ++i)
  {
    destination->length_m[i] = source->length_m[i];
    destination->frequency_hz[i] = source->frequency_hz[i];
  }
}

static void Calibration_ImportLegacyDouble(const LegacyDoubleCalibrationData *source,
                                           DoubleCalibrationData *destination)
{
  if ((source == NULL) || (destination == NULL) ||
      (source->point_count > CALIBRATION_LEGACY_MAX_POINTS))
  {
    return;
  }

  *destination = (DoubleCalibrationData){0};
  destination->point_count = source->point_count;
  destination->resistance_per_meter = source->resistance_per_meter;
  destination->zero_resistance = source->zero_resistance;

  for (uint32_t i = 0U; i < source->point_count; ++i)
  {
    destination->length_m[i] = source->length_m[i];
    destination->resistance_ohm[i] = source->resistance_ohm[i];
  }
}

static void Calibration_RequestSave(uint8_t success_status)
{
  if (spi_flash_ready == 0U)
  {
    calibration_status = 5U;
    page_dirty = 1U;
    return;
  }

  storage_save_success_status = success_status;
  storage_save_pending = 1U;
  calibration_status = 8U;
  page_dirty = 1U;
}

static uint8_t Calibration_SaveNow(void)
{
  CalibrationStorage storage = {0};
  CalibrationStorage verify = {0};

  if (spi_flash_ready == 0U)
  {
    return 0U;
  }

  storage.magic = CALIBRATION_MAGIC;
  storage.version = CALIBRATION_VERSION;
  storage.single = single_calibration;
  storage.utp = double_calibrations[0];
  storage.sftp = double_calibrations[1];
  storage.utp_field = double_field_calibrations[0];
  storage.sftp_field = double_field_calibrations[1];
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
          (verify.version == CALIBRATION_VERSION) &&
          (verify.checksum == Calibration_Checksum(&verify))) ? 1U : 0U;
}

static void Calibration_Load(void)
{
  uint32_t header[2] = {0U, 0U};
  CalibrationStorage stored = {0};
  CalibrationStorageV4 previous = {0};
  CalibrationStorageV3 version3 = {0};
  CalibrationStorageV2 legacy = {0};

  spi_flash_ready = SpiFlash_Init();
  if (spi_flash_ready != 0U)
  {
    spi_flash_id = SpiFlash_ReadId();
    spi_flash_ready = ((spi_flash_id != 0U) && (spi_flash_id != 0xFFFFU)) ? 1U : 0U;
  }

  if ((spi_flash_ready == 0U) ||
      (SpiFlash_Read(CALIBRATION_FLASH_ADDRESS, (uint8_t *)header, (uint16_t)sizeof(header)) == 0U) ||
      (header[0] != CALIBRATION_MAGIC))
  {
    return;
  }

  if ((header[1] == CALIBRATION_VERSION) &&
      (SpiFlash_Read(CALIBRATION_FLASH_ADDRESS, (uint8_t *)&stored, (uint16_t)sizeof(stored)) != 0U) &&
      (stored.single.point_count <= CALIBRATION_MAX_POINTS) &&
      (stored.utp.point_count <= CALIBRATION_MAX_POINTS) &&
      (stored.sftp.point_count <= CALIBRATION_MAX_POINTS) &&
      (stored.checksum == Calibration_Checksum(&stored)))
  {
    single_calibration = stored.single;
    double_calibrations[0] = stored.utp;
    double_calibrations[1] = stored.sftp;
    double_field_calibrations[0] = stored.utp_field;
    double_field_calibrations[1] = stored.sftp_field;
  }
  else if ((header[1] == CALIBRATION_VERSION_PREVIOUS) &&
           (SpiFlash_Read(CALIBRATION_FLASH_ADDRESS, (uint8_t *)&previous, (uint16_t)sizeof(previous)) != 0U) &&
           (previous.single.point_count <= CALIBRATION_LEGACY_MAX_POINTS) &&
           (previous.utp.point_count <= CALIBRATION_LEGACY_MAX_POINTS) &&
           (previous.sftp.point_count <= CALIBRATION_LEGACY_MAX_POINTS) &&
           (previous.checksum == Calibration_ChecksumV4(&previous)))
  {
    /* v4的5组数据扩展到当前10组结构，现场校准参数保持不变。 */
    Calibration_ImportLegacySingle(&previous.single, &single_calibration);
    Calibration_ImportLegacyDouble(&previous.utp, &double_calibrations[0]);
    Calibration_ImportLegacyDouble(&previous.sftp, &double_calibrations[1]);
    double_field_calibrations[0] = previous.utp_field;
    double_field_calibrations[1] = previous.sftp_field;
  }
  else if ((header[1] == CALIBRATION_VERSION_V3) &&
           (SpiFlash_Read(CALIBRATION_FLASH_ADDRESS, (uint8_t *)&version3, (uint16_t)sizeof(version3)) != 0U) &&
           (version3.single.point_count <= CALIBRATION_LEGACY_MAX_POINTS) &&
           (version3.utp.point_count <= CALIBRATION_LEGACY_MAX_POINTS) &&
           (version3.sftp.point_count <= CALIBRATION_LEGACY_MAX_POINTS) &&
           (version3.checksum == Calibration_ChecksumV3(&version3)))
  {
    /* v3没有独立现场层：保留基础参数，现场校准从未启用状态开始。 */
    Calibration_ImportLegacySingle(&version3.single, &single_calibration);
    Calibration_ImportLegacyDouble(&version3.utp, &double_calibrations[0]);
    Calibration_ImportLegacyDouble(&version3.sftp, &double_calibrations[1]);
    CalibrationModel_ResetField(&double_field_calibrations[0]);
    CalibrationModel_ResetField(&double_field_calibrations[1]);
  }
  else if ((header[1] == CALIBRATION_VERSION_LEGACY) &&
           (SpiFlash_Read(CALIBRATION_FLASH_ADDRESS, (uint8_t *)&legacy, (uint16_t)sizeof(legacy)) != 0U) &&
           (legacy.single.point_count <= CALIBRATION_LEGACY_MAX_POINTS) &&
           (legacy.double_end.point_count <= CALIBRATION_LEGACY_MAX_POINTS) &&
           (legacy.checksum == Calibration_ChecksumV2(&legacy)))
  {
    /* 旧版只有一套双端参数，先复制给两种类型，之后可分别重新校准。 */
    Calibration_ImportLegacySingle(&legacy.single, &single_calibration);
    Calibration_ImportLegacyDouble(&legacy.double_end, &double_calibrations[0]);
    Calibration_ImportLegacyDouble(&legacy.double_end, &double_calibrations[1]);
    CalibrationModel_ResetField(&double_field_calibrations[0]);
    CalibrationModel_ResetField(&double_field_calibrations[1]);
  }
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

  /* 先清标志再绘制；其他任务在绘制期间置 1 时，下一轮仍会继续刷新。 */
  page_dirty = 0U;
  Ui_DrawPageStatic();

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
    Ui_DrawDoubleValue(result);
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
    LCD_ShowString(38, 44, 48, 16, 16, (uint8_t *)"DIFF:");
    LCD_ShowString(38, 66, 48, 16, 16, (uint8_t *)"VOLT:");
    LCD_ShowString(38, 88, 40, 16, 16, (uint8_t *)"RES:");
    LCD_ShowString(38, 110, 56, 16, 16, (uint8_t *)"LENGTH:");
    LCD_ShowString(38, 132, 48, 16, 16, (uint8_t *)"ATTEN:");
    LCD_ShowString(38, 154, 40, 16, 16, (uint8_t *)"TYPE:");
    LCD_ShowString(38, 176, 40, 16, 16, (uint8_t *)"WIRE:");
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

  if (calibration_view != 0U)
  {
    Ui_DrawFieldCalibrationPage();
    return;
  }

  LCD_Fill(5, 8, 42, 30, calibration_mode == 0U ? BLUE : WHITE);
  POINT_COLOR = calibration_mode == 0U ? WHITE : BLACK;
  BACK_COLOR = calibration_mode == 0U ? BLUE : WHITE;
  LCD_ShowString(9, 12, 30, 12, 12, (uint8_t *)"S-CAL");
  LCD_Fill(46, 8, 86, 30, calibration_mode == 1U ? BLUE : WHITE);
  POINT_COLOR = calibration_mode == 1U ? WHITE : BLACK;
  BACK_COLOR = calibration_mode == 1U ? BLUE : WHITE;
  LCD_ShowString(57, 12, 18, 12, 12, (uint8_t *)"UTP");
  LCD_Fill(90, 8, 134, 30, calibration_mode == 2U ? BLUE : WHITE);
  POINT_COLOR = calibration_mode == 2U ? WHITE : BLACK;
  BACK_COLOR = calibration_mode == 2U ? BLUE : WHITE;
  LCD_ShowString(100, 12, 24, 12, 12, (uint8_t *)"SFTP");
  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
  LCD_ShowString(145, 10, 48, 12, 12,
                 (uint8_t *)(spi_flash_ready != 0U ? "FL OK" : "FL ERR"));
  LCD_ShowString(194, 10, 12, 12, 12, (uint8_t *)"I");
  LCD_ShowNum(208, 10, spi_flash_id, 5, 12);
  LCD_DrawRectangle(260, 8, 315, 30);
  LCD_ShowString(270, 13, 36, 12, 12, (uint8_t *)"FIELD");

  LCD_DrawRectangle(5, 140, 35, 164);
  LCD_ShowString(11, 146, 18, 12, 12, (uint8_t *)"ADD");
  LCD_DrawRectangle(38, 140, 68, 164);
  LCD_ShowString(44, 146, 18, 12, 12, (uint8_t *)"DEL");
  LCD_DrawRectangle(71, 140, 101, 164);
  LCD_ShowString(74, 146, 24, 12, 12, (uint8_t *)"SAVE");
  LCD_DrawRectangle(104, 140, 134, 164);
  LCD_ShowString(110, 146, 18, 12, 12, (uint8_t *)"CLR");
  LCD_DrawRectangle(5, 170, 35, 194);
  LCD_ShowString(8, 176, 24, 12, 12, (uint8_t *)"PREV");
  LCD_DrawRectangle(38, 170, 68, 194);
  LCD_ShowString(41, 176, 24, 12, 12, (uint8_t *)"NEXT");

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

static void Ui_DrawFieldCalibrationPage(void)
{
  DoubleCalibrationData *calibration;
  DoubleFieldCalibrationData *field_calibration;
  uint8_t zero_ready;
  uint8_t zero_running;
  uint8_t reference_running;

  if ((calibration_mode != 1U) && (calibration_mode != 2U))
  {
    calibration_mode = 1U;
  }

  calibration = Calibration_GetActiveDouble();
  field_calibration = Calibration_GetActiveField();
  zero_ready = ((field_calibration != NULL) &&
                (field_calibration->pending_zero_valid != 0U)) ? 1U : 0U;
  zero_running = ((measurement_running != 0U) &&
                  (calibration_auto_action == CAL_AUTO_ZERO)) ? 1U : 0U;
  reference_running = ((measurement_running != 0U) &&
                       (calibration_auto_action == CAL_AUTO_REFERENCE)) ? 1U : 0U;

  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;

  LCD_DrawRectangle(8, 8, 62, 32);
  LCD_ShowString(19, 14, 30, 12, 12, (uint8_t *)"DATA");
  LCD_ShowString(92, 10, 108, 24, 24, (uint8_t *)"FIELD CAL");
  LCD_ShowString(250, 10, 36, 12, 12,
                 (uint8_t *)(spi_flash_ready != 0U ? "FL OK" : "FL ERR"));

  LCD_Fill(36, 44, 154, 72, calibration_mode == 1U ? BLUE : WHITE);
  POINT_COLOR = calibration_mode == 1U ? WHITE : BLACK;
  BACK_COLOR = calibration_mode == 1U ? BLUE : WHITE;
  LCD_DrawRectangle(36, 44, 154, 72);
  LCD_ShowString(82, 51, 24, 16, 16, (uint8_t *)"UTP");

  LCD_Fill(166, 44, 284, 72, calibration_mode == 2U ? BLUE : WHITE);
  POINT_COLOR = calibration_mode == 2U ? WHITE : BLACK;
  BACK_COLOR = calibration_mode == 2U ? BLUE : WHITE;
  LCD_DrawRectangle(166, 44, 284, 72);
  LCD_ShowString(210, 51, 32, 16, 16, (uint8_t *)"SFTP");

  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
  LCD_ShowString(48, 86, 80, 16, 16, (uint8_t *)"REFERENCE:");
  if (calibration_mode == 1U)
  {
    LCD_ShowString(154, 86, 48, 16, 16, (uint8_t *)"50.0m");
  }
  else
  {
    LCD_ShowString(154, 86, 40, 16, 16, (uint8_t *)"1.0m");
  }

  LCD_ShowString(48, 110, 48, 16, 16, (uint8_t *)"MODEL:");
  LCD_ShowString(154, 110, 72, 16, 16,
                 (uint8_t *)((calibration != NULL) &&
                             (calibration->resistance_per_meter > 0.000001f) ? "READY" : "EMPTY"));

  LCD_DrawRectangle(238, 104, 306, 128);
  LCD_ShowString(248, 110, 48, 12, 12, (uint8_t *)"RESET");

  LCD_ShowString(48, 134, 48, 16, 16, (uint8_t *)"FIELD:");
  if (calibration_status == 8U)
  {
    LCD_ShowString(154, 134, 56, 16, 16, (uint8_t *)"WAIT...");
  }
  else if (calibration_status == 7U)
  {
    LCD_ShowString(154, 134, 48, 16, 16, (uint8_t *)"CAL OK");
  }
  else if (calibration_status == 6U)
  {
    LCD_ShowString(154, 134, 56, 16, 16, (uint8_t *)"ZERO OK");
  }
  else if (calibration_status == 5U)
  {
    LCD_ShowString(154, 134, 72, 16, 16, (uint8_t *)"FLASH ERR");
  }
  else if (calibration_status == 4U)
  {
    LCD_ShowString(154, 134, 80, 16, 16, (uint8_t *)"NEED MODEL");
  }
  else if (calibration_status == 3U)
  {
    LCD_ShowString(154, 134, 40, 16, 16, (uint8_t *)"ERROR");
  }
  else if (calibration_status == 9U)
  {
    LCD_ShowString(154, 134, 64, 16, 16, (uint8_t *)"RESET OK");
  }
  else if (zero_ready != 0U)
  {
    LCD_ShowString(154, 134, 80, 16, 16, (uint8_t *)"ZERO READY");
  }
  else
  {
    LCD_ShowString(154, 134, 24, 16, 16,
                   (uint8_t *)((field_calibration != NULL) &&
                               (field_calibration->valid != 0U) ? "ON" : "OFF"));
  }

  LCD_Fill(24, 160, 138, 194, zero_running != 0U ? RED : BLUE);
  POINT_COLOR = WHITE;
  BACK_COLOR = zero_running != 0U ? RED : BLUE;
  LCD_ShowString(62, 170, 32, 16, 16,
                 (uint8_t *)(zero_running != 0U ? "WAIT" : "ZERO"));

  LCD_Fill(156, 160, 296, 194, reference_running != 0U ? RED : BLUE);
  POINT_COLOR = WHITE;
  BACK_COLOR = reference_running != 0U ? RED : BLUE;
  LCD_ShowString(194, 170, 56, 16, 16,
                 (uint8_t *)(reference_running != 0U ? "WAIT" : "REF CAL"));

  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;
}

static void Ui_DrawCalibrationValues(void)
{
  DoubleCalibrationData *double_calibration = Calibration_GetActiveDouble();
  uint32_t point_count = (calibration_mode == 0U) ? single_calibration.point_count :
                         ((double_calibration != NULL) ? double_calibration->point_count : 0U);
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
  if (calibration_mode == 1U)
  {
    LCD_ShowString(76, 82, 48, 16, 16, (uint8_t *)"REF50m");
  }
  else if (calibration_mode == 2U)
  {
    LCD_ShowString(76, 82, 40, 16, 16, (uint8_t *)"REF1m");
  }
  if (point_count != 0U)
  {
    /* 现在最多支持10组拟合点，序号和总数都按两位宽度显示。 */
    LCD_ShowNum(20, 82, calibration_selected_point + 1U, 2, 16);
    LCD_ShowString(40, 82, 8, 16, 16, (uint8_t *)"/");
    LCD_ShowNum(48, 82, point_count, 2, 16);

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
      value_x100 = (uint32_t)(double_calibration->resistance_ohm[calibration_selected_point] * 100.0f + 0.5f);
      LCD_ShowNum(28, 104, value_x100 / 100U, 2, 12);
      LCD_ShowString(40, 104, 6, 12, 12, (uint8_t *)".");
      LCD_ShowxNum(46, 104, value_x100 % 100U, 2, 12, 0x80U);
      LCD_ShowString(8, 120, 18, 12, 12, (uint8_t *)"PL:");
      value_x100 = (uint32_t)(double_calibration->length_m[calibration_selected_point] * 100.0f + 0.5f);
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
  else if (calibration_status == 6U)
  {
    LCD_ShowString(92, 120, 24, 12, 12, (uint8_t *)"ZERO");
  }
  else if (calibration_status == 7U)
  {
    LCD_ShowString(86, 120, 42, 12, 12, (uint8_t *)"CAL OK");
  }
  else if (calibration_status == 8U)
  {
    LCD_ShowString(92, 120, 24, 12, 12, (uint8_t *)"WAIT");
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

  /* Flash 保存期间冻结校准数据编辑，避免存储任务读取到一半更新的数据。 */
  if ((storage_save_pending != 0U) || (storage_save_busy != 0U))
  {
    return;
  }

  if (calibration_view != 0U)
  {
    Ui_HandleFieldCalibrationTouch(x, y);
    return;
  }

  if ((x >= 260U) && (x <= 315U) && (y >= 8U) && (y <= 30U))
  {
    if (calibration_mode == 0U)
    {
      calibration_mode = 1U;
    }
    calibration_view = 1U;
    calibration_status = 0U;
    page_dirty = 1U;
  }
  else if ((y >= 8U) && (y <= 30U) && (x >= 5U) && (x <= 134U))
  {
    if (x <= 42U)
    {
      calibration_mode = 0U;
    }
    else if ((x >= 46U) && (x <= 86U))
    {
      calibration_mode = 1U;
    }
    else if (x >= 90U)
    {
      calibration_mode = 2U;
    }
    else
    {
      return;
    }
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
  else if ((x >= 5U) && (x <= 35U) && (y >= 140U) && (y <= 164U))
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
  else if ((x >= 38U) && (x <= 68U) && (y >= 140U) && (y <= 164U))
  {
    Calibration_DeletePoint();
    calibration_status = 0U;
    Ui_DrawCalibrationValues();
  }
  else if ((x >= 71U) && (x <= 101U) && (y >= 140U) && (y <= 164U))
  {
    DoubleCalibrationData *double_calibration = Calibration_GetActiveDouble();
    if ((calibration_mode == 0U) && (single_calibration.point_count == 0U))
    {
      calibration_status = 4U;
    }
    else if ((calibration_mode != 0U) && (double_calibration == NULL))
    {
      calibration_status = 4U;
    }
    else if (spi_flash_ready == 0U)
    {
      calibration_status = 5U;
    }
    else
    {
      Calibration_RequestSave(2U);
    }
    Ui_DrawCalibrationValues();
  }
  else if ((x >= 104U) && (x <= 134U) && (y >= 140U) && (y <= 164U))
  {
    DoubleCalibrationData *double_calibration = Calibration_GetActiveDouble();

    if (calibration_mode == 0U)
    {
      single_calibration.point_count = 0U;
      single_calibration.inverse_period_slope = 0.0f;
      single_calibration.offset_m = 0.0f;
    }
    else if (double_calibration != NULL)
    {
      double_calibration->point_count = 0U;
      double_calibration->resistance_per_meter = 0.0f;
      double_calibration->zero_resistance = 0.0f;
      CalibrationModel_ResetField(Calibration_GetActiveField());
    }
    calibration_selected_point = 0U;
    calibration_value_input_length = 0U;
    calibration_value_input[0] = '\0';
    calibration_input_length = 0U;
    calibration_length_input[0] = '\0';
    calibration_status = 0U;
    Ui_DrawCalibrationValues();
  }
  else if ((x >= 5U) && (x <= 35U) && (y >= 170U) && (y <= 194U))
  {
    Calibration_SelectPoint(-1);
    Ui_DrawCalibrationValues();
  }
  else if ((x >= 38U) && (x <= 68U) && (y >= 170U) && (y <= 194U))
  {
    Calibration_SelectPoint(1);
    Ui_DrawCalibrationValues();
  }
}

static void Ui_HandleFieldCalibrationTouch(uint16_t x, uint16_t y)
{
  if ((x >= 8U) && (x <= 62U) && (y >= 8U) && (y <= 32U))
  {
    calibration_view = 0U;
    calibration_status = 0U;
    page_dirty = 1U;
  }
  else if ((x >= 36U) && (x <= 154U) && (y >= 44U) && (y <= 72U))
  {
    calibration_mode = 1U;
    calibration_status = 0U;
    page_dirty = 1U;
  }
  else if ((x >= 166U) && (x <= 284U) && (y >= 44U) && (y <= 72U))
  {
    calibration_mode = 2U;
    calibration_status = 0U;
    page_dirty = 1U;
  }
  else if ((x >= 238U) && (x <= 306U) && (y >= 104U) && (y <= 128U))
  {
    /* RESET只清除当前线型的现场修正，基础多点拟合数据保持不变。 */
    CalibrationModel_ResetField(Calibration_GetActiveField());
    Calibration_RequestSave(9U);
  }
  else if ((x >= 24U) && (x <= 138U) && (y >= 160U) && (y <= 194U))
  {
    Calibration_AutoStart(CAL_AUTO_ZERO);
  }
  else if ((x >= 156U) && (x <= 296U) && (y >= 160U) && (y <= 194U))
  {
    Calibration_AutoStart(CAL_AUTO_REFERENCE);
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

  if (frequency == 0U)
  {
    return 0U;
  }

  if (CalibrationModel_GetSingleLength(&single_calibration,
                                       frequency,
                                       &calibrated_length_m) != 0U)
  {
    uint32_t calibrated_x10 = (uint32_t)(calibrated_length_m * 10.0f + 0.5f);
    return (calibrated_x10 > LENGTH_MAX_X10) ? LENGTH_MAX_X10 : (uint16_t)calibrated_x10;
  }

  return MeasurementMath_ReciprocalLengthX10(frequency,
                                             LENGTH_SCALE_X1000,
                                             LENGTH_OFFSET_X1000,
                                             LENGTH_MAX_X10,
                                             LENGTH_INVALID_X10);
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

static void Ui_DrawDoubleValue(const UiMeasurementResult *result)
{
  const DoubleEndMeasurementResult *double_result;
  uint32_t voltage_display;
  uint32_t resistance_x100;
  uint32_t length_x10;
  int32_t attenuation_x100;
  uint32_t attenuation_abs_x100;

  if ((result == NULL) || (result->valid == 0U) || (result->double_end.valid == 0U))
  {
    return;
  }

  double_result = &result->double_end;
  voltage_display = (uint32_t)(double_result->voltage_mv + 0.5f);
  resistance_x100 = (uint32_t)(double_result->resistance_ohm * 100.0f + 0.5f);

  POINT_COLOR = BLACK;
  BACK_COLOR = WHITE;

  /* DIFF 保留正负号，表示 ADC1 相对于 ADC2 的极性。 */
  Ui_DrawSignedValue(118, 44, double_result->difference, 16U);

  /* mV/mA 的数值关系等同于欧姆，因此可直接计算电阻。 */
  LCD_ShowNum(118, 66, voltage_display, 4, 16);
  LCD_ShowString(158, 66, 16, 16, 16, (uint8_t *)"mV");

  LCD_ShowNum(118, 88, resistance_x100 / 100U, 2, 16);
  LCD_ShowString(134, 88, 8, 16, 16, (uint8_t *)".");
  LCD_ShowxNum(142, 88, resistance_x100 % 100U, 2, 16, 0x80U);
  LCD_ShowString(162, 88, 24, 16, 16, (uint8_t *)"ohm");

  if (double_result->length_valid != 0U)
  {
    length_x10 = (uint32_t)(double_result->length_m * 10.0f + 0.5f);
    LCD_ShowNum(118, 110, length_x10 / 10U, 4, 16);
    LCD_ShowString(150, 110, 8, 16, 16, (uint8_t *)".");
    LCD_ShowNum(158, 110, length_x10 % 10U, 1, 16);
    LCD_ShowString(170, 110, 8, 16, 16, (uint8_t *)"m");
  }
  else
  {
    LCD_ShowString(118, 110, 48, 16, 16, (uint8_t *)"--.-m");
  }

  if (double_result->attenuation_valid != 0U)
  {
    attenuation_x100 = (double_result->attenuation_db >= 0.0f) ?
                       (int32_t)(double_result->attenuation_db * 100.0f + 0.5f) :
                       (int32_t)(double_result->attenuation_db * 100.0f - 0.5f);
    if (attenuation_x100 < 0)
    {
      LCD_ShowString(118, 132, 8, 16, 16, (uint8_t *)"-");
      attenuation_abs_x100 = (uint32_t)(-attenuation_x100);
    }
    else
    {
      LCD_ShowString(118, 132, 8, 16, 16, (uint8_t *)"+");
      attenuation_abs_x100 = (uint32_t)attenuation_x100;
    }

    LCD_ShowNum(130, 132, attenuation_abs_x100 / 100U, 3, 16);
    LCD_ShowString(154, 132, 8, 16, 16, (uint8_t *)".");
    LCD_ShowxNum(162, 132, attenuation_abs_x100 % 100U, 2, 16, 0x80U);
    LCD_ShowString(182, 132, 16, 16, 16, (uint8_t *)"dB");
  }
  else
  {
    LCD_ShowString(118, 132, 72, 16, 16, (uint8_t *)"--.-- dB");
  }

  if (double_result->cable.valid != 0U)
  {
    LCD_ShowString(118, 154, 32, 16, 16,
                   (uint8_t *)(double_result->cable.shielded != 0U ? "SFTP" : "UTP"));

    if (double_result->cable.wiring == CABLE_WIRING_STRAIGHT)
    {
      LCD_ShowString(118, 176, 64, 16, 16, (uint8_t *)"STRAIGHT");
    }
    else if (double_result->cable.wiring == CABLE_WIRING_CROSS)
    {
      LCD_ShowString(118, 176, 40, 16, 16, (uint8_t *)"CROSS");
      LCD_ShowxNum(162, 176, double_result->cable.out1_input_mask, 2, 16, 0x80U);
      LCD_ShowString(178, 176, 8, 16, 16, (uint8_t *)"/");
      LCD_ShowxNum(190, 176, double_result->cable.out2_input_mask, 2, 16, 0x80U);
    }
    else
    {
      /*
       * 故障时显示输出回读和 OUT1/OUT2 读到的输入位图，便于排查：
       * IN1=1、IN2=2、IN3=4、IN6=8；直连应为 01/02，交叉应为 04/08。
       */
      LCD_ShowString(118, 176, 8, 16, 16, (uint8_t *)"O");
      LCD_ShowNum(126, 176, double_result->cable.output_mask, 1, 16);
      LCD_ShowString(138, 176, 8, 16, 16, (uint8_t *)":");
      LCD_ShowxNum(150, 176, double_result->cable.out1_input_mask, 2, 16, 0x80U);
      LCD_ShowString(166, 176, 8, 16, 16, (uint8_t *)"/");
      LCD_ShowxNum(178, 176, double_result->cable.out2_input_mask, 2, 16, 0x80U);
    }
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
