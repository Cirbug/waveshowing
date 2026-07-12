#include "touch.h"
#include "lcd.h"

#define TOUCH_PEN_PORT GPIOB
#define TOUCH_PEN_PIN GPIO_PIN_1
#define TOUCH_CS_PORT GPIOC
#define TOUCH_CS_PIN GPIO_PIN_13
#define TOUCH_MISO_PORT GPIOB
#define TOUCH_MISO_PIN GPIO_PIN_2
#define TOUCH_MOSI_PORT GPIOF
#define TOUCH_MOSI_PIN GPIO_PIN_11
#define TOUCH_CLK_PORT GPIOB
#define TOUCH_CLK_PIN GPIO_PIN_0

#define TOUCH_READ_TIMES 5U
#define TOUCH_LOST_VAL 1U
#define TOUCH_ERR_RANGE 80U

/* 这里是默认粗略校准值。若坐标偏差大，后续按你的屏实测 raw 值再精调。 */
#define TOUCH_RAW_X_MIN 200U
#define TOUCH_RAW_X_MAX 3900U
#define TOUCH_RAW_Y_MIN 200U
#define TOUCH_RAW_Y_MAX 3900U
#define TOUCH_INVERT_X 1U
#define TOUCH_INVERT_Y 0U

#define TOUCH_MOSI_WRITE(x) HAL_GPIO_WritePin(TOUCH_MOSI_PORT, TOUCH_MOSI_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define TOUCH_CLK_WRITE(x) HAL_GPIO_WritePin(TOUCH_CLK_PORT, TOUCH_CLK_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define TOUCH_CS_WRITE(x) HAL_GPIO_WritePin(TOUCH_CS_PORT, TOUCH_CS_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define TOUCH_PEN_READ() (HAL_GPIO_ReadPin(TOUCH_PEN_PORT, TOUCH_PEN_PIN) == GPIO_PIN_RESET)
#define TOUCH_MISO_READ() (HAL_GPIO_ReadPin(TOUCH_MISO_PORT, TOUCH_MISO_PIN) == GPIO_PIN_SET)

static void Touch_DelayUs(uint32_t us);
static void Touch_WriteByte(uint8_t data);
static uint16_t Touch_ReadAd(uint8_t cmd);
static uint16_t Touch_ReadFiltered(uint8_t cmd);
static uint8_t Touch_ReadRaw(uint16_t *raw_x, uint16_t *raw_y);
static uint16_t Touch_MapToScreen(uint16_t raw, uint16_t raw_min, uint16_t raw_max, uint16_t screen_max, uint8_t invert);

void Touch_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  gpio.Pin = TOUCH_PEN_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TOUCH_PEN_PORT, &gpio);

  gpio.Pin = TOUCH_MISO_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TOUCH_MISO_PORT, &gpio);

  gpio.Pin = TOUCH_CS_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TOUCH_CS_PORT, &gpio);

  gpio.Pin = TOUCH_MOSI_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TOUCH_MOSI_PORT, &gpio);

  gpio.Pin = TOUCH_CLK_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TOUCH_CLK_PORT, &gpio);

  TOUCH_CS_WRITE(1);
  TOUCH_CLK_WRITE(1);
  TOUCH_MOSI_WRITE(1);
}

uint8_t Touch_Scan(TouchState *state)
{
  uint16_t raw_x;
  uint16_t raw_y;

  if (state == NULL)
  {
    return 0U;
  }

  if (!TOUCH_PEN_READ())
  {
    state->pressed = 0U;
    return 0U;
  }

  if (Touch_ReadRaw(&raw_x, &raw_y) == 0U)
  {
    return state->pressed;
  }

  state->raw_x = raw_x;
  state->raw_y = raw_y;
  state->x = Touch_MapToScreen(raw_x, TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX, lcddev.width - 1U, TOUCH_INVERT_X);
  state->y = Touch_MapToScreen(raw_y, TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX, lcddev.height - 1U, TOUCH_INVERT_Y);
  state->pressed = 1U;

  return 1U;
}

static void Touch_DelayUs(uint32_t us)
{
  uint32_t cycles_per_us = SystemCoreClock / 1000000U;
  uint32_t start;
  uint32_t ticks = us * cycles_per_us;

  if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }

  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
  {
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < ticks)
  {
  }
}

static void Touch_WriteByte(uint8_t data)
{
  for (uint8_t i = 0U; i < 8U; ++i)
  {
    TOUCH_MOSI_WRITE((data & 0x80U) != 0U);
    data <<= 1U;
    TOUCH_CLK_WRITE(0);
    Touch_DelayUs(1U);
    TOUCH_CLK_WRITE(1);
  }
}

static uint16_t Touch_ReadAd(uint8_t cmd)
{
  uint16_t value = 0U;

  TOUCH_CLK_WRITE(0);
  TOUCH_MOSI_WRITE(0);
  TOUCH_CS_WRITE(0);
  Touch_WriteByte(cmd);
  Touch_DelayUs(6U);

  TOUCH_CLK_WRITE(0);
  Touch_DelayUs(1U);
  TOUCH_CLK_WRITE(1);
  Touch_DelayUs(1U);
  TOUCH_CLK_WRITE(0);

  for (uint8_t i = 0U; i < 16U; ++i)
  {
    value <<= 1U;
    TOUCH_CLK_WRITE(0);
    Touch_DelayUs(1U);
    TOUCH_CLK_WRITE(1);
    if (TOUCH_MISO_READ())
    {
      value++;
    }
  }

  TOUCH_CS_WRITE(1);
  return value >> 4U;
}

static uint16_t Touch_ReadFiltered(uint8_t cmd)
{
  uint16_t buf[TOUCH_READ_TIMES];
  uint32_t sum = 0U;

  for (uint8_t i = 0U; i < TOUCH_READ_TIMES; ++i)
  {
    buf[i] = Touch_ReadAd(cmd);
  }

  for (uint8_t i = 0U; i < (TOUCH_READ_TIMES - 1U); ++i)
  {
    for (uint8_t j = (uint8_t)(i + 1U); j < TOUCH_READ_TIMES; ++j)
    {
      if (buf[i] > buf[j])
      {
        uint16_t temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }

  for (uint8_t i = TOUCH_LOST_VAL; i < (TOUCH_READ_TIMES - TOUCH_LOST_VAL); ++i)
  {
    sum += buf[i];
  }

  return (uint16_t)(sum / (TOUCH_READ_TIMES - 2U * TOUCH_LOST_VAL));
}

static uint8_t Touch_ReadRaw(uint16_t *raw_x, uint16_t *raw_y)
{
  uint16_t x1;
  uint16_t y1;
  uint16_t x2;
  uint16_t y2;

  if (lcddev.dir != 0U)
  {
    x1 = Touch_ReadFiltered(0x90U);
    y1 = Touch_ReadFiltered(0xD0U);
    x2 = Touch_ReadFiltered(0x90U);
    y2 = Touch_ReadFiltered(0xD0U);
  }
  else
  {
    x1 = Touch_ReadFiltered(0xD0U);
    y1 = Touch_ReadFiltered(0x90U);
    x2 = Touch_ReadFiltered(0xD0U);
    y2 = Touch_ReadFiltered(0x90U);
  }

  if (((x1 > x2) ? (x1 - x2) : (x2 - x1)) > TOUCH_ERR_RANGE)
  {
    return 0U;
  }

  if (((y1 > y2) ? (y1 - y2) : (y2 - y1)) > TOUCH_ERR_RANGE)
  {
    return 0U;
  }

  *raw_x = (uint16_t)((x1 + x2) / 2U);
  *raw_y = (uint16_t)((y1 + y2) / 2U);
  return 1U;
}

static uint16_t Touch_MapToScreen(uint16_t raw, uint16_t raw_min, uint16_t raw_max, uint16_t screen_max, uint8_t invert)
{
  uint32_t value;

  if (raw <= raw_min)
  {
    value = 0U;
  }
  else if (raw >= raw_max)
  {
    value = screen_max;
  }
  else
  {
    value = ((uint32_t)(raw - raw_min) * screen_max) / (raw_max - raw_min);
  }

  if (invert != 0U)
  {
    value = screen_max - value;
  }

  return (uint16_t)value;
}
