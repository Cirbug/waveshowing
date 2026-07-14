#include "spi_flash.h"

#define SPI_FLASH_CS_PORT GPIOB
#define SPI_FLASH_CS_PIN GPIO_PIN_14
#define SPI_FLASH_SCK_PIN GPIO_PIN_3
#define SPI_FLASH_MISO_PIN GPIO_PIN_4
#define SPI_FLASH_MOSI_PIN GPIO_PIN_5

#define SPI_FLASH_CMD_WRITE_ENABLE 0x06U
#define SPI_FLASH_CMD_READ_STATUS 0x05U
#define SPI_FLASH_CMD_READ_DATA 0x03U
#define SPI_FLASH_CMD_PAGE_PROGRAM 0x02U
#define SPI_FLASH_CMD_SECTOR_ERASE 0x20U
#define SPI_FLASH_CMD_READ_ID 0x90U

#define SPI_FLASH_PAGE_SIZE 256U
#define SPI_FLASH_SECTOR_SIZE 4096U
#define SPI_FLASH_TIMEOUT_MS 1000U

static SPI_HandleTypeDef spi_flash_handle;

static uint8_t SpiFlash_Transfer(uint8_t value);
static void SpiFlash_Select(uint8_t selected);
static void SpiFlash_SendAddress(uint32_t address);
static uint8_t SpiFlash_WriteEnable(void);
static uint8_t SpiFlash_WaitReady(void);
static uint8_t SpiFlash_EraseSector(uint32_t address);
static uint8_t SpiFlash_WritePage(uint32_t address, const uint8_t *data, uint16_t length);

uint8_t SpiFlash_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

  gpio.Pin = SPI_FLASH_SCK_PIN | SPI_FLASH_MISO_PIN | SPI_FLASH_MOSI_PIN;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = SPI_FLASH_CS_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI_FLASH_CS_PORT, &gpio);
  SpiFlash_Select(0U);

  spi_flash_handle.Instance = SPI1;
  spi_flash_handle.Init.Mode = SPI_MODE_MASTER;
  spi_flash_handle.Init.Direction = SPI_DIRECTION_2LINES;
  spi_flash_handle.Init.DataSize = SPI_DATASIZE_8BIT;
  spi_flash_handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
  spi_flash_handle.Init.CLKPhase = SPI_PHASE_2EDGE;
  spi_flash_handle.Init.NSS = SPI_NSS_SOFT;
  spi_flash_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  spi_flash_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi_flash_handle.Init.TIMode = SPI_TIMODE_DISABLE;
  spi_flash_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi_flash_handle.Init.CRCPolynomial = 7U;

  if (HAL_SPI_Init(&spi_flash_handle) != HAL_OK)
  {
    return 0U;
  }

  /* NM25Q128 支持 SPI Mode 0/3。若 Mode 3 无法读到ID，自动尝试 Mode 0。 */
  {
    uint16_t id = SpiFlash_ReadId();
    if ((id == 0U) || (id == 0xFFFFU))
    {
      spi_flash_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
      spi_flash_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
      if (HAL_SPI_Init(&spi_flash_handle) != HAL_OK)
      {
        return 0U;
      }
    }
  }

  return 1U;
}

uint16_t SpiFlash_ReadId(void)
{
  uint16_t id;

  SpiFlash_Select(1U);
  (void)SpiFlash_Transfer(SPI_FLASH_CMD_READ_ID);
  (void)SpiFlash_Transfer(0U);
  (void)SpiFlash_Transfer(0U);
  (void)SpiFlash_Transfer(0U);
  id = (uint16_t)SpiFlash_Transfer(0xFFU) << 8;
  id |= SpiFlash_Transfer(0xFFU);
  SpiFlash_Select(0U);

  return id;
}

uint8_t SpiFlash_Read(uint32_t address, uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U))
  {
    return 0U;
  }

  SpiFlash_Select(1U);
  (void)SpiFlash_Transfer(SPI_FLASH_CMD_READ_DATA);
  SpiFlash_SendAddress(address);

  for (uint16_t i = 0U; i < length; ++i)
  {
    data[i] = SpiFlash_Transfer(0xFFU);
  }

  SpiFlash_Select(0U);
  return 1U;
}

uint8_t SpiFlash_WriteSector(uint32_t address, const uint8_t *data, uint16_t length)
{
  uint16_t written = 0U;

  if ((data == NULL) || (length == 0U) || (length > SPI_FLASH_SECTOR_SIZE))
  {
    return 0U;
  }

  address &= ~(SPI_FLASH_SECTOR_SIZE - 1U);
  if (SpiFlash_EraseSector(address) == 0U)
  {
    return 0U;
  }

  while (written < length)
  {
    uint16_t chunk = (uint16_t)(length - written);
    if (chunk > SPI_FLASH_PAGE_SIZE)
    {
      chunk = SPI_FLASH_PAGE_SIZE;
    }

    if (SpiFlash_WritePage(address + written, &data[written], chunk) == 0U)
    {
      return 0U;
    }

    written = (uint16_t)(written + chunk);
  }

  return 1U;
}

static uint8_t SpiFlash_Transfer(uint8_t value)
{
  uint8_t received = 0U;

  if (HAL_SPI_TransmitReceive(&spi_flash_handle, &value, &received, 1U, 100U) != HAL_OK)
  {
    return 0U;
  }

  return received;
}

static void SpiFlash_Select(uint8_t selected)
{
  HAL_GPIO_WritePin(SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN,
                    selected != 0U ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void SpiFlash_SendAddress(uint32_t address)
{
  (void)SpiFlash_Transfer((uint8_t)(address >> 16));
  (void)SpiFlash_Transfer((uint8_t)(address >> 8));
  (void)SpiFlash_Transfer((uint8_t)address);
}

static uint8_t SpiFlash_WriteEnable(void)
{
  SpiFlash_Select(1U);
  (void)SpiFlash_Transfer(SPI_FLASH_CMD_WRITE_ENABLE);
  SpiFlash_Select(0U);
  return 1U;
}

static uint8_t SpiFlash_WaitReady(void)
{
  uint32_t start = HAL_GetTick();
  uint8_t status;

  do
  {
    SpiFlash_Select(1U);
    (void)SpiFlash_Transfer(SPI_FLASH_CMD_READ_STATUS);
    status = SpiFlash_Transfer(0xFFU);
    SpiFlash_Select(0U);

    if ((status & 0x01U) == 0U)
    {
      return 1U;
    }
  } while ((HAL_GetTick() - start) < SPI_FLASH_TIMEOUT_MS);

  return 0U;
}

static uint8_t SpiFlash_EraseSector(uint32_t address)
{
  if (SpiFlash_WriteEnable() == 0U)
  {
    return 0U;
  }

  SpiFlash_Select(1U);
  (void)SpiFlash_Transfer(SPI_FLASH_CMD_SECTOR_ERASE);
  SpiFlash_SendAddress(address);
  SpiFlash_Select(0U);

  return SpiFlash_WaitReady();
}

static uint8_t SpiFlash_WritePage(uint32_t address, const uint8_t *data, uint16_t length)
{
  if (SpiFlash_WriteEnable() == 0U)
  {
    return 0U;
  }

  SpiFlash_Select(1U);
  (void)SpiFlash_Transfer(SPI_FLASH_CMD_PAGE_PROGRAM);
  SpiFlash_SendAddress(address);

  for (uint16_t i = 0U; i < length; ++i)
  {
    (void)SpiFlash_Transfer(data[i]);
  }

  SpiFlash_Select(0U);
  return SpiFlash_WaitReady();
}
