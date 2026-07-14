#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include "main.h"

#define SPI_FLASH_NM25Q128_ID 0x5217U

uint8_t SpiFlash_Init(void);
uint16_t SpiFlash_ReadId(void);
uint8_t SpiFlash_Read(uint32_t address, uint8_t *data, uint16_t length);
uint8_t SpiFlash_WriteSector(uint32_t address, const uint8_t *data, uint16_t length);

#endif
