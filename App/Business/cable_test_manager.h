#ifndef CABLE_TEST_MANAGER_H
#define CABLE_TEST_MANAGER_H

#include <stdint.h>
#include "cable_gpio.h"

typedef enum
{
  CABLE_WIRING_UNKNOWN = 0,
  CABLE_WIRING_STRAIGHT,
  CABLE_WIRING_CROSS,
  CABLE_WIRING_FAULT
} CableWiringType;

typedef struct
{
  uint8_t valid;
  uint8_t shielded;
  uint8_t output_mask;
  uint8_t out1_input_mask;
  uint8_t out2_input_mask;
  CableWiringType wiring;
} CableTestResult;

typedef enum
{
  CABLE_TEST_IDLE = 0,
  CABLE_TEST_OUTPUT_SETTLE,
  CABLE_TEST_INPUT_SAMPLE,
  CABLE_TEST_SHIELD_SAMPLE
} CableTestState;

typedef struct
{
  uint8_t running;
  CableTestState state;
  uint8_t output_index;
  uint8_t sample_index;
  uint8_t output_high_mask;
  uint8_t shield_high_count;
  uint8_t input_masks[CABLE_GPIO_OUTPUT_COUNT];
  uint8_t high_counts[CABLE_GPIO_INPUT_COUNT];
  uint32_t next_tick;
} CableTestManager;

void CableTestManager_Start(CableTestManager *manager, uint32_t now);
uint8_t CableTestManager_Process(CableTestManager *manager,
                                 uint32_t now,
                                 CableTestResult *result);

#endif /* CABLE_TEST_MANAGER_H */
