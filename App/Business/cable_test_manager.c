#include "cable_test_manager.h"
#include "cable_gpio.h"
#include <stddef.h>
#include <string.h>

#define CABLE_SAMPLE_COUNT 5U
#define CABLE_SETTLE_MS 300U
#define CABLE_SAMPLE_INTERVAL_MS 240U

static uint8_t CableTestManager_TimeReached(uint32_t now, uint32_t target)
{
  return ((int32_t)(now - target) >= 0) ? 1U : 0U;
}

static void CableTestManager_StartOutput(CableTestManager *manager, uint32_t now)
{
  memset(manager->high_counts, 0, sizeof(manager->high_counts));
  manager->sample_index = 0U;
  CableGpio_SetOutputsHighImpedance();
  CableGpio_EnableOutput(manager->output_index);
  manager->next_tick = now + CABLE_SETTLE_MS;
  manager->state = CABLE_TEST_OUTPUT_SETTLE;
}

void CableTestManager_Start(CableTestManager *manager, uint32_t now)
{
  if (manager == NULL)
  {
    return;
  }

  memset(manager, 0, sizeof(*manager));
  manager->running = 1U;
  manager->output_index = 0U;
  CableTestManager_StartOutput(manager, now);
}

uint8_t CableTestManager_Process(CableTestManager *manager,
                                 uint32_t now,
                                 CableTestResult *result)
{
  uint8_t input_mask;

  if ((manager == NULL) || (result == NULL) || (manager->running == 0U))
  {
    return 0U;
  }

  if (CableTestManager_TimeReached(now, manager->next_tick) == 0U)
  {
    return 0U;
  }

  if (manager->state == CABLE_TEST_OUTPUT_SETTLE)
  {
    if (CableGpio_IsOutputHigh(manager->output_index) != 0U)
    {
      manager->output_high_mask |= (uint8_t)(1U << manager->output_index);
    }
    manager->state = CABLE_TEST_INPUT_SAMPLE;
    manager->next_tick = now + CABLE_SAMPLE_INTERVAL_MS;
    return 0U;
  }

  if (manager->state == CABLE_TEST_INPUT_SAMPLE)
  {
    input_mask = CableGpio_ReadInputMask();
    for (uint8_t input = 0U; input < CABLE_GPIO_INPUT_COUNT; ++input)
    {
      if ((input_mask & (uint8_t)(1U << input)) != 0U)
      {
        manager->high_counts[input]++;
      }
    }

    manager->sample_index++;
    if (manager->sample_index < CABLE_SAMPLE_COUNT)
    {
      manager->next_tick = now + CABLE_SAMPLE_INTERVAL_MS;
      return 0U;
    }

    for (uint8_t input = 0U; input < CABLE_GPIO_INPUT_COUNT; ++input)
    {
      if (manager->high_counts[input] > (CABLE_SAMPLE_COUNT / 2U))
      {
        manager->input_masks[manager->output_index] |= (uint8_t)(1U << input);
      }
    }

    CableGpio_SetOutputsHighImpedance();
    manager->output_index++;
    if (manager->output_index < CABLE_GPIO_OUTPUT_COUNT)
    {
      CableTestManager_StartOutput(manager, now + 1U);
      return 0U;
    }

    manager->sample_index = 0U;
    manager->shield_high_count = 0U;
    manager->state = CABLE_TEST_SHIELD_SAMPLE;
    manager->next_tick = now + 1U;
    return 0U;
  }

  if (manager->state != CABLE_TEST_SHIELD_SAMPLE)
  {
    return 0U;
  }

  if (CableGpio_ReadShield() != 0U)
  {
    manager->shield_high_count++;
  }
  manager->sample_index++;
  if (manager->sample_index < CABLE_SAMPLE_COUNT)
  {
    manager->next_tick = now + 1U;
    return 0U;
  }

  CableGpio_RestoreOutputsLow();

  result->valid = 1U;
  result->shielded = (manager->shield_high_count > (CABLE_SAMPLE_COUNT / 2U)) ? 1U : 0U;
  result->output_mask = manager->output_high_mask;
  result->out1_input_mask = manager->input_masks[0];
  result->out2_input_mask = manager->input_masks[1];

  if (((manager->input_masks[0] & 0x01U) != 0U) &&
      ((manager->input_masks[1] & 0x02U) != 0U))
  {
    result->wiring = CABLE_WIRING_STRAIGHT;
  }
  else
  {
    result->wiring = CABLE_WIRING_CROSS;
  }

  manager->running = 0U;
  manager->state = CABLE_TEST_IDLE;
  return 1U;
}
