#include "measurement_math.h"

float MeasurementMath_DifferenceToResistance(int32_t difference,
                                             float voltage_scale_mv,
                                             float current_ma)
{
  uint32_t absolute_difference;

  if (current_ma <= 0.0f)
  {
    return 0.0f;
  }

  absolute_difference = (difference < 0) ? (uint32_t)(-difference) : (uint32_t)difference;
  return ((float)absolute_difference * voltage_scale_mv) / current_ma;
}

uint16_t MeasurementMath_ReciprocalLengthX10(uint32_t frequency_hz,
                                             uint32_t scale_x1000,
                                             uint32_t offset_x1000,
                                             uint16_t max_length_x10,
                                             uint16_t invalid_value)
{
  uint32_t reciprocal_x1000;
  uint32_t length_x1000;

  if (frequency_hz == 0U)
  {
    return invalid_value;
  }

  reciprocal_x1000 = scale_x1000 / frequency_hz;
  if (reciprocal_x1000 <= offset_x1000)
  {
    return 0U;
  }

  length_x1000 = reciprocal_x1000 - offset_x1000;
  if (length_x1000 >= ((uint32_t)max_length_x10 * 100U))
  {
    return max_length_x10;
  }

  return (uint16_t)((length_x1000 + 50U) / 100U);
}
