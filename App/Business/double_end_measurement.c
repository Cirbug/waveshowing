#include "double_end_measurement.h"
#include "measurement_math.h"
#include <stddef.h>
#include <string.h>

#define DOUBLE_END_VOLTAGE_SCALE_MV 2.0f
#define DOUBLE_END_MEASURE_CURRENT_MA 330.0f

void DoubleEndMeasurement_Calculate(int32_t difference,
                                    const CableTestResult *cable,
                                    const DoubleCalibrationData *calibration,
                                    DoubleEndMeasurementResult *result)
{
  uint32_t absolute_difference;

  if (result == NULL)
  {
    return;
  }

  memset(result, 0, sizeof(*result));
  result->valid = 1U;
  result->difference = difference;
  absolute_difference = (difference < 0) ? (uint32_t)(-difference) : (uint32_t)difference;
  result->voltage_mv = (float)absolute_difference * DOUBLE_END_VOLTAGE_SCALE_MV;
  result->resistance_ohm = MeasurementMath_DifferenceToResistance(difference,
                                                                  DOUBLE_END_VOLTAGE_SCALE_MV,
                                                                  DOUBLE_END_MEASURE_CURRENT_MA);

  if (cable != NULL)
  {
    result->cable = *cable;
  }

  result->length_valid = CalibrationModel_GetDoubleLength(calibration,
                                                           result->resistance_ohm,
                                                           &result->length_m);
}
