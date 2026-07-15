#include "run_measurement.h"
#include "median_filter.h"
#include <stddef.h>

#define RUN_DURATION_MS 4000U
#define RUN_SAMPLE_PERIOD_MS 20U

void RunMeasurement_Start(RunMeasurement *measurement, uint8_t mode, uint32_t now)
{
  if (measurement == NULL)
  {
    return;
  }

  measurement->mode = mode;
  measurement->start_tick = now;
  measurement->last_sample_tick = now - RUN_SAMPLE_PERIOD_MS;
  measurement->frequency_count = 0U;
  measurement->difference_count = 0U;
  measurement->running = 1U;
}

uint8_t RunMeasurement_Update(RunMeasurement *measurement,
                              uint32_t now,
                              uint32_t frequency_hz,
                              uint16_t adc1,
                              uint16_t adc2,
                              RunMeasurementResult *result)
{
  if ((measurement == NULL) || (result == NULL) || (measurement->running == 0U))
  {
    return 0U;
  }

  if ((now - measurement->last_sample_tick) >= RUN_SAMPLE_PERIOD_MS)
  {
    measurement->last_sample_tick = now;

    if (measurement->difference_count < RUN_MEASUREMENT_MAX_SAMPLES)
    {
      measurement->difference_samples[measurement->difference_count++] = (int32_t)adc1 - (int32_t)adc2;
    }

    if ((frequency_hz != 0U) && (measurement->frequency_count < RUN_MEASUREMENT_MAX_SAMPLES))
    {
      measurement->frequency_samples[measurement->frequency_count++] = frequency_hz;
    }
  }

  if ((now - measurement->start_tick) < RUN_DURATION_MS)
  {
    return 0U;
  }

  result->frequency_valid = (measurement->frequency_count != 0U) ? 1U : 0U;
  result->frequency_hz = (measurement->frequency_count != 0U) ?
                         MedianFilter_U32(measurement->frequency_samples, measurement->frequency_count) : 0U;
  result->difference = (measurement->difference_count != 0U) ?
                       MedianFilter_S32(measurement->difference_samples, measurement->difference_count) : 0;
  result->valid = (measurement->difference_count != 0U) ? 1U : 0U;
  measurement->running = 0U;
  return 1U;
}
