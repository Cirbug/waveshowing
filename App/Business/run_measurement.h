#ifndef RUN_MEASUREMENT_H
#define RUN_MEASUREMENT_H

#include <stdint.h>

#define RUN_MEASUREMENT_MAX_SAMPLES 200U

typedef struct
{
  uint8_t valid;
  uint8_t frequency_valid;
  uint32_t frequency_hz;
  int32_t difference;
} RunMeasurementResult;

typedef struct
{
  uint8_t running;
  uint8_t mode;
  uint32_t start_tick;
  uint32_t last_sample_tick;
  uint32_t frequency_samples[RUN_MEASUREMENT_MAX_SAMPLES];
  int32_t difference_samples[RUN_MEASUREMENT_MAX_SAMPLES];
  uint16_t frequency_count;
  uint16_t difference_count;
} RunMeasurement;

void RunMeasurement_Start(RunMeasurement *measurement, uint8_t mode, uint32_t now);
uint8_t RunMeasurement_Update(RunMeasurement *measurement,
                              uint32_t now,
                              uint32_t frequency_hz,
                              uint16_t adc1,
                              uint16_t adc2,
                              RunMeasurementResult *result);

#endif /* RUN_MEASUREMENT_H */
