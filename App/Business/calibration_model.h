#ifndef CALIBRATION_MODEL_H
#define CALIBRATION_MODEL_H

#include <stdint.h>

#define CALIBRATION_MAX_POINTS 5U

typedef struct
{
  uint32_t point_count;
  float length_m[CALIBRATION_MAX_POINTS];
  float resistance_ohm[CALIBRATION_MAX_POINTS];
  float resistance_per_meter;
  float zero_resistance;
} DoubleCalibrationData;

typedef struct
{
  uint32_t point_count;
  float length_m[CALIBRATION_MAX_POINTS];
  uint32_t frequency_hz[CALIBRATION_MAX_POINTS];
  float inverse_period_slope;
  float offset_m;
} SingleCalibrationData;

uint8_t CalibrationModel_FitDouble(DoubleCalibrationData *calibration);
uint8_t CalibrationModel_FitSingle(SingleCalibrationData *calibration);
uint8_t CalibrationModel_GetDoubleLength(const DoubleCalibrationData *calibration,
                                         float resistance_ohm,
                                         float *length_m);
uint8_t CalibrationModel_GetSingleLength(const SingleCalibrationData *calibration,
                                         uint32_t frequency_hz,
                                         float *length_m);

#endif /* CALIBRATION_MODEL_H */
