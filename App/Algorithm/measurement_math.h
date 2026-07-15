#ifndef MEASUREMENT_MATH_H
#define MEASUREMENT_MATH_H

#include <stdint.h>

float MeasurementMath_DifferenceToResistance(int32_t difference,
                                             float voltage_scale_mv,
                                             float current_ma);

uint16_t MeasurementMath_ReciprocalLengthX10(uint32_t frequency_hz,
                                             uint32_t scale_x1000,
                                             uint32_t offset_x1000,
                                             uint16_t max_length_x10,
                                             uint16_t invalid_value);

#endif /* MEASUREMENT_MATH_H */
