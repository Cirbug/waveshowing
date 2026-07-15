#include "linear_fit.h"
#include <stddef.h>

uint8_t LinearFit_Calculate(const float *x,
                            const float *y,
                            uint32_t count,
                            float *slope,
                            float *offset)
{
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_xx = 0.0f;
  float sum_xy = 0.0f;
  float denominator;
  float point_count;

  if ((x == NULL) || (y == NULL) || (slope == NULL) || (offset == NULL) || (count < 2U))
  {
    return 0U;
  }

  for (uint32_t i = 0U; i < count; ++i)
  {
    sum_x += x[i];
    sum_y += y[i];
    sum_xx += x[i] * x[i];
    sum_xy += x[i] * y[i];
  }

  point_count = (float)count;
  denominator = point_count * sum_xx - sum_x * sum_x;
  if ((denominator > -0.0001f) && (denominator < 0.0001f))
  {
    return 0U;
  }

  *slope = (point_count * sum_xy - sum_x * sum_y) / denominator;
  *offset = (sum_y - *slope * sum_x) / point_count;
  return 1U;
}
