#include "median_filter.h"

uint32_t MedianFilter_U32(uint32_t *values, uint16_t count)
{
  for (uint16_t i = 1U; i < count; ++i)
  {
    uint32_t key = values[i];
    uint16_t j = i;

    while ((j > 0U) && (values[j - 1U] > key))
    {
      values[j] = values[j - 1U];
      --j;
    }
    values[j] = key;
  }

  if ((count & 1U) != 0U)
  {
    return values[count / 2U];
  }

  return (uint32_t)(((uint64_t)values[(count / 2U) - 1U] + values[count / 2U]) / 2U);
}

int32_t MedianFilter_S32(int32_t *values, uint16_t count)
{
  for (uint16_t i = 1U; i < count; ++i)
  {
    int32_t key = values[i];
    uint16_t j = i;

    while ((j > 0U) && (values[j - 1U] > key))
    {
      values[j] = values[j - 1U];
      --j;
    }
    values[j] = key;
  }

  if ((count & 1U) != 0U)
  {
    return values[count / 2U];
  }

  return (int32_t)(((int64_t)values[(count / 2U) - 1U] + values[count / 2U]) / 2);
}
