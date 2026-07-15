#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include <stdint.h>

uint32_t MedianFilter_U32(uint32_t *values, uint16_t count);
int32_t MedianFilter_S32(int32_t *values, uint16_t count);

#endif /* MEDIAN_FILTER_H */
