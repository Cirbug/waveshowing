#ifndef LINEAR_FIT_H
#define LINEAR_FIT_H

#include <stdint.h>

uint8_t LinearFit_Calculate(const float *x,
                            const float *y,
                            uint32_t count,
                            float *slope,
                            float *offset);

#endif /* LINEAR_FIT_H */
