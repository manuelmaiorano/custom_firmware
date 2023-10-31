#pragma once

#include <stdint.h>
#include "imu_types.h"

typedef struct {
  Axis3f sum;
  uint32_t count;
  float conversionFactor;

  Axis3f subSample;
} Axis3fSubSampler_t;

/**
 * @brief Initialize sub sampler
 *
 * @param this  Pointer to sub sampler
 * @param conversionFactor  Conversion factor used for unit conversion.
 */
void axis3fSubSamplerInit(Axis3fSubSampler_t* this, const float conversionFactor);

/**
 * @brief Accumulate a sample
 *
 * @param this  Pointer to sub sampler
 * @param sample  The sample to accumulate
 */
void axis3fSubSamplerAccumulate(Axis3fSubSampler_t* this, const Axis3f* sample);

/**
 * @brief Compute the sub sample, uses simple averaging of samples. The sub sample is multiplied with the conversion
 * factor and the result is stored in the subSample member of the Axis3fSubSampler_t.
 *
 * @param this  Pointer to sub sampler
 * @return Axis3f*  Pointer to the resulting sub sample
 */
Axis3f* axis3fSubSamplerFinalize(Axis3fSubSampler_t* this);
