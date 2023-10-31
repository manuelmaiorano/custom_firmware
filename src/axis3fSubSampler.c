
#include <string.h>
#include "axis3fSubSampler.h"

void axis3fSubSamplerInit(Axis3fSubSampler_t* this, const float conversionFactor) {
  memset(this, 0, sizeof(Axis3fSubSampler_t));
  this->conversionFactor = conversionFactor;
}

void axis3fSubSamplerAccumulate(Axis3fSubSampler_t* this, const Axis3f* sample) {
  this->sum.x += sample->x;
  this->sum.y += sample->y;
  this->sum.z += sample->z;

  this->count++;
}

Axis3f* axis3fSubSamplerFinalize(Axis3fSubSampler_t* this) {
  if (this->count > 0) {
    this->subSample.x = this->sum.x * this->conversionFactor / this->count;
    this->subSample.y = this->sum.y * this->conversionFactor / this->count;
    this->subSample.z = this->sum.z * this->conversionFactor / this->count;

    // Reset
    this->count = 0;
    this->sum = (Axis3f){.axis={0}};
  }

  return &this->subSample;
}
