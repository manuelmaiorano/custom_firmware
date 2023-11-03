
#pragma once

#include "stabilizer_types.h"

typedef struct {
    float integrator;
    uint32_t latestUpdateMs;
    bool isFilterOpen;
} OutlierFilterTdoaState_t;

void outlierFilterTdoaReset(OutlierFilterTdoaState_t* this);
bool outlierFilterTdoaValidateIntegrator(OutlierFilterTdoaState_t* this, const tdoaMeasurement_t* tdoa, const float error, const uint32_t nowMs);
