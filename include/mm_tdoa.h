
#pragma once

#include "kalman_core.h"
#include "outlierFilterTdoa.h"

// Measurements of a UWB Tx/Rx
void kalmanCoreUpdateWithTdoa(kalmanCoreData_t* this, tdoaMeasurement_t *tdoa, const uint32_t nowMs, OutlierFilterTdoaState_t* outlierFilterState);
