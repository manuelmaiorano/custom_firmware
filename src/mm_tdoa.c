
#include "mm_tdoa.h"

#if CONFIG_ESTIMATOR_KALMAN_TDOA_OUTLIERFILTER_FALLBACK
#include "outlierFilterTdoaSteps.h"
#endif

void kalmanCoreUpdateWithTdoa(kalmanCoreData_t* this, tdoaMeasurement_t *tdoa, const uint32_t nowMs, OutlierFilterTdoaState_t* outlierFilterState)
{
  /**
   * Measurement equation:
   * dR = dT + d1 - d0
   */

  float measurement = tdoa->distanceDiff;

  // predict based on current state
  float x = this->S[KC_STATE_X];
  float y = this->S[KC_STATE_Y];
  float z = this->S[KC_STATE_Z];

  float x1 = tdoa->anchorPositions[1].x, y1 = tdoa->anchorPositions[1].y, z1 = tdoa->anchorPositions[1].z;
  float x0 = tdoa->anchorPositions[0].x, y0 = tdoa->anchorPositions[0].y, z0 = tdoa->anchorPositions[0].z;

  float dx1 = x - x1;
  float dy1 = y - y1;
  float dz1 = z - z1;

  float dy0 = y - y0;
  float dx0 = x - x0;
  float dz0 = z - z0;

  float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
  float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));

  float predicted = d1 - d0;
  float error = measurement - predicted;

  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  if ((d0 != 0.0f) && (d1 != 0.0f)) {
    h[KC_STATE_X] = (dx1 / d1 - dx0 / d0);
    h[KC_STATE_Y] = (dy1 / d1 - dy0 / d0);
    h[KC_STATE_Z] = (dz1 / d1 - dz0 / d0);

  #if CONFIG_ESTIMATOR_KALMAN_TDOA_OUTLIERFILTER_FALLBACK
    vector_t jacobian = {
      .x = h[KC_STATE_X],
      .y = h[KC_STATE_Y],
      .z = h[KC_STATE_Z],
    };

    point_t estimatedPosition = {
      .x = this->S[KC_STATE_X],
      .y = this->S[KC_STATE_Y],
      .z = this->S[KC_STATE_Z],
    };

    bool sampleIsGood = outlierFilterTdoaValidateSteps(tdoa, error, &jacobian, &estimatedPosition);
    #else
    bool sampleIsGood = outlierFilterTdoaValidateIntegrator(outlierFilterState, tdoa, error, nowMs);
    #endif

    if (sampleIsGood) {
      kalmanCoreScalarUpdate(this, &H, error, tdoa->stdDev);
    }
  }
}
