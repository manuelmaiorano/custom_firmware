#include "stabilizer_types.h"


typedef enum {
  MeasurementTypeTDOA,
  MeasurementTypePosition,
  MeasurementTypePose,
  MeasurementTypeDistance,
  MeasurementTypeTOF,
  MeasurementTypeAbsoluteHeight,
  MeasurementTypeFlow,
  MeasurementTypeYawError,
  MeasurementTypeSweepAngle,
  MeasurementTypeGyroscope,
  MeasurementTypeAcceleration,
  MeasurementTypeBarometer,
} MeasurementType;

typedef struct
{
  MeasurementType type;
  union
  {
    tdoaMeasurement_t tdoa;
    positionMeasurement_t position;
    poseMeasurement_t pose;
    distanceMeasurement_t distance;
    tofMeasurement_t tof;
    heightMeasurement_t height;
    flowMeasurement_t flow;
    yawErrorMeasurement_t yawError;
    //sweepAngleMeasurement_t sweepAngle;
    gyroscopeMeasurement_t gyroscope;
    accelerationMeasurement_t acceleration;
    barometerMeasurement_t barometer;
  } data;
} measurement_t;

void kalman_init();
void estimatorEnqueueTDOA(const tdoaMeasurement_t *measurement);
void estimatorEnqueue(const measurement_t* measurement);

void estimatorKalman(state_t *state, const stabilizerStep_t stabilizerStep);