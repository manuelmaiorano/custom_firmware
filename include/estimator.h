#include "stabilizer_types.h"


void kalman_init();
void estimatorEnqueueTDOA(const tdoaMeasurement_t *measurement);

void estimatorKalman(state_t *state, const stabilizerStep_t stabilizerStep);