#line 1 "C:\\Users\\gbeve\\Downloads\\FALCO\\controller.h"
#ifndef CTRL_H
#define CTRL_H

#include <math.h>

#include "types.h"

// VELOCITY CONTROLLER
#define KP_X 5
#define KP_Y 5
#define KP_Z 500
#define KI_X 0.01
#define KI_Y 0.01
#define KI_Z 0.4
#define KD_X 0.001
#define KD_Y 0.001
#define KD_Z 5

void stabilityPID(float thrust[4], accel_t newAcc, accel_t oldAcc, quat_t newQuat, quat_t oldQuat);

#endif