#ifndef IMU_H
#define IMU_H

#include <Wire.h>

#include "FastIMU.h"
#include "Madgwick.h"
#include "pinDef.h"
#include "types.h"

#define ULONG_MAX 0xFFFFFFFF

void initializeImu(int calibrate = 1);
quat_t getQuaternion();
accel_t getAcceleration();
speed_t computeLinSpeed(accel_t accel = getAcceleration(), speed_t prevSpeed);

void printIMUData(accel_t accel, speed_t speed, quat_t quat);

#endif