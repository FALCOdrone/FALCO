#ifndef IMU_H
#define IMU_H

#include <Wire.h>

#include "FastIMU.h"
#include "Madgwick.h"
#include "pinDef.h"
#include "types.h"
//kalman filter
#include "Kalman.h"

#define Nstate 6
#define Nobs 3

#define ULONG_MAX 0xFFFFFFFF

void initializeImu(int calibrate = 1);
void getQuaternion(quat_t *quat);
void getAcceleration(vec_t *accel);
vec_t computeLinSpeed(vec_t accel, vec_t prevSpeed);
void updateKALMAN(KALMAN<Nstate, Nobs> *K , vec_t *pos, vec_t *speed, vec_t *acc);

void printIMUData(vec_t data, const char *unit);
void printIMUData(quat_t quat);

#endif