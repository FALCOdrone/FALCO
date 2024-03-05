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
#define Ncom 3

#define ULONG_MAX 0xFFFFFFFF

void initializeImu(int calibrate = 1);
void getQuaternion(quat_t *quat);
void getAcceleration(accel_t *accel);
speed_t computeLinSpeed(accel_t accel, speed_t prevSpeed);
state_t updateKALMAN(KALMAN<Nstate, Nobs, Ncom> *K , accel_t acc);

void printIMUData(accel_t accel);
void printIMUData(speed_t speed);
void printIMUData(quat_t quat);
void printIMUData(state_t state);
void printIMUData(accel_t accel, speed_t speed);
void printIMUData(accel_t accel, quat_t quat);
void printIMUData(speed_t speed, quat_t quat);
void printIMUData(accel_t accel, speed_t speed, quat_t quat);



#endif