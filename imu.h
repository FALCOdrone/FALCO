#ifndef IMU_H
#define IMU_H

#include <Wire.h>

#include "FastIMU.h"
#include "Madgwick.h"
#include "pinDef.h"
#include "types.h"

#define ULONG_MAX 0xFFFFFFFF

void initializeImu(int calibrate = 1);
void getQuaternion(quat_t *quat);
void getAcceleration(accel_t *accel);
speed_t computeLinSpeed(accel_t accel, speed_t prevSpeed);

void printIMUData(accel_t accel);
void printIMUData(speed_t speed);
void printIMUData(quat_t quat);
void printIMUData(accel_t accel, speed_t speed);
void printIMUData(accel_t accel, quat_t quat);
void printIMUData(speed_t speed, quat_t quat);
void printIMUData(accel_t accel, speed_t speed, quat_t quat);


#endif