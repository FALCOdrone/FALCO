#include "controller.h"
#include "imu.h"
#include "motor.h"
#include "pinDef.h"

#define DEBUG 1

speed_t speed;
speed_t prevSpeed ;
accel_t accel;
quat_t quat;

accel_t desiredAccel;
quat_t desiredQuat;

float thrust[4];

void updateIMU() {
    getAcceleration(&accel);
    getQuaternion(&quat);
    speed = computeLinSpeed(accel, prevSpeed);
    prevSpeed = speed;
}

void computeAccel(accel_t *desiredAccel) {
    desiredAccel->x = 0.0;
    desiredAccel->y = 0.0;
    desiredAccel->z = 0.0;
}

void computeQuat(quat_t *desiredQuat) {
    desiredQuat->w = 1.0;
    desiredQuat->x = 0.0;
    desiredQuat->y = 0.0;
    desiredQuat->z = 0.0;
}

void setup() {
    Serial.begin(115200);
    initializeImu();
    updateIMU();
    accel.t = micros();
}

void loop() {
    updateIMU();
    if (DEBUG) printIMUData(accel, speed, quat);

    //computeAccel(&desiredAccel);
    //computeQuat(&desiredQuat);

    //stabilityPID(thrust, desiredAccel, accel, desiredQuat, quat);
    //driveMotors(thrust);

    delay(100);
}
