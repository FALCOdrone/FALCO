#include <Arduino.h>
#line 1 "C:\\Users\\gbeve\\Downloads\\FALCO\\FALCO.ino"
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

#line 18 "C:\\Users\\gbeve\\Downloads\\FALCO\\FALCO.ino"
void updateIMU();
#line 25 "C:\\Users\\gbeve\\Downloads\\FALCO\\FALCO.ino"
void computeAccel(accel_t *desiredAccel);
#line 31 "C:\\Users\\gbeve\\Downloads\\FALCO\\FALCO.ino"
void computeQuat(quat_t *desiredQuat);
#line 38 "C:\\Users\\gbeve\\Downloads\\FALCO\\FALCO.ino"
void setup();
#line 45 "C:\\Users\\gbeve\\Downloads\\FALCO\\FALCO.ino"
void loop();
#line 18 "C:\\Users\\gbeve\\Downloads\\FALCO\\FALCO.ino"
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

