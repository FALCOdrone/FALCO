#include "controller.h"
#include "imu.h"
#include "motor.h"
#include "pinDef.h"

#define DEBUG 1

speed_t speed;
speed_t prevSpeed;
accel_t accel;
quat_t quat;

accel_t desiredAccel;
quat_t desiredQuat;

//create kalman filter
using namespace BLA;
#define Nstate 6
#define Nobs 3
KALMAN<Nstate, Nobs> K;
BLA::Matrix<Nobs> obs;
// measurement std (to be characterized from your sensors)
#define N1 0.03 // noise on the measurement component


// model std (~1/inertia). Freedom you give to relieve your evolution equation
#define M1 0.01;

K.Q ={{M1, 0, 0, 0, 0, 0},
      {0, M1, 0, 0, 0, 0},
      {0, 0,  M1, 0, 0, 0},
      {0, 0,  0, M1,  0, 0},
      {0, 0,  0, 0, M1,  0},
      {0, 0,  0, 0, 0, M1 }};

K.R={{N1, 0,  0},
     {0,  N1, 0},
     {0,  0,  N1}};
//end

float thrust[4];

void updateIMU() {
    accel = getAcceleration();
    quat = getQuaternion();
    speed = computeLinSpeed(accel, prevSpeed);
    prevSpeed = speed;
}

void computeAccel(accel_t *desiredAccel) {
    desiredAccel->x = 0;
    desiredAccel->y = 0;
    desiredAccel->z = 0;
}

void computeQuat(quat_t *desiredQuat) {
    desiredQuat->w = 1;
    desiredQuat->x = 0;
    desiredQuat->y = 0;
    desiredQuat->z = 0;
}

void setup() {
    Serial.begin(115200);
    initializeImu();
}

void loop() {
    updateIMU();
    state = updateKALMAN(&K, accel);
    if (DEBUG) printIMUData(accel, speed, quat, state);

    computeAccel(&desiredAccel);
    computeQuat(&desiredQuat);

    stabilityPID(thrust, desiredAccel, accel, desiredQuat, quat);
    driveMotors(thrust);

    delay(100);
}
