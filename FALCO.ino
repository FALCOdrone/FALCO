// #include "controller.h"
#include "imu.h"
// #include "motor.h"
#include "pinDef.h"

#define DEBUG 1

vec_t pos;
vec_t speed;
vec_t accel;
quat_t quat;

vec_t prevSpeed;

vec_t desiredAccel;
quat_t desiredQuat;

// create kalman filter
using namespace BLA;

KALMAN<Nstate, Nobs> K;
BLA::Matrix<Nobs> obs;

// measurement std (to be characterized from your sensors)
// noise on the measurement component
//#define n_p 0.3 // position measurement noise
#define n_a 0.03 // acceleration measurement noise

// model std (~1/inertia). Freedom you give to relieve your evolution equation
#define m_p 0.001
#define m_s 0.001
#define m_a 0.001

float thrust[4];

void updateIMU() {
    getAcceleration(&accel);
    getQuaternion(&quat);
    speed = computeLinSpeed(accel, prevSpeed);
    prevSpeed = speed;
}

void computeAccel(vec_t *desiredAccel) {
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
    
    //     v_x, v_y, v_z, a_x, a_y, a_z
    K.H = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    K.Q = {m_p, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, m_p, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, m_p, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, m_s, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, m_s, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, m_s, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m_a, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m_a, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m_a};

    //     a_x, a_y, a_z
    K.R = {n_a, 0.0, 0.0,
           0.0, n_a, 0.0,
           0.0, 0.0, n_a};
}

void loop() {
    updateIMU();
    updateKALMAN(&K, &pos, &speed, &accel);
    if (DEBUG) {
        /*
        Serial.print("Position: ");
        printIMUData(&pos, "m");
        Serial.print("Speed: ");
        printIMUData(&speed, "m/s");
        Serial.print("Acceleration: ");
        printIMUData(&accel, "m/s^2");
        Serial.print("Quaternion: ");
        printIMUData(&quat);
        Serial.println();*/
        logIMU(&pos, &speed, &accel);
    }

    // computeAccel(&desiredAccel);
    // computeQuat(&desiredQuat);

    // stabilityPID(thrust, desiredAccel, accel, desiredQuat, quat);
    // driveMotors(thrust);

    delay(100);
}
