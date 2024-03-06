# 1 "C:\\Users\\gbeve\\Downloads\\FALCO\\FALCO.ino"
// #include "controller.h"
# 3 "C:\\Users\\gbeve\\Downloads\\FALCO\\FALCO.ino" 2
// #include "motor.h"
# 5 "C:\\Users\\gbeve\\Downloads\\FALCO\\FALCO.ino" 2



vec_t pos;
vec_t speed;
vec_t accel;
quat_t quat;

vec_t prevSpeed;

vec_t desiredAccel;
quat_t desiredQuat;

// create kalman filter
using namespace BLA;

KALMAN<6, 3> K;
BLA::Matrix<3> obs;

// measurement std (to be characterized from your sensors)
// noise on the measurement component
//#define n_p 0.3 // position measurement noise


// model std (~1/inertia). Freedom you give to relieve your evolution equation



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
    K.H = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    K.Q = {0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.001};

    //     a_x, a_y, a_z
    K.R = {0.03 /* acceleration measurement noise*/, 0.0, 0.0,
           0.0, 0.03 /* acceleration measurement noise*/, 0.0,
           0.0, 0.0, 0.03 /* acceleration measurement noise*/};
}

void loop() {
    updateIMU();
    updateKALMAN(&K, &pos, &speed, &accel);
    if (1) {
        Serial.print("Position: ");
        printIMUData(pos, "m");
        Serial.print("Speed: ");
        printIMUData(speed, "m/s");
        Serial.print("Acceleration: ");
        printIMUData(accel, "m/s^2");
        Serial.print("Quaternion: ");
        printIMUData(quat);
        Serial.println();
    }

    // computeAccel(&desiredAccel);
    // computeQuat(&desiredQuat);

    // stabilityPID(thrust, desiredAccel, accel, desiredQuat, quat);
    // driveMotors(thrust);

    delay(100);
}
