#include "imu.h"

MPU6050 IMU;  // Change to the name of any supported IMU!

calData calib = {0};  // Calibration data
Madgwick filter;

float prevTime = 0;

// WARNING: run this strictly when the drone is on a flat surface and not moving
void initializeImu(int calibrate) {
    Wire.begin();
    Wire.setClock(400000);  // 400khz clock

    int err = IMU.init(calib, IMU_ADDR);
    if (err != 0) {
        Serial.print("Error initializing IMU: ");
        Serial.println(err);
        while (true) {
            ;
        }
    }

    if (calibrate) {
        Serial.println("Keep IMU level.");
        delay(5000);
        IMU.calibrateAccelGyro(&calib);
        Serial.println("Calibration done!");
        Serial.println("Accel biases X/Y/Z: ");
        Serial.print(calib.accelBias[0]);
        Serial.print(", ");
        Serial.print(calib.accelBias[1]);
        Serial.print(", ");
        Serial.println(calib.accelBias[2]);
        Serial.println("Gyro biases X/Y/Z: ");
        Serial.print(calib.gyroBias[0]);
        Serial.print(", ");
        Serial.print(calib.gyroBias[1]);
        Serial.print(", ");
        Serial.println(calib.gyroBias[2]);
        delay(2000);
        IMU.init(calib, IMU_ADDR);

        filter.begin(0.2f);
    }
}

quat_t getQuaternion() {
    AccelData IMUAccel;
    GyroData IMUGyro;
    quat_t quat;
    IMU.update();
    unsigned long currentTime = micros();
    IMU.getAccel(&IMUAccel);
    IMU.getGyro(&IMUGyro);
    filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);

    quat.x = filter.getQuatX();
    quat.y = filter.getQuatY();
    quat.z = filter.getQuatZ();
    quat.w = filter.getQuatW();
    quat.dt = (currentTime >= quat.t) ? (currentTime - quat.t) / 1000000.0 : (currentTime + (ULONG_MAX - quat.t + 1)) / 1000000.0;
    quat.t = currentTime;

    return quat;
}

speed_t computeLinSpeed(accel_t acc, speed_t prevSpeed) {
    speed_t currSpeed;

    currSpeed.t = micros();

    currSpeed.x = prevSpeed.x + acc.x * acc.dt;
    currSpeed.y = prevSpeed.y + acc.y * acc.dt;
    currSpeed.z = prevSpeed.z + acc.z * acc.dt;

    // Apply a low pass filter to the speed values
    currSpeed.x = 0.9 * currSpeed.x + 0.1 * prevSpeed.x;
    currSpeed.y = 0.9 * currSpeed.y + 0.1 * prevSpeed.y;
    currSpeed.z = 0.9 * currSpeed.z + 0.1 * prevSpeed.z;

    return currSpeed;
}

accel_t getAcceleration() {
    IMU.update();
    unsigned long currentTime = micros();

    accel_t accel;
    AccelData tmp;

    IMU.getAccel(&tmp);

    accel.x = tmp.accelX;
    accel.y = tmp.accelY;
    accel.z = tmp.accelZ;
    accel.dt = (currentTime >= accel.t) ? (currentTime - accel.t) / 1000000.0 : (currentTime + (ULONG_MAX - accel.t + 1)) / 1000000.0;
    accel.t = currentTime;

    return accel;
}

void printIMUData(accel_t accel, speed_t speed, quat_t quat) {
    Serial.print("Linear Speed: ");
    Serial.print(speed.x);
    Serial.print("m/s, ");
    Serial.print(speed.y);
    Serial.print("m/s, ");
    Serial.print(speed.z);
    Serial.println("m/s");

    Serial.print("Quaternion: ");
    Serial.print(quat.w);
    Serial.print(", ");
    Serial.print(quat.x);
    Serial.print(", ");
    Serial.print(quat.y);
    Serial.print(", ");
    Serial.println(quat.z);

    Serial.print("Acceleration: ");
    Serial.print(accel.x * 9.81);
    Serial.print("m/s^2, ");
    Serial.print(accel.y * 9.81);
    Serial.print("m/s^2, ");
    Serial.print(accel.z * 9.81);
    Serial.print("m/s^2, Time:");
    Serial.print(accel.dt);
    Serial.println("s");
}
