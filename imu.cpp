#include "imu.h"

// MPU6050 IMU;
MPU6500 IMU;  // UDOO KEY

calData calib = {0};  // Calibration data
float deadZone[3] = {0.0, 0.0, 0.0};
Madgwick filter;

float prevTime = 0;

// WARNING: run this strictly when the drone is on a flat surface and not moving
void initializeImu(int calibrate) {
    Wire.begin(18, 21);  // UDOO KEY
    // Wire.begin();
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

        // Get the dead zone reading the worst values while still for 1.5 seconds
        AccelData tmp;
        float t = millis();
        while (millis() - t <= 2000) {
            IMU.update();
            IMU.getAccel(&tmp);
            deadZone[0] = max(deadZone[0], abs(tmp.accelX)*5);
            deadZone[1] = max(deadZone[1], abs(tmp.accelY)*5);
            deadZone[2] = max(deadZone[2], abs((float)(tmp.accelZ - 1.0))*5);
            delay(100);
        }

        Serial.print("Dead zone: ");
        Serial.print(deadZone[0], 6);
        Serial.print(", ");
        Serial.print(deadZone[1], 6);
        Serial.print(", ");
        Serial.println(deadZone[2], 6);

        filter.begin(0.2f);
    }
}

void getQuaternion(quat_t *quat) {
    AccelData IMUAccel;
    GyroData IMUGyro;
    IMU.update();
    unsigned long currentTime = micros();
    IMU.getAccel(&IMUAccel);
    IMU.getGyro(&IMUGyro);
    filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);

    quat->x = filter.getQuatX();
    quat->y = filter.getQuatY();
    quat->z = filter.getQuatZ();
    quat->w = filter.getQuatW();
    quat->dt = (currentTime >= quat->t) ? (currentTime - quat->t) / 1000000.0f : (currentTime + (ULONG_MAX - quat->t + 1)) / 1000000.0f;
    quat->t = currentTime;
}

speed_t computeLinSpeed(accel_t acc, speed_t prevSpeed) {
    speed_t currSpeed;

    currSpeed.t = micros();
    float ax = abs(acc.x) > deadZone[0] ? acc.x : 0.0;
    float ay = abs(acc.y) > deadZone[1] ? acc.y : 0.0;
    float az = abs(acc.z - 1.0) > deadZone[2] ? acc.z : 0.0;
    currSpeed.x = prevSpeed.x + ax * 9.81 * acc.dt;
    currSpeed.y = prevSpeed.y + ay * 9.81 * acc.dt;
    currSpeed.z = prevSpeed.z + az * 9.81 * acc.dt;

    // Apply a low pass filter to the speed values
    currSpeed.x = 0.85 * currSpeed.x + 0.15 * prevSpeed.x;
    currSpeed.y = 0.85 * currSpeed.y + 0.15 * prevSpeed.y;
    currSpeed.z = 0.85 * currSpeed.z + 0.15 * prevSpeed.z;

    return currSpeed;
}

void getAcceleration(accel_t *accel) {
    IMU.update();
    unsigned long currentTime = micros();

    AccelData tmp;

    IMU.getAccel(&tmp);

    // Save data considering the dead zone
    accel->x = tmp.accelX;
    accel->y = tmp.accelY;
    accel->z = tmp.accelZ;
    accel->dt = (currentTime >= accel->t) ? (currentTime - accel->t) / 1000000.0f : (currentTime + (ULONG_MAX - accel->t + 1)) / 1000000.0f;
    accel->t = currentTime;
}

void printIMUData(accel_t accel) {
    Serial.print("Acceleration: ");
    Serial.print(accel.x);
    Serial.print("g, ");
    Serial.print(accel.y);
    Serial.print("g, ");
    Serial.print(accel.z);
    Serial.print("g, Time:");
    Serial.print(accel.dt);
    Serial.println("s");
}

void printIMUData(speed_t speed) {
    Serial.print("Linear Speed: ");
    Serial.print(speed.x);
    Serial.print("m/s, ");
    Serial.print(speed.y);
    Serial.print("m/s, ");
    Serial.print(speed.z);
    Serial.println("m/s");
}

void printIMUData(quat_t quat) {
    Serial.print("Quaternion: ");
    Serial.print(quat.w);
    Serial.print(", ");
    Serial.print(quat.x);
    Serial.print(", ");
    Serial.print(quat.y);
    Serial.print(", ");
    Serial.println(quat.z);
}

void printIMUData(accel_t accel, speed_t speed) {
    printIMUData(accel);
    printIMUData(speed);
}

void printIMUData(accel_t accel, quat_t quat) {
    printIMUData(accel);
    printIMUData(quat);
}

void printIMUData(speed_t speed, quat_t quat) {
    printIMUData(speed);
    printIMUData(quat);
}

void printIMUData(accel_t accel, speed_t speed, quat_t quat) {
    printIMUData(accel);
    printIMUData(speed);
    printIMUData(quat);
}