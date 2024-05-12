/*
  Arduino LSM6DSOX - Simple Accelerometer

  This example reads the acceleration values from the LSM6DSOX
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano RP2040 Connect

  created 10 May 2021
  by Arturo Guadalupi

  This example code is in the public domain.
*/

#include <Arduino_LSM6DSOX.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  /*
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");
  */
}

void loop() {
  float x, y, z;
  float xg, yg, zg;
  float temp;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.temperatureAvailable()) {
    IMU.readAcceleration(x, y, z);
    IMU.readGyroscope(xg, yg, zg);
    IMU.readTemperatureFloat(temp);

    Serial.print(x);
    Serial.print(',');
    Serial.print(y);
    Serial.print(',');
    Serial.print(z);
    Serial.print(',');
    Serial.print(xg);
    Serial.print(',');
    Serial.print(yg);
    Serial.print(',');
    Serial.print(zg);
    Serial.print(',');
    Serial.println(temp);
  }
}
