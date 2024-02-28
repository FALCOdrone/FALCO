#ifndef PINDEF_H
#define PINDEF_H

// ESCs
#define M1 23
#define M2 33
#define M3 2
#define M4 12

// Ultrasonic distance sensors
#define US1_TRIG 22
#define US1_ECHO 21
#define US2_TRIG 35
#define US2_ECHO 34
#define US3_TRIG 4
#define US3_ECHO 3
#define US4_TRIG 27
#define US4_ECHO 26

// Buzzer
#define BUZZER 11

// Camera servo
#define CAM_SERVO 20

// PDB
#define VBAT 38  // A14
#define CURR 39  // A15

// GPS (UART7)
#define GPS_RX 28
#define GPS_TX 29

// Radio (UART6)
#define RADIO_RX 25
#define RADIO_TX 24

// Jetson (UART1)
#define JETSON_RX 0
#define JETSON_TX 1

// IMU Address
#define IMU_ADDR 0x68

// Barometer Address
#define BARO_ADDR 0x76  // TODO

// Magnetometer Address
#define MAG_ADDR  // TODO

#endif