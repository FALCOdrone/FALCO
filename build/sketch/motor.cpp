#line 1 "C:\\Users\\gbeve\\Downloads\\FALCO\\motor.cpp"
#include "motor.h"

// TODO: update with real transfer function between thrust and PWM

void driveMotors(float thrust[4]) {
    // Motor 1
    analogWrite(M1, map(thrust[0], 0, 100, 0, 255));
    // Motor 2
    analogWrite(M2, map(thrust[1], 0, 100, 0, 255));
    // Motor 3
    analogWrite(M3, map(thrust[2], 0, 100, 0, 255));
    // Motor 4
    analogWrite(M4, map(thrust[3], 0, 100, 0, 255));
}