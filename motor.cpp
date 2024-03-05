#include "motor.h"

// TODO: update with real transfer function between thrust and PWM

void driveMotors(float thrust[4]) {
    // Motor 1
    analogWrite(ESC1, map(thrust[0], 0, 100, 0, 255));
    // Motor 2
    analogWrite(ESC2, map(thrust[1], 0, 100, 0, 255));
    // Motor 3
    analogWrite(ESC3, map(thrust[2], 0, 100, 0, 255));
    // Motor 4
    analogWrite(ESC4, map(thrust[3], 0, 100, 0, 255));
}