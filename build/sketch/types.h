#line 1 "C:\\Users\\gbeve\\Downloads\\FALCO\\types.h"
#ifndef TYPES_H
#define TYPES_H

// Quaternion
typedef struct {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    unsigned long t = 0.0f;
    float dt = 0.0f;
} quat_t;

// Speed
typedef struct {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    unsigned long t = 0.0f;
} speed_t;

// Acceleration
typedef struct {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    unsigned long t = 0.0f;
    float dt = 0.0f;
} accel_t;

#endif