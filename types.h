#ifndef TYPES_H
#define TYPES_H

// Quaternion
typedef struct {
    float w;
    float x;
    float y;
    float z;
    unsigned long t;
    float dt;
} quat_t;

// Speed
typedef struct {
    float x;
    float y;
    float z;
    unsigned long t;
} speed_t;

// Acceleration
typedef struct {
    float x;
    float y;
    float z;
    unsigned long t;
    float dt;
} accel_t;

#endif