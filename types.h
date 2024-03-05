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

typedef struct {
    float x;
    float y;
    float z;
} pos_t;

typedef struct {
    speed_t s; 
    pos_t p;
} state_t; // state variable

#endif