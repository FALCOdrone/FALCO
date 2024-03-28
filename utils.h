#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

void loopRate(int freq);
void loopBlink();
void setupBlink(int numBlinks, int upTime, int downTime);

float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq);
float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq);

#endif