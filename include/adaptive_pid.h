#ifndef ADAPTIVE_PID_H
#define ADAPTIVE_PID_H

#include "pico/stdlib.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float alpha; // Adaptation rate
    float integral;
    float prev_error;
    float output_min;
    float output_max;
} AdaptivePID;

void AdaptivePID_Init(AdaptivePID *pid, float Kp, float Ki, float Kd, float alpha, float output_min, float output_max);
void AdaptivePID_UpdateParameters(AdaptivePID *pid, float error, float dt);
float AdaptivePID_Compute(AdaptivePID *pid, float setpoint, float measured_value, float dt);
void AdaptivePID_example();

#endif // ADAPTIVE_PID_H
