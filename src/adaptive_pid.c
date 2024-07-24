#include "adaptive_pid.h"
#include <stdio.h>

void AdaptivePID_Init(AdaptivePID *pid, float Kp, float Ki, float Kd, float alpha, float output_min, float output_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->alpha = alpha;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output_min = output_min;
    pid->output_max = output_max;
}

void AdaptivePID_UpdateParameters(AdaptivePID *pid, float error, float dt) {
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;

    // Update PID parameters
    pid->Kp += pid->alpha * error;
    pid->Ki += pid->alpha * pid->integral;
    pid->Kd += pid->alpha * derivative;

    pid->prev_error = error;
}

float AdaptivePID_Compute(AdaptivePID *pid, float setpoint, float measured_value, float dt) {
    float error = setpoint - measured_value;
    AdaptivePID_UpdateParameters(pid, error, dt);

    float P = pid->Kp * error;
    float I = pid->Ki * pid->integral;
    float D = pid->Kd * ((error - pid->prev_error) / dt);

    float output = P + I + D;

    // Clamp output to min/max range
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    return output;
}

void AdaptivePID_example() {
    AdaptivePID pid;
    AdaptivePID_Init(&pid, 1.0, 0.5, 0.1, 0.01, 0.0, 100.0);

    float setpoint = 50.0; // Desired temperature
    float dt = 1.0; // Time step in seconds

    while (true) {
        float current_temperature = 25.0;
        float power = AdaptivePID_Compute(&pid, setpoint, current_temperature, dt);
        printf("power: %.2f\n", power);
        // set_heater_power(power);

        // Simulate time delay
        sleep_ms(dt * 1000);
    }
}
