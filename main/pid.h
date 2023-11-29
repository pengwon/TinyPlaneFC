#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#define INTEGRAL_MAX 2000

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error[2];
} delta_pid_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
} abs_pid_t;

void pid_delta_init(delta_pid_t *pid, float kp, float ki, float kd);
float pid_delta_update(delta_pid_t *pid, float setpoint, float actual);
void pid_init(abs_pid_t *pid, float kp, float ki, float kd);
float pid_update(abs_pid_t *pid, float setpoint, float actual);

#ifdef __cplusplus
}
#endif
