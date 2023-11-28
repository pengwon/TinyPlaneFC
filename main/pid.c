#include "pid.h"

void pid_delta_init(delta_pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    for (int i = 0; i < 2; ++i) {
        pid->prev_error[i] = 0.0;
    }
}

float pid_delta_update(delta_pid_t *pid, float setpoint, float actual)
{
    float error = setpoint - actual;

    float output = pid->kp * (error - pid->prev_error[0]) +
                   pid->ki * error +
                   pid->kd * (error - 2 * pid->prev_error[0] + pid->prev_error[1]);

    pid->prev_error[1] = pid->prev_error[0];
    pid->prev_error[0] = error;
    
    return output;
}
