#include "pid.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pid, LOG_LEVEL_DBG);

/* ========================================================================= *
 * INITIALISATION                                                            *
 * ========================================================================= */
void pid_init(pid_struct *pid, float kp, float ki,
    float integral_limit, float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->integral = 0.0f;
    pid->integral_limit = integral_limit;
    pid->out_min = out_min;
    pid->out_max = out_max;
}


/* ========================================================================= *
 * PID COMPUTE                                                               *
 * ========================================================================= */
float pid_compute(pid_struct *pid, float target, float measured, float dt) {

    float error = target - measured;
    pid->integral += error * dt;

    // CHECK IF PAST INTEGRAL LIMIT
    if(pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float output = pid->kp * error + pid->ki * pid->integral;

    if(output > pid->out_max) output = pid->out_max;
    else if(output < pid->out_min) output = pid->out_min;

    return output;
}

void pid_reset(pid_struct *pid){
    pid->integral = 0.0f;
}