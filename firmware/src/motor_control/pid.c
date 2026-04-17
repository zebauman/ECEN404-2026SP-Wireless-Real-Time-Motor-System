#include "pid.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pid, LOG_LEVEL_INF);

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
    
    // 1. Calculate the Proportional term
    float p_term = pid->kp * error;
    
    // 2. Add to the Integral (Simpler approach)
    pid->integral += error * dt;

    // 3. Absolute Integral Limit (Your existing safety net)
    if(pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    // 4. Calculate Final Output
    float output = (pid->kp * error) + (pid->ki * pid->integral);

    // 5. Clamp Final Output (This naturally prevents windup if using an integral limit)
    if(output > pid->out_max) output = pid->out_max;
    else if(output < pid->out_min) output = pid->out_min;

    /* ── DEBUG LOGGING THROTTLE ─────────────────────────────────────────── */
    #define LOG_INTERVAL 25
    static uint32_t print_tick = 0;
    
    if (++print_tick >= LOG_INTERVAL) {
        print_tick = 0;
        
        LOG_INF("PID | Tgt: %6.1f | Meas: %6.1f | Err: %6.1f | Int: %6.2f | Out: %6.2f",
                (double)target, 
                (double)measured, 
                (double)error, 
                (double)pid->integral, 
                (double)output);
    }

    return output;
}

/* ========================================================================= *
 * RESET                                                                     *
 * ========================================================================= */
void pid_reset(pid_struct *pid){
    pid->integral = 0.0f;
}