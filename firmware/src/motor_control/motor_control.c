#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "motor_control.h"
#include "motor.h"
#include "bldc_driver.h"
#include "pid.h"

LOG_MODULE_REGISTER(motor_control, LOG_LEVEL_INF);

/* ========================================================================= *
 * THREAD DEFINITIONS                                                        *
 * ========================================================================= */
#define STACK_SIZE      2048
#define PRIO_PID        5           // Medium priority: control math

#define PID_PERIOD_MS   10          // 10ms period = 100Hz control loop
#define DT              0.01f       // DT in seconds - matching PID period mS

#define HALL_TIMEOUT_MS 100U        // IF THERE'S NO HALL EFFECT READING WITHIN THIS PERIOD THEN MOTOR IS STOPPED
#define STALL_TIMEOUT_MS 2000U      // PERIOD OF TIME BEFORE MOTOR DECLARED STUCK w/ TARGET != 0 BUT RPM == 0

#define RPM_FILTER_ALPHA  0.3f      // EMA Smoothing 23ms tim constant at 100Hz

K_THREAD_STACK_DEFINE(pid_stack, STACK_SIZE);
static struct k_thread pid_thread_data;

/* ========================================================================= *
 * INTERNAL CONTROL STATE                                                    *
 * ========================================================================= */
static pid_struct   rpm_pid;                // PI CONTROLLER STATE
static float        filtered_rpm    = 0;    // EMA FILTER OUTPUT
static uint32_t     stall_ms        = 0;    // STALL ACCUMULATOR mS

extern atomic_t g_motor_speed_atomic;

// RESET HELPER WHENEVER THE MOTOR STOPS, FAULTS, OR ESTOPS SO NEXT RUN WILL START WITH A CLEAN STATE
static void reset_control_state(void)
{
    pid_reset(&rpm_pid);
    filtered_rpm = 0.0f;    // clear stale EMA value so P term for the PID isn't ruined (large negative error spike if target is low)
    stall_ms     = 0;
}

/* ========================================================================= *
 * PID CONTROL THREAD                                                        *
 * ========================================================================= */
static void pid_control_thread(void *p1, void *p2, void *p3) {
    LOG_INF("PID control thread started - %u Hz, stack %u bytes", 1000u/ PID_PERIOD_MS, STACK_SIZE);

    /** TODO: TUNE THE PI GAIN (kp FIRST W/ KI=0 until step response looks right)
     *        THEN SLOWLY ADD ki to remove steady-state error.
     */
    // INTEGRAL LIMIT = 500 RPM*s: @ki=0.01, max I contribution = 500 * 0.01 = 5% duty

    pid_init(&rpm_pid,
            /* kp */                0.05f,
            /* ki */                0.01f,
            /* integral_limit */    500.0f,
            /* out_min */           6.0f,
            /* out_max */           96.0f);


    while (1) {
        int32_t raw_rpm = (int32_t)atomic_get(&g_motor_speed_atomic);


        // HALL EFFECT SENSOR TIMEOUT - DETECT IF THE MOTOR HAS STOPPED
        uint32_t elapsed_ms = k_cyc_to_ms_near32(k_cycle_get_32() - bldc_get_last_cycle_count());
        
        if(elapsed_ms > HALL_TIMEOUT_MS){
            raw_rpm = 0;
            atomic_set(&g_motor_speed_atomic, 0);   // MOTOR IS STOPPED
        }


        // EMA FILTER - exponential moving average to smooth the hall sensor jutters
        // THE FILTERED RPM for PI calculation and the raw RPM for hall effect timeout (b/c raw responses to zero rpm faster than EMA)
        filtered_rpm = RPM_FILTER_ALPHA * (float) raw_rpm + (1.0f - RPM_FILTER_ALPHA) * filtered_rpm;


        // UPDATE motor_states for BLE telemetry
        motor_set_speed(raw_rpm);
        motor_set_filtered_speed((int32_t)filtered_rpm);


        // READ THE TARGET STATE AND SPEED
        uint8_t target_state    = motor_get_target_state();
        int32_t target_rpm      = motor_get_target_speed();


        // STALL DETECTION - MOTOR IS COMMANDED TO MOVE BUT IT'S NOT MOVING
        if(target_rpm != 0 && raw_rpm == 0){
            stall_ms += PID_PERIOD_MS;
            if(stall_ms >= STALL_TIMEOUT_MS){
                LOG_ERR("STALL: TARGET=%d RPM, no movement for %u ms",
                        target_rpm, STALL_TIMEOUT_MS);
                        motor_trigger_estop();
                        motor_set_stall_warning(true);
                        reset_control_state();
            }
        } else {
            stall_ms = 0;   // RESET ACCUMULATOR IF MOTOR IS MOVING OR TARGET = 0
        }

        
        // PID OUTPUT OR SAFE SHUTDOWN
        if(target_state == MOTOR_STATE_RUNNING_SPEED){
            float duty = pid_compute(&rpm_pid,
                                    (float)target_rpm,
                                    filtered_rpm, 
                                    DT);
            bldc_set_pwm(bldc_percent_to_pulse(duty));
        } else {
            // IF NOT ACTIVELY RUNNING (STOPPED / ESTOP / FAULT)
            bldc_set_pwm(0);
            reset_control_state();
        }

        // WAIT UNTIL THE NEXT CYCLE FOR PID PERIOD ms
        k_msleep(PID_PERIOD_MS);
    }
}

/* ========================================================================= *
 * INITIALIZATION                                                            *
 * ========================================================================= */
int motor_control_init(void) {
    LOG_INF("Initializing Motor Control...");

    k_thread_create(&pid_thread_data, pid_stack,
                    K_THREAD_STACK_SIZEOF(pid_stack),
                    pid_control_thread, NULL, NULL, NULL,
                    PRIO_PID, 0, K_NO_WAIT);

    k_thread_name_set(&pid_thread_data, "pid_ctrl");

    return 0;
}