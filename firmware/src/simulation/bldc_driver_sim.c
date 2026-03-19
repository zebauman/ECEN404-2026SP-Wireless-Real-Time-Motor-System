#include "bldc_driver.h"
#include "motor_sim.h"
#include "motor.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <stdint.h>

LOG_MODULE_REGISTER(mock_bldc, LOG_LEVEL_INF);

/* ========================================================================= *
 * MOCK BLDC DRIVER                                                          *
 *                                                                           *
 * Replaces bldc_driver.c for software-only BLE + PID testing.             *
 * Build with CONFIG_MOTOR_SIM=y.                                           *
 *                                                                           *
 * Physics model:                                                            *
 *   Steady-state RPM = (pulse - PULSE_ZERO) * RPM_PER_TICK                *
 *   actual_rpm ramps toward steady-state at RPM_RAMP_PER_TICK per 10ms.   *
 *                                                                           *
 * Why inertia matters:                                                      *
 *   Without it the sim responds instantly. The PID sees its full output    *
 *   reflected as RPM immediately, overshoots massively, the integral       *
 *   winds to -500 (its limit), and the output gets stuck well below        *
 *   target forever. The ramp gives the PI controller something physically  *
 *   realistic to control.                                                   *
 * ========================================================================= */

#define TIM1_ARR            3200    // Must match bldc_driver.c
#define PULSE_ZERO          192     // Below this = no torque (~6% duty)
#define RPM_PER_TICK        2.0f    // RPM per pulse tick above PULSE_ZERO

// How fast the simulated motor accelerates/decelerates per 10ms tick.
// 300 RPM/tick = 0→3000 RPM in ~100ms. Tune to match your real motor's feel.
#define RPM_RAMP_PER_TICK   300

// Fixed sim period — must match PID_PERIOD_MS in motor_control.c (10ms).
// Old code used variable sleep based on RPM which drifted out of phase with
// the PID thread, causing the sim to read stale pulse values each wake.
#define SIM_PERIOD_MS       10

#define SIM_STACK_SIZE      512
#define SIM_PRIO            6       // Below PID (5), above telemetry (7)

K_THREAD_STACK_DEFINE(sim_stack, SIM_STACK_SIZE);
static struct k_thread sim_thread_data;

/* ========================================================================= *
 * INTERNAL STATE                                                            *
 * ========================================================================= */
static atomic_t sim_pulse_atomic      = ATOMIC_INIT(0);
static atomic_t sim_last_cycle_atomic = ATOMIC_INIT(0);

atomic_t g_motor_speed_atomic = ATOMIC_INIT(0);

/* ========================================================================= *
 * HALL SIMULATION THREAD                                                    *
 * ========================================================================= */
static void sim_thread_fn(void *p1, void *p2, void *p3)
{
    int32_t actual_rpm = 0;     // Current simulated RPM — ramps toward target

    while (1) {
        int pulse = (int)atomic_get(&sim_pulse_atomic);

        if (pulse <= PULSE_ZERO) {
            // Ramp DOWN instead of snapping to 0.
            // Snapping to 0 causes the stall detector to fire during normal
            // deceleration: target!=0, rpm==0 → stall after STALL_TIMEOUT_MS.
            actual_rpm -= RPM_RAMP_PER_TICK;
            if (actual_rpm < 0) {
                actual_rpm = 0;
            }

            if (actual_rpm == 0) {
                // Fully stopped — stop refreshing timestamp so PID hall-timeout
                // correctly detects the motor as stopped, same as real hardware.
                k_msleep(SIM_PERIOD_MS);
                continue;
            }

        } else {
            int32_t target_rpm = (int32_t)((pulse - PULSE_ZERO) * RPM_PER_TICK);

            // Ramp toward target — prevents instant response that winds up
            // the PI integral on overshoot, which was the root cause of the
            // pulse=2012→265→722→654 oscillation seen in testing.
            if (actual_rpm < target_rpm) {
                actual_rpm += RPM_RAMP_PER_TICK;
                if (actual_rpm > target_rpm) {
                    actual_rpm = target_rpm;
                }
            } else if (actual_rpm > target_rpm) {
                actual_rpm -= RPM_RAMP_PER_TICK;
                if (actual_rpm < target_rpm) {
                    actual_rpm = target_rpm;
                }
            }
        }

        // Write current RPM for PID thread
        atomic_set(&g_motor_speed_atomic, (atomic_val_t)actual_rpm);

        // Refresh hall-edge timestamp — keeps PID watchdog alive while moving
        atomic_set(&sim_last_cycle_atomic, (atomic_val_t)k_cycle_get_32());

        k_msleep(SIM_PERIOD_MS);
    }
}

/* ========================================================================= *
 * PUBLIC API                                                                *
 * ========================================================================= */

int bldc_driver_init(void)
{
    LOG_INF("================================================");
    LOG_INF("  MOCK BLDC DRIVER — NO HARDWARE WILL ACTUATE  ");
    LOG_INF("  ARR=%-4d  PULSE_ZERO=%-3d  RPM/TICK=%.1f     ",
            TIM1_ARR, PULSE_ZERO, (double)RPM_PER_TICK);
    LOG_INF("  Max RPM: %.0f  Ramp: %d RPM per %dms         ",
            (double)((TIM1_ARR - PULSE_ZERO) * RPM_PER_TICK),
            RPM_RAMP_PER_TICK, SIM_PERIOD_MS);
    LOG_INF("================================================");

    atomic_set(&sim_last_cycle_atomic, (atomic_val_t)k_cycle_get_32());
    return 0;
}

void motor_sim_init(void)
{
    // Re-seed here too — bldc_driver_init() may have been called early enough
    // that the gap triggers a false hall timeout before the thread starts
    atomic_set(&sim_last_cycle_atomic, (atomic_val_t)k_cycle_get_32());

    k_thread_create(&sim_thread_data, sim_stack,
                    K_THREAD_STACK_SIZEOF(sim_stack),
                    sim_thread_fn, NULL, NULL, NULL,
                    SIM_PRIO, 0, K_NO_WAIT);

    k_thread_name_set(&sim_thread_data, "hall_sim");
    LOG_INF("Hall sim thread started (prio=%d period=%dms ramp=%d RPM/tick)",
            SIM_PRIO, SIM_PERIOD_MS, RPM_RAMP_PER_TICK);
}

void bldc_set_pwm(int pulse)
{
    if (pulse > TIM1_ARR) pulse = TIM1_ARR;
    if (pulse < 0)        pulse = 0;

    atomic_set(&sim_pulse_atomic, (atomic_val_t)pulse);

    // Threshold reduced from 50 to 10 so settling is visible in the log.
    // At threshold=50, a PI converging the last 130 RPM of error (~4 pulse
    // ticks at these gains) would never trigger a log line — looks frozen.
    static int last_logged = -999;
    if (abs(pulse - last_logged) > 10) {
        last_logged = pulse;
        LOG_INF("[SIM PWM] pulse=%4d (%4.1f%%) actual_rpm=%-5d target_rpm=%-5d",
                pulse,
                (double)(pulse * 100.0f / TIM1_ARR),
                (int32_t)atomic_get(&g_motor_speed_atomic),
                motor_get_target_speed());
    }
}

void bldc_set_commutation(uint8_t step)
{
    static uint8_t last_step = 0xFF;
    if (step != last_step) {
        last_step = step;
        if (step != 0 && step != 7) {
            LOG_DBG("[SIM COMM] step=%u (%s)", step, step > 8 ? "CCW" : "CW");
        }
    }
}

int bldc_read_hall_state(void)
{
    static const uint8_t HALL_SEQ[6] = {1, 5, 4, 6, 2, 3};
    static uint8_t idx = 0;

    int32_t rpm = (int32_t)atomic_get(&g_motor_speed_atomic);
    if (abs(rpm) < 10) {
        return HALL_SEQ[idx];
    }
    idx = (rpm > 0) ? (idx + 1) % 6 : (idx + 5) % 6;
    return HALL_SEQ[idx];
}

int bldc_percent_to_pulse(float percent_duty_cycle)
{
    int pulse = (int)(percent_duty_cycle * 32.0f);
    if (pulse > TIM1_ARR) pulse = TIM1_ARR;
    if (pulse < 0)        pulse = 0;
    return pulse;
}

uint32_t bldc_get_last_cycle_count(void)
{
    return (uint32_t)atomic_get(&sim_last_cycle_atomic);
}

void bldc_set_direction(int ccw)
{
    LOG_INF("[SIM DIR] %s", ccw ? "CCW" : "CW");
}