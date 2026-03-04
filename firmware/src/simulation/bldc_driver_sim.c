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
 * Replaces bldc_driver.c entirely for software-only testing.               *
 * Build with CONFIG_MOTOR_SIM=y (or swap the source file in CMakeLists).   *
 *                                                                           *
 * What this mock does:                                                      *
 *   - bldc_set_pwm()     → converts pulse to simulated RPM, writes atomic  *
 *   - A background thread fires at hall-edge intervals, writing             *
 *     g_motor_speed_atomic just like the real hall ISR does. This keeps     *
 *     last_cycle_atomic fresh so the PID watchdog behaves correctly.        *
 *   - bldc_get_last_cycle_count() returns the real timestamp of the last    *
 *     simulated hall edge — the PID timeout works exactly as on hardware.   *
 *                                                                           *
 * Physics model (linear, tune to your motor):                              *
 *   RPM = (pulse - PULSE_ZERO) * RPM_PER_TICK   for pulse > PULSE_ZERO    *
 *   RPM = 0                                      for pulse <= PULSE_ZERO   *
 *                                                                           *
 *   At max pulse (3072): (3072 - 192) * 2.0 = 5760 RPM                    *
 *   Adjust RPM_PER_TICK to match your motor's actual kV.                   *
 * ========================================================================= */

#define TIM1_ARR        3200        // Must match bldc_driver.c
#define PULSE_ZERO      192         // Threshold below which no torque (~6% duty)
#define RPM_PER_TICK    2.0f        // RPM per pulse tick above PULSE_ZERO

/* Hall sim thread: fires at the simulated hall-edge interval.
 * Stack sized for a minimal periodic thread — no float, no log in the loop. */
#define SIM_STACK_SIZE  512
#define SIM_PRIO        6           // Below PID (5), above telemetry (7)

K_THREAD_STACK_DEFINE(sim_stack, SIM_STACK_SIZE);
static struct k_thread sim_thread_data;

/* ========================================================================= *
 * INTERNAL STATE                                                            *
 * ========================================================================= */

/* sim_pulse: written by bldc_set_pwm() (PID thread), read by sim thread.
 * Declared atomic so the sim thread reads a consistent value without a mutex. */
static atomic_t sim_pulse_atomic = ATOMIC_INIT(0);

/* Last simulated hall-edge cycle count — mirrors last_cycle_atomic in the
 * real driver. Written by sim thread, read by PID thread via
 * bldc_get_last_cycle_count(). Must be atomic for the same reason.         */
static atomic_t sim_last_cycle_atomic = ATOMIC_INIT(0);
atomic_t g_motor_speed_atomic = ATOMIC_INIT(0);


/* ========================================================================= *
 * HALL SIMULATION THREAD                                                    *
 *                                                                           *
 * Mimics what the hall ISR does in the real driver:                        *
 *   1. Compute RPM from current pulse                                       *
 *   2. Write g_motor_speed_atomic                                           *
 *   3. Update sim_last_cycle_atomic (so PID watchdog sees fresh timestamp)  *
 *   4. Sleep until the next hall edge would fire                            *
 *                                                                           *
 * When pulse <= PULSE_ZERO (motor stopped), the thread sleeps for a long    *
 * interval and does NOT update sim_last_cycle_atomic — this lets the PID   *
 * thread's hall-timeout fire correctly, exactly as on real hardware.       *
 * ========================================================================= */
static void sim_thread_fn(void *p1, void *p2, void *p3)
{
    /* Pole pairs × 6 hall states = edges per mechanical revolution */
    const uint32_t POLE_PAIRS      = 8U;
    const uint32_t STEPS_PER_REV   = POLE_PAIRS * 6U;  // = 48

    while (1) {
        int pulse = (int)atomic_get(&sim_pulse_atomic);

        if (pulse <= PULSE_ZERO) {
            /* Motor stopped: do NOT refresh sim_last_cycle_atomic.
             * The PID thread will see a stale timestamp and correctly
             * declare the motor stopped after HALL_TIMEOUT_MS.             */
            k_msleep(20);
            continue;
        }

        /* Compute simulated RPM from pulse */
        int32_t sim_rpm = (int32_t)((pulse - PULSE_ZERO) * RPM_PER_TICK);

        /* Write speed for PID thread to consume (mirrors real hall ISR) */
        atomic_set(&g_motor_speed_atomic, (atomic_val_t)sim_rpm);

        /* Refresh hall-edge timestamp so PID watchdog doesn't false-trip */
        atomic_set(&sim_last_cycle_atomic, (atomic_val_t)k_cycle_get_32());

        /* Sleep for one simulated hall-edge period:
         *   steps_per_sec = (RPM / 60) * STEPS_PER_REV
         *   ms_per_step   = 1000 / steps_per_sec
         * Minimum 1ms to prevent busy-loop at very high simulated RPM.     */
        uint32_t steps_per_sec = ((uint32_t)sim_rpm * STEPS_PER_REV) / 60U;
        uint32_t ms_per_step   = (steps_per_sec > 0)
                                 ? (1000U / steps_per_sec)
                                 : 20U;
        if (ms_per_step < 1U) ms_per_step = 1U;

        k_msleep(ms_per_step);
    }
}

/* ========================================================================= *
 * PUBLIC API — mirrors bldc_driver.h exactly                               *
 * ========================================================================= */

int bldc_driver_init(void)
{
    LOG_INF("================================================");
    LOG_INF("  MOCK BLDC DRIVER — NO HARDWARE WILL ACTUATE  ");
    LOG_INF("  ARR=%-4d  PULSE_ZERO=%-3d  RPM/TICK=%.1f     ",
            TIM1_ARR, PULSE_ZERO, (double)RPM_PER_TICK);
    LOG_INF("  Max simulated RPM: %.0f                       ",
            (double)((TIM1_ARR - PULSE_ZERO) * RPM_PER_TICK));
    LOG_INF("================================================");

    /* Seed the last-cycle timestamp to NOW so the PID watchdog doesn't
     * fire a false stall on the very first iteration at boot.              */
    atomic_set(&sim_last_cycle_atomic, (atomic_val_t)k_cycle_get_32());

    return 0;
}

void motor_sim_init(void)
{
    atomic_set(&sim_last_cycle_atomic, (atomic_val_t)k_cycle_get_32());

    k_thread_create(&sim_thread_data, sim_stack,
                    K_THREAD_STACK_SIZEOF(sim_stack),
                    sim_thread_fn, NULL, NULL, NULL,
                    SIM_PRIO, 0, K_NO_WAIT);

    k_thread_name_set(&sim_thread_data, "hall_sim");

    LOG_INF("Hall simulation thread started (prio %d)", SIM_PRIO);
}

void bldc_set_pwm(int pulse)
{
    if (pulse > TIM1_ARR) pulse = TIM1_ARR;
    if (pulse < 0)        pulse = 0;

    atomic_set(&sim_pulse_atomic, (atomic_val_t)pulse);

    /* Throttled log — only fires when duty changes by >50 ticks (~1.6%) */
    static int last_logged = -999;
    if (abs(pulse - last_logged) > 50) {
        last_logged = pulse;
        int32_t sim_rpm = (pulse > PULSE_ZERO)
                          ? (int32_t)((pulse - PULSE_ZERO) * RPM_PER_TICK)
                          : 0;
        LOG_INF("[MOCK PWM] pulse=%d (%.1f%%) -> sim_rpm=%d | target=%d RPM",
                pulse,
                (double)(pulse * 100.0f / TIM1_ARR),
                sim_rpm,
                motor_get_target_speed());
    }
}

void bldc_set_commutation(uint8_t step)
{
    /* In the mock, commutation has no effect — phases aren't real.
     * Log only on change to avoid 100Hz spam.                              */
    static uint8_t last_step = 0xFF;
    if (step != last_step) {
        last_step = step;
        if (step != 0 && step != 7) {
            LOG_DBG("[MOCK COMM] step=%u (%s)", step, step > 8 ? "CCW" : "CW");
        }
    }
}

int bldc_read_hall_state(void)
{
    /* Static hall sequence for 8 pole-pair CW rotation */
    static const uint8_t HALL_SEQ[6] = {1, 5, 4, 6, 2, 3};
    static uint8_t idx = 0;

    int32_t sim_rpm = (int32_t)atomic_get(&g_motor_speed_atomic);
    if (abs(sim_rpm) < 10) {
        return HALL_SEQ[idx];   // Motor stopped, hold current state
    }

    idx = (sim_rpm > 0) ? (idx + 1) % 6
                        : (idx + 5) % 6;   // CCW walks backward
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
    LOG_INF("[MOCK DIR] %s", ccw ? "CCW" : "CW");
}