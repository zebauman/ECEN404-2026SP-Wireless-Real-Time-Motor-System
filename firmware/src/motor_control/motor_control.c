#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "motor_control.h"
#include "motor.h"
#include "bldc_driver.h"
#include "pid.h"

LOG_MODULE_REGISTER(motor_control, LOG_LEVEL_INF);

/* ========================================================================= *
 * CONFIGURATION                                                             *
 * ========================================================================= */
#define STACK_SIZE          2048
#define PRIO_PID            5

#define PID_PERIOD_MS       10
#define DT                  0.01f

#define HALL_TIMEOUT_MS     100U
#define STALL_TIMEOUT_MS    5000U
#define RPM_FILTER_ALPHA    0.3f

#define LOG_EVERY_N_TICKS   100

K_THREAD_STACK_DEFINE(pid_stack, STACK_SIZE);
static struct k_thread pid_thread_data;

/* ========================================================================= *
 * INTERNAL STATE                                                            *
 * ========================================================================= */
static pid_struct   rpm_pid;
static float        filtered_rpm = 0.0f;
static uint32_t     stall_ms     = 0;

extern atomic_t g_motor_speed_atomic;

static void reset_control_state(void)
{
    pid_reset(&rpm_pid);
    filtered_rpm = 0.0f;
    stall_ms     = 0;
}

/* ========================================================================= *
 * PID CONTROL THREAD                                                        *
 * ========================================================================= */
static void pid_control_thread(void *p1, void *p2, void *p3)
{
    LOG_INF("PID control thread started - %u Hz, stack %u bytes",
            1000U / PID_PERIOD_MS, STACK_SIZE);

    pid_init(&rpm_pid,
             /* kp */             0.05f,
             /* ki */             0.01f,
             /* integral_limit */ 500.0f,
             /* out_min */        6.0f,
             /* out_max */        96.0f);

    uint32_t log_tick    = 0;
    uint8_t  last_state  = 0xFF;  // tracks transitions for bootstrap/start

    while (1) {

        /* ── 1. Read speed ───────────────────────────────────────────────── */
        int32_t raw_rpm = (int32_t)atomic_get(&g_motor_speed_atomic);

        /* ── 2. Hall timeout ─────────────────────────────────────────────── */
        uint32_t elapsed_ms = k_cyc_to_ms_near32(
                                  k_cycle_get_32() - bldc_get_last_cycle_count());
        if (elapsed_ms > HALL_TIMEOUT_MS) {
            raw_rpm = 0;
            atomic_set(&g_motor_speed_atomic, 0);
        }

        /* ── 3. EMA filter ───────────────────────────────────────────────── */
        filtered_rpm = RPM_FILTER_ALPHA * (float)raw_rpm
                     + (1.0f - RPM_FILTER_ALPHA) * filtered_rpm;

        /* ── 4. Sync to motor stats for BLE telemetry ───────────────────── */
        motor_set_speed(raw_rpm);
        motor_set_filtered_speed((int32_t)filtered_rpm);

        /* ── 5. Read target ──────────────────────────────────────────────── */
        uint8_t target_state = motor_get_target_state();
        int32_t target_rpm   = motor_get_target_speed();

        /* ── 6. Periodic status log ──────────────────────────────────────── */
        if (++log_tick >= LOG_EVERY_N_TICKS) {
            log_tick = 0;
            LOG_INF("[PID] raw=%6d  filt=%6d  tgt=%6d  "
                    "hall_age=%5ums  state=0x%02X  stall=%ums",
                    raw_rpm,
                    (int32_t)filtered_rpm,
                    target_rpm,
                    elapsed_ms,
                    motor_get_full_status(),
                    stall_ms);
        }

        /* ── 7. Stall detection ──────────────────────────────────────────── */
        if (target_rpm != 0 && raw_rpm == 0) {
            stall_ms += PID_PERIOD_MS;
            if (stall_ms >= STALL_TIMEOUT_MS) {
                LOG_ERR("STALL: target=%d RPM, no movement for %u ms",
                        target_rpm, STALL_TIMEOUT_MS);
                motor_trigger_estop();
                motor_set_stall_warning(true);
                reset_control_state();
            }
        } else {
            stall_ms = 0;
        }

        /* ── 8. PID output or safe shutdown ──────────────────────────────── */
        if (target_state == MOTOR_STATE_RUNNING_SPEED) {

            // On transition INTO running — fire softstart
            if (last_state != MOTOR_STATE_RUNNING_SPEED) {
                last_state = MOTOR_STATE_RUNNING_SPEED;
                bldc_set_running();
                LOG_INF("Motor START — softstart initiated");
            }

            float duty = pid_compute(&rpm_pid,
                                     (float)target_rpm,
                                     filtered_rpm,
                                     DT);
            bldc_set_pwm(bldc_percent_to_pulse(duty));

        } else {

            // On transition OUT of running — bootstrap and reset
            if (last_state != target_state) {
                last_state = target_state;
                LOG_WRN("PID inactive — state=0x%02X "
                        "(0x00=stopped  0x03=estop  0x05=fault)",
                        target_state);
                bldc_set_bootstrap();
                reset_control_state();
            }
        }

        k_msleep(PID_PERIOD_MS);
    }
}

/* ========================================================================= *
 * INITIALIZATION                                                            *
 * ========================================================================= */
int motor_control_init(void)
{
    LOG_INF("Initializing Motor Control...");

    k_thread_create(&pid_thread_data, pid_stack,
                    K_THREAD_STACK_SIZEOF(pid_stack),
                    pid_control_thread, NULL, NULL, NULL,
                    PRIO_PID, 0, K_NO_WAIT);

    k_thread_name_set(&pid_thread_data, "pid_ctrl");

    return 0;
}