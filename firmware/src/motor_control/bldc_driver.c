#include "bldc_driver.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <soc.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_bus.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bldc_driver, LOG_LEVEL_INF);

/* ========================================================================= *
 * HARDWARE CONSTANTS                                                        *
 * ========================================================================= */

// STM32WB55 @ 64MHz, APB2 prescaler=1 → TIM1 = 64MHz
// ARR=3200 → 20kHz PWM
#define TIM1_ARR            3200
#define DEADTIME_TICKS      50          // 50/64MHz = 781ns
#define POLE_PAIRS          8

/* ── Debounce ──────────────────────────────────────────────────────────────
 * 5000us = hand winding
 * 50us   = real motor with IHM08M1 stacked
 * !! Change to 50 before stacking IHM08M1 !!                             */
#define HALL_DEBOUNCE_US    50

/* ── Duty cycle constants ───────────────────────────────────────────────────
 * BOOTSTRAP_DUTY: CCR value that makes low-side complementary ON for ~5%
 *   of cycle, keeping IHM08M1 bootstrap caps charged when motor is stopped.
 *   Set to 95% of ARR so CHxN is active during the remaining 5%.
 *
 * SOFTSTART_DUTY / SOFTSTART_STEP: ramp from 10% → 15% on each hall edge
 *   before PID takes over. Prevents jerk on startup.                      */
#define BOOTSTRAP_DUTY      ((TIM1_ARR * 95) / 100)   // 3040 counts
#define SOFTSTART_DUTY      ((TIM1_ARR * 10) / 100)   //  320 counts
#define SOFTSTART_STEP      ((TIM1_ARR *  1) / 100)   //   32 counts/edge

/* ========================================================================= *
 * COMMUTATION LOOKUP TABLES                                                 *
 * ========================================================================= *
 *
 * Confirmed by observation: direct mapping (case==state) = CCW.
 * CCW sequence: 6→4→5→1→3→2→(repeat)
 *
 * Pin assignments:
 *   PA8 /CH1  = U+   PB13/CH1N = U-
 *   PA9 /CH2  = V+   PB14/CH2N = V-
 *   PA10/CH3  = W+   PB15/CH3N = W-
 *
 * CW = swap + and - on every pair:
 *   state 0x1 → case 6 (V+W-)    state 0x2 → case 5 (U+V-)
 *   state 0x3 → case 4 (U+W-)    state 0x4 → case 3 (W+U-)
 *   state 0x5 → case 2 (V+U-)    state 0x6 → case 1 (W+V-)
 *
 * CCW = direct mapping (case==state), confirmed working.                  */

static const uint8_t cw_commutation[8] = {
    0, 6, 5, 4, 3, 2, 1, 0
};
static const uint8_t ccw_commutation[8] = {
    0, 1, 2, 3, 4, 5, 6, 0
};

/* ========================================================================= *
 * GPIO DEFINITIONS                                                          *
 * ========================================================================= */
static const struct gpio_dt_spec hall_u = GPIO_DT_SPEC_GET(DT_ALIAS(hall_u), gpios);
static const struct gpio_dt_spec hall_v = GPIO_DT_SPEC_GET(DT_ALIAS(hall_v), gpios);
static const struct gpio_dt_spec hall_w = GPIO_DT_SPEC_GET(DT_ALIAS(hall_w), gpios);

/* ========================================================================= *
 * STATE                                                                     *
 * ========================================================================= */
static struct gpio_callback hall_u_cb;
static struct gpio_callback hall_v_cb;
static struct gpio_callback hall_w_cb;

atomic_t g_motor_speed_atomic     = ATOMIC_INIT(0);
static atomic_t last_cycle_atomic = ATOMIC_INIT(0);

static volatile int  current_direction_ccw = 0;
static volatile bool motor_running         = false;
static volatile int  softstart_pulse       = SOFTSTART_DUTY;

static void hall_isr_callback(const struct device *dev,
                               struct gpio_callback *cb, uint32_t pins);

/* ========================================================================= *
 * INITIALIZATION                                                            *
 * ========================================================================= */
int bldc_driver_init(void)
{
    LOG_INF("Initializing BLDC driver...");

    if (!gpio_is_ready_dt(&hall_u) ||
        !gpio_is_ready_dt(&hall_v) ||
        !gpio_is_ready_dt(&hall_w)) {
        LOG_ERR("Hall GPIOs not ready");
        return -ENODEV;
    }

    // Active-low open-drain sensors — internal pull-up redundant but harmless
    gpio_pin_configure_dt(&hall_u, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&hall_v, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&hall_w, GPIO_INPUT | GPIO_PULL_UP);

    int boot_state = bldc_read_hall_state();
    LOG_INF("Boot hall state: 0x%X  %s", boot_state,
            (boot_state == 0 || boot_state == 7)
            ? "*** INVALID — rotate shaft slightly ***" : "OK");

    /* ── TIM1 ──────────────────────────────────────────────────────────── */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    LL_TIM_SetPrescaler(TIM1, 0);
    LL_TIM_SetAutoReload(TIM1, TIM1_ARR - 1);
    LL_TIM_EnableARRPreload(TIM1);

    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);

    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);

    // Disabled channels driven LOW — prevents floating gate driver inputs
    LL_TIM_SetOffStates(TIM1, LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);
    LL_TIM_OC_SetDeadTime(TIM1, DEADTIME_TICKS);

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_GenerateEvent_UPDATE(TIM1);

    // Start in bootstrap state — charges IHM08M1 bootstrap caps
    // Motor will not move until bldc_set_running() is called
    bldc_set_bootstrap();

    atomic_set(&last_cycle_atomic, (atomic_val_t)k_cycle_get_32());

    /* ── Hall interrupts ────────────────────────────────────────────────── */
    gpio_pin_interrupt_configure_dt(&hall_u, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_v, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_w, GPIO_INT_EDGE_BOTH);

    gpio_init_callback(&hall_u_cb, hall_isr_callback, BIT(hall_u.pin));
    gpio_init_callback(&hall_v_cb, hall_isr_callback, BIT(hall_v.pin));
    gpio_init_callback(&hall_w_cb, hall_isr_callback, BIT(hall_w.pin));

    gpio_add_callback_dt(&hall_u, &hall_u_cb);
    gpio_add_callback_dt(&hall_v, &hall_v_cb);
    gpio_add_callback_dt(&hall_w, &hall_w_cb);

    LOG_INF("BLDC ready — 20kHz PWM  781ns dead-time  bootstrap active");
    return 0;
}

/* ========================================================================= *
 * BOOTSTRAP STATE (motor stopped)                                          *
 * ========================================================================= *
 * All three low-side complementary outputs held ON at ~5% duty.
 * This keeps the IHM08M1 bootstrap capacitors charged continuously.
 * High-side outputs are disabled.
 * Must be called on stop/estop. Wait 100ms before calling bldc_set_running().*/
void bldc_set_bootstrap(void)
{
    motor_running   = false;
    softstart_pulse = SOFTSTART_DUTY;

    TIM1->CCER &= ~(TIM_CCER_CC1E  | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E  | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E  | TIM_CCER_CC3NE);

    // CCR at 95% → complementary outputs active for remaining 5%
    TIM1->CCR1 = BOOTSTRAP_DUTY;
    TIM1->CCR2 = BOOTSTRAP_DUTY;
    TIM1->CCR3 = BOOTSTRAP_DUTY;

    // Enable only low-side complementary channels
    TIM1->CCER |= TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
    LL_TIM_GenerateEvent_UPDATE(TIM1);

    LOG_INF("Bootstrap: low-sides active, high-sides off");
}

/* ========================================================================= *
 * MOTOR START                                                               *
 * ========================================================================= */
void bldc_set_running(void)
{
    softstart_pulse = SOFTSTART_DUTY;
    motor_running   = true;

    uint8_t state = (uint8_t)bldc_read_hall_state();
    if (state != 0 && state != 7) {
        bldc_set_commutation_with_duty(state, softstart_pulse);
        LOG_INF("Motor start: hall=0x%X duty=%d", state, softstart_pulse);
    }
}

/* ========================================================================= *
 * HALL SENSOR ISR                                                           *
 * ========================================================================= */
static void hall_isr_callback(const struct device *dev,
                               struct gpio_callback *cb, uint32_t pins)
{
    uint32_t now            = k_cycle_get_32();
    uint32_t past           = (uint32_t)atomic_get(&last_cycle_atomic);
    uint32_t dt_cycles      = now - past;
    uint32_t cycles_per_sec = sys_clock_hw_cycles_per_sec();

    uint32_t dt_us = (uint32_t)(((uint64_t)dt_cycles * 1000000U) / cycles_per_sec);
    if (dt_us < HALL_DEBOUNCE_US) return;

    atomic_set(&last_cycle_atomic, (atomic_val_t)now);

    uint8_t raw_step = (uint8_t)bldc_read_hall_state();
    if (raw_step == 0 || raw_step == 7) return;

    if (!motor_running) {
        atomic_set(&g_motor_speed_atomic, 0);
        return;
    }

    // Softstart ramp — each hall edge increments duty until PID takes over
    if (softstart_pulse < bldc_percent_to_pulse(15.0f)) {
        softstart_pulse += SOFTSTART_STEP;
    }

    // Apply commutation with correct duty for this step
    bldc_set_commutation_with_duty(raw_step, softstart_pulse);

    // RPM: 48 edges/rev (8 pole pairs × 6 steps)
    // mech_rpm = (60 × cycles_per_sec) / (48 × dt_cycles)
    //          = (cycles_per_sec × 5) / (dt_cycles × 4)
    int32_t mech_rpm = (int32_t)(((uint64_t)cycles_per_sec * 5ULL)
                                 / ((uint64_t)dt_cycles    * 4ULL));

    atomic_set(&g_motor_speed_atomic,
               (atomic_val_t)(current_direction_ccw ? -mech_rpm : mech_rpm));
}

/* ========================================================================= *
 * SENSOR READ                                                               *
 * ========================================================================= */
int bldc_read_hall_state(void)
{
    // Active-low open-drain — invert so 1 = sensor triggered = magnet aligned
    int u = !gpio_pin_get_dt(&hall_u);
    int v = !gpio_pin_get_dt(&hall_v);
    int w = !gpio_pin_get_dt(&hall_w);
    return (u << 2) | (v << 1) | w;
}

/* ========================================================================= *
 * COMMUTATION WITH PER-STEP DUTY (from partner's working code)             *
 * ========================================================================= *
 * High-side CCR  = pulse     → PWMs at requested duty cycle
 * Low-side CCR   = 0         → complementary output ON full cycle
 *                               provides solid return current path
 *
 * This is the key difference from the old approach of setting all CCRs
 * identically. Setting low-side CCR=0 ensures the return path is always
 * conducting when the high-side switches.                                  */
void bldc_set_commutation_with_duty(uint8_t hall_state, int pulse)
{
    if (pulse > TIM1_ARR) pulse = TIM1_ARR;
    if (pulse < 0)        pulse = 0;

    uint8_t comm = current_direction_ccw
                   ? ccw_commutation[hall_state]
                   : cw_commutation[hall_state];

    unsigned int key = irq_lock();

    // Clear all channel enables first
    TIM1->CCER &= ~(TIM_CCER_CC1E  | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E  | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E  | TIM_CCER_CC3NE);

    switch (comm) {
        case 1: // W+ V-   PA10 high-side, PB14 low-side
            TIM1->CCR3 = (uint32_t)pulse;   // W+ active duty
            TIM1->CCR2 = 0;                 // V- full conduction
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC2NE;
            break;
        case 2: // V+ U-   PA9 high-side, PB13 low-side
            TIM1->CCR2 = (uint32_t)pulse;   // V+ active duty
            TIM1->CCR1 = 0;                 // U- full conduction
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1NE;
            break;
        case 3: // W+ U-   PA10 high-side, PB13 low-side
            TIM1->CCR3 = (uint32_t)pulse;   // W+ active duty
            TIM1->CCR1 = 0;                 // U- full conduction
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC1NE;
            break;
        case 4: // U+ W-   PA8 high-side, PB15 low-side
            TIM1->CCR1 = (uint32_t)pulse;   // U+ active duty
            TIM1->CCR3 = 0;                 // W- full conduction
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3NE;
            break;
        case 5: // U+ V-   PA8 high-side, PB14 low-side
            TIM1->CCR1 = (uint32_t)pulse;   // U+ active duty
            TIM1->CCR2 = 0;                 // V- full conduction
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2NE;
            break;
        case 6: // V+ W-   PA9 high-side, PB15 low-side
            TIM1->CCR2 = (uint32_t)pulse;   // V+ active duty
            TIM1->CCR3 = 0;                 // W- full conduction
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3NE;
            break;
        default:
            LOG_ERR("bldc: invalid comm %u for hall 0x%X", comm, hall_state);
            return;
    }

    LL_TIM_GenerateEvent_UPDATE(TIM1);

    irq_unlock(key);
}

/* ========================================================================= *
 * PWM OUTPUT — called by PID thread after softstart completes              *
 * ========================================================================= */
void bldc_set_pwm(int pulse)
{
    if (!motor_running) return;
    if (softstart_pulse < bldc_percent_to_pulse(15.0f)) return;

    // Once PID takes over, read current hall state and apply with new duty
    uint8_t state = (uint8_t)bldc_read_hall_state();
    if (state != 0 && state != 7) {
        bldc_set_commutation_with_duty(state, pulse);
    }

    // Also update softstart tracker so ISR doesn't reset duty downward
    if (pulse > softstart_pulse) {
        softstart_pulse = pulse;
    }
}

/* ========================================================================= *
 * LEGACY bldc_set_commutation — kept for any external callers              *
 * ========================================================================= */
void bldc_set_commutation(uint8_t step)
{
    bldc_set_commutation_with_duty(bldc_read_hall_state(), softstart_pulse);
    (void)step;
}

/* ========================================================================= *
 * CONVERSION / GETTERS / SETTERS                                            *
 * ========================================================================= */
int bldc_percent_to_pulse(float pct)
{
    int pulse = (int)(pct * 32.0f);    // 3200/100 = 32
    if (pulse > TIM1_ARR) pulse = TIM1_ARR;
    if (pulse < 0)        pulse = 0;
    return pulse;
}

uint32_t bldc_get_last_cycle_count(void)
{
    return (uint32_t)atomic_get(&last_cycle_atomic);
}

void bldc_set_direction(int ccw)
{
    current_direction_ccw = ccw;
}