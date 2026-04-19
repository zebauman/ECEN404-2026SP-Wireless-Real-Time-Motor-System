#include "bldc_driver.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <soc.h>
#include <stm32wbxx.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_bus.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bldc_driver, LOG_LEVEL_INF);

/* ========================================================================= *
 * HARDWARE CONSTANTS                                                         *
 * ========================================================================= */

/* STM32WB55 @ 32 MHz, PSC=0 → TIM1 = 32 MHz
 * ARR = 32 000 000 / 21 000 ≈ 1523 → 21 kHz PWM                           */
#define TIMER_CLOCK_HZ   32000000U
#define TARGET_PWM_FREQ  21000U
#define TIM1_ARR         (TIMER_CLOCK_HZ / TARGET_PWM_FREQ)   /* 1523       */
#define DEADTIME_TICKS   8U           /* DEADTIME_TICKS / 32 MHz = 1 µs                 */
#define POLE_PAIRS       4U            /* adjust to match your motor          */

/* ── Duty cycle constants ───────────────────────────────────────────────── *
 * BOOTSTRAP_DUTY: placed on the return-path (low-side) CCR during both     *
 * standby and active commutation. At 95% ARR the complementary output      *
 * fires for the remaining ~5%, keeping IHM08M1 bootstrap caps charged.     */
#define BOOTSTRAP_DUTY  ((TIM1_ARR * 95) / 100)   /* 1447 — ~5% on-time    */
#define SOFTSTART_DUTY  ((TIM1_ARR * 10) / 100)   /* 152 — 10%            */
#define DUTY_TARGET     ((TIM1_ARR * 60) / 100)   /* 913 — 60% cruise     */
#define DUTY_STEP       ((TIM1_ARR * 5) / 1000)  /* 7 — +0.5% / step   */

/* ── Hall debounce ──────────────────────────────────────────────────────── */
#define HALL_DEBOUNCE_US  20U

/* ── TIM2 free-running 1 MHz counter ────────────────────────────────────── *
 * PSC = (32 MHz / 1 MHz) − 1 = 31                                          */
#define TIM2_PRESCALER   31U
#define TIMEOUT_TICKS    1000000U      /* 1 s @ 1 tick = 1 µs               */

/* ── RPM: 8-sample circular buffer, TIM2 @ 1 MHz ────────────────────────── *
 * RPM = 60 000 000 / (POLE_PAIRS × sum_of_8_deltas_µs)
 * = RPM_CONSTANT_FILT / sum_delta                                       */
#define HISTORY_SIZE       8U
#define RPM_CONSTANT_FILT  (60000000UL / POLE_PAIRS)   /* 15 000 000        */

/* ========================================================================= *
 * COMMUTATION LOOKUP TABLES  (active-HIGH hall sensors)                     *
 * ========================================================================= *
 *
 * comm 1 = W+V−   comm 2 = V+U−   comm 3 = W+U−
 * comm 4 = U+W−   comm 5 = U+V−   comm 6 = V+W−
 *
 * Pin mapping:
 * PA8 /CH1  = U+    PB13/CH1N = U−
 * PA9 /CH2  = V+    PB14/CH2N = V−
 * PA10/CH3  = W+    PB15/CH3N = W−
 *
 * CW  (counter_clockwise=false):
 * hall→1:W+V−(1)  2:V+U−(2)  3:W+U−(3)  4:U+W−(4)  5:U+V−(5)  6:V+W−(6)
 *
 * CCW (counter_clockwise=true):
 * hall→1:V+W−(6)  2:U+V−(5)  3:U+W−(4)  4:W+U−(3)  5:V+U−(2)  6:W+V−(1)
 * ========================================================================= */
static const uint8_t cw_commutation[8]  = { 0, 4, 1, 3, 2, 5, 6, 0 }; 
static const uint8_t ccw_commutation[8] = { 0, 6, 5, 2, 3, 1, 4, 0 };

/* ========================================================================= *
 * GPIO                                                                       *
 * ========================================================================= */
static const struct gpio_dt_spec hall_u = GPIO_DT_SPEC_GET(DT_ALIAS(hall_u), gpios);
static const struct gpio_dt_spec hall_v = GPIO_DT_SPEC_GET(DT_ALIAS(hall_v), gpios);
static const struct gpio_dt_spec hall_w = GPIO_DT_SPEC_GET(DT_ALIAS(hall_w), gpios);

/* ========================================================================= *
 * STATE                                                                      *
 * ========================================================================= */
static struct gpio_callback hall_port_a_cb;
static struct gpio_callback hall_port_c_cb;

atomic_t  g_motor_speed_atomic    = ATOMIC_INIT(0);
static atomic_t last_cycle_atomic = ATOMIC_INIT(0);   /* for motor_control  */

static volatile int   current_direction_ccw = 0;
static volatile bool  motor_running         = false;
static volatile int   active_duty           = SOFTSTART_DUTY;

/* ── RPM circular buffer (written in ISR, read in control thread) ─────────  */
static volatile uint32_t delta_history[HISTORY_SIZE];
static volatile uint8_t  delta_idx      = 0;
static volatile uint32_t previous_ticks = 0;   /* TIM2->CNT at last edge    */

static void hall_isr_callback(const struct device *dev,
                               struct gpio_callback *cb, uint32_t pins);

/* ========================================================================= *
 * TIM2 FREE-RUNNING 1 MHz COUNTER                                           *
 * ========================================================================= */
static void setup_tim2_freerun(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    TIM2->PSC      = TIM2_PRESCALER;   /* 32 MHz / 32 = 1 MHz               */
    TIM2->ARR      = 0xFFFFFFFFU;      /* full 32-bit wrap-around            */
    TIM2->EGR     |= TIM_EGR_UG;       /* load prescaler immediately         */
    TIM2->CR1     |= TIM_CR1_CEN;
}

/* ========================================================================= *
 * INITIALIZATION                                                             *
 * ========================================================================= */
int bldc_driver_init(void)
{
    LOG_INF("Initializing BLDC driver (32 MHz / 21 kHz / ~1 µs dead-time)...");

    if (!gpio_is_ready_dt(&hall_u) ||
        !gpio_is_ready_dt(&hall_v) ||
        !gpio_is_ready_dt(&hall_w)) {
        LOG_ERR("Hall GPIOs not ready");
        return -ENODEV;
    }

    /* Active-HIGH hall sensors — no inversion, no internal pull-up needed   */
    gpio_pin_configure_dt(&hall_u, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&hall_v, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&hall_w, GPIO_INPUT | GPIO_PULL_UP);
    
    int boot_state = bldc_read_hall_state();
    LOG_INF("Boot hall state: 0x%X  %s", boot_state,
            (boot_state == 0 || boot_state == 7)
            ? "*** INVALID — rotate shaft slightly ***" : "OK");

    /* ── TIM2: 1 MHz free-running for RPM timing & stall timeout ─────────── */
    setup_tim2_freerun();
    previous_ticks = TIM2->CNT;

    /* ── TIM1: 21 kHz complementary PWM ─────────────────────────────────── */
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

    /* Disabled outputs driven LOW — prevents floating gate driver inputs     */
    LL_TIM_SetOffStates(TIM1, LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);
    LL_TIM_OC_SetDeadTime(TIM1, DEADTIME_TICKS);

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_GenerateEvent_UPDATE(TIM1);

    /* Bootstrap: charge IHM08M1 caps before first start                     */
    bldc_set_bootstrap();

    atomic_set(&last_cycle_atomic, (atomic_val_t)k_cycle_get_32());

    /* ── Hall interrupts (Grouped by Port for efficiency) ───────────────── */
    gpio_pin_interrupt_configure_dt(&hall_u, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_v, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_w, GPIO_INT_EDGE_BOTH);

    gpio_init_callback(&hall_port_c_cb, hall_isr_callback, BIT(hall_u.pin));
    gpio_init_callback(&hall_port_a_cb, hall_isr_callback, BIT(hall_v.pin) | BIT(hall_w.pin));

    gpio_add_callback(hall_u.port, &hall_port_c_cb);
    gpio_add_callback(hall_v.port, &hall_port_a_cb); // Covers both V and W

    return 0;
}

/* ========================================================================= *
 * BOOTSTRAP STATE (motor stopped)                                           *
 * ========================================================================= *
 * All three complementary outputs held at BOOTSTRAP_DUTY (95% CCR).        *
 * The CHxN pin fires for the remaining ~5%, charging bootstrap caps.        *
 * High-side outputs are fully disabled.                                     */
void bldc_set_bootstrap(void)
{
    motor_running = false;
    active_duty   = SOFTSTART_DUTY;

    TIM1->CCER &= ~(TIM_CCER_CC1E  | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E  | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E  | TIM_CCER_CC3NE);

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    TIM1->CCER |= TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
    LL_TIM_GenerateEvent_UPDATE(TIM1);

    LOG_INF("Bootstrap: low-sides active (~5%%), high-sides off");
}

/* ========================================================================= *
 * MOTOR START                                                                *
 * ========================================================================= */
void bldc_set_running(void)
{
    active_duty   = SOFTSTART_DUTY;
    motor_running = true;

    uint8_t state = (uint8_t)bldc_read_hall_state();
    if (state != 0 && state != 7) {
        bldc_set_commutation_with_duty(state, active_duty);
    }
}

/* ========================================================================= *
 * HALL SENSOR ISR                                                            *
 * ========================================================================= */
static void hall_isr_callback(const struct device *dev,
                               struct gpio_callback *cb, uint32_t pins)
{
    uint32_t now   = TIM2->CNT;
    uint32_t dt_us = now - previous_ticks;

    /* REJECT: Noise filter */
    if (dt_us < HALL_DEBOUNCE_US) return;

    /* REJECT: Physically impossible speed (e.g., > 5000 RPM) */
    if (dt_us < (RPM_CONSTANT_FILT / 5000)) return;


    /* ── Push delta to circular buffer ──────────────────────────────────── */
    delta_history[delta_idx] = dt_us;
    delta_idx = (delta_idx + 1) % HISTORY_SIZE;
    previous_ticks = now;

    /* ── Update kernel-cycle timestamp used by motor_control timeout ──────  */
    atomic_set(&last_cycle_atomic, (atomic_val_t)k_cycle_get_32());

    uint8_t raw_step = (uint8_t)bldc_read_hall_state();
    if (raw_step == 0 || raw_step == 7) return;   /* invalid sensor state   */

    if (!motor_running) {
        atomic_set(&g_motor_speed_atomic, 0);
        return;
    }

    /* ── Apply commutation at current active duty ────────────────────────── */
    bldc_set_commutation_with_duty(raw_step, active_duty);

    /* ── RPM: average the most recent inter-edge periods ─────────────────── */
    uint32_t sum = 0;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        sum += delta_history[i];
    }
    int32_t rpm = (sum > 0) ? (int32_t)(RPM_CONSTANT_FILT / sum) : 0;
    atomic_set(&g_motor_speed_atomic,
               (atomic_val_t)(current_direction_ccw ? -rpm : rpm));
}

/* ========================================================================= *
 * HALL STATE READ                                                            *
 * ========================================================================= */
int bldc_read_hall_state(void)
{
    uint8_t hu = gpio_pin_get_dt(&hall_u) ? 1 : 0;
    uint8_t hv = gpio_pin_get_dt(&hall_v) ? 1 : 0;
    uint8_t hw = gpio_pin_get_dt(&hall_w) ? 1 : 0;
    
    return (hu << 2) | (hv << 1) | hw;}

/* ========================================================================= *
 * COMMUTATION WITH DUTY                                                      *
 * ========================================================================= *
 * High-side CCR  = pulse          → PWMs at requested duty                 *
 * Low-side CCR   = BOOTSTRAP_DUTY → complementary fires ~5%, keeping       *
 * IHM08M1 bootstrap caps topped up        *
 * during active switching.                */
void bldc_set_commutation_with_duty(uint8_t hall_state, uint32_t pulse) {
    uint32_t key = irq_lock();

    // 1. Enforce 6% - 95% Hardware Safety Limits
    uint32_t min_pulse = (TIM1_ARR * 6) / 100;
    uint32_t max_pulse = (TIM1_ARR * 95) / 100;

    if (pulse < min_pulse) pulse = min_pulse;
    if (pulse > max_pulse) pulse = max_pulse;

    // 2. Map Hall state to 6-step sequence
    uint8_t comm = current_direction_ccw ? ccw_commutation[hall_state & 0x07] 
                                         : cw_commutation[hall_state & 0x07];

    // 3. Reset all High-sides to 0 before switching (Safety)
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    /* Ensure all CCRs are 0 initially to avoid shorts */
TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;

switch (comm) {
    case 1: /* U+ V− */
        TIM1->CCR1 = pulse;     // U High (PWM)
        TIM1->CCR2 = max_pulse; // V Low (95% ON)
        TIM1->CCER = (TIM_CCER_CC1E | TIM_CCER_CC2NE);
        break;
    case 2: /* U+ W− */
        TIM1->CCR1 = pulse;     // U High
        TIM1->CCR3 = max_pulse; // W Low
        TIM1->CCER = (TIM_CCER_CC1E | TIM_CCER_CC3NE);
        break;
    case 3: /* V+ W− */
        TIM1->CCR2 = pulse;     // V High
        TIM1->CCR3 = max_pulse; // W Low
        TIM1->CCER = (TIM_CCER_CC2E | TIM_CCER_CC3NE);
        break;
    case 4: /* V+ U− */
        TIM1->CCR2 = pulse;     // V High
        TIM1->CCR1 = max_pulse; // U Low
        TIM1->CCER = (TIM_CCER_CC2E | TIM_CCER_CC1NE);
        break;
    case 5: /* W+ U− */
        TIM1->CCR3 = pulse;     // W High
        TIM1->CCR1 = max_pulse; // U Low
        TIM1->CCER = (TIM_CCER_CC3E | TIM_CCER_CC1NE);
        break;
    case 6: /* W+ V− */
        TIM1->CCR3 = pulse;     // W High
        TIM1->CCR2 = max_pulse; // V Low
        TIM1->CCER = (TIM_CCER_CC3E | TIM_CCER_CC2NE);
        break;
    default:
        TIM1->CCER = 0; 
        break;
}

    LL_TIM_GenerateEvent_UPDATE(TIM1);
    irq_unlock(key);
}

/* ========================================================================= *
 * bldc_set_pwm — update running duty from the control thread               *
 * ========================================================================= */
void bldc_set_pwm(int pulse)
{
    if (!motor_running) return;

    active_duty = pulse;

    uint8_t state = (uint8_t)bldc_read_hall_state();
    if (state != 0 && state != 7) {
        bldc_set_commutation_with_duty(state, pulse);
    }
}

/* ========================================================================= *
 * bldc_softstart_step — call from control thread every STEP_DELAY_MS       *
 * Advances active_duty by DUTY_STEP toward DUTY_TARGET.                    *
 * Returns true when the ramp is complete.                                   *
 * ========================================================================= */
bool bldc_softstart_step(void)
{
    if (!motor_running)             return false;
    if (active_duty >= DUTY_TARGET) return true;

    active_duty += DUTY_STEP;
    if (active_duty > DUTY_TARGET) active_duty = DUTY_TARGET;

    uint8_t state = (uint8_t)bldc_read_hall_state();
    if (state != 0 && state != 7) {
        bldc_set_commutation_with_duty(state, active_duty);
    }
    return (active_duty >= DUTY_TARGET);
}

/* ========================================================================= *
 * bldc_is_hall_timeout — true if no hall edge received for > 1 s           *
 * ========================================================================= */
bool bldc_is_hall_timeout(void)
{
    return (TIM2->CNT - previous_ticks) > TIMEOUT_TICKS;
}

/* ========================================================================= *
 * bldc_get_duty_percent — current active duty as 0–100                     *
 * ========================================================================= */
uint32_t bldc_get_duty_percent(void)
{
    return ((uint32_t)active_duty * 100U) / TIM1_ARR;
}

/* ========================================================================= *
 * LEGACY / CONVERSION HELPERS                                               *
 * ========================================================================= */
void bldc_set_commutation(uint8_t step)
{
    bldc_set_commutation_with_duty((uint8_t)bldc_read_hall_state(), active_duty);
    (void)step;
}

int bldc_percent_to_pulse(float pct)
{
    int pulse = (int)(pct * (float)(TIM1_ARR) / 100.0f);
    if (pulse > (int)TIM1_ARR) pulse = (int)TIM1_ARR;
    if (pulse < 0)             pulse = 0;
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