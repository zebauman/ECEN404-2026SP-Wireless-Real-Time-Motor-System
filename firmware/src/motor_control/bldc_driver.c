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

// STM32WB55 @ 64MHz (HSE 32MHz → PLL ×2), APB2 prescaler = 1
// TIM1 clock = PCLK2 = 64 MHz  (×2 multiplier only activates when APB prescaler > 1)
// ARR = 3200 → PWM freq = 64,000,000 / 3200 = 20 kHz
#define TIM1_ARR        3200

// Dead-time: 50 ticks × (1 / 64MHz) = 781 ns
// Sufficient for most bootstrap gate drivers. Increase if your driver's
// input capacitance or propagation delay needs a longer hold-off.
#define DEADTIME_TICKS  50      // 781ns at 64MHz

#define POLE_PAIRS      8
// Steps per mechanical revolution = POLE_PAIRS * 6 hall states = 48
// RPM = (cycles_per_sec / dt_cycles) * (60 / 48)
//     = (cycles_per_sec * 5) / (dt_cycles * 4)
// Max RPM before 50us debounce drops valid edges:
//   60 / (50e-6 * POLE_PAIRS * 6) = ~25,000 RPM
#define HALL_DEBOUNCE_US  50


/* ========================================================================= *
 * GPIO DEFINITIONS (From DeviceTree Aliases)                                *
 * ========================================================================= */
static const struct gpio_dt_spec hall_u = GPIO_DT_SPEC_GET(DT_ALIAS(hall_u), gpios);
static const struct gpio_dt_spec hall_v = GPIO_DT_SPEC_GET(DT_ALIAS(hall_v), gpios);
static const struct gpio_dt_spec hall_w = GPIO_DT_SPEC_GET(DT_ALIAS(hall_w), gpios);

/* ========================================================================= *
 * INTERRUPT STATE                                                           *
 * ========================================================================= */
static struct gpio_callback hall_u_cb;
static struct gpio_callback hall_v_cb;
static struct gpio_callback hall_w_cb;

// ATOMIC ISR TO THREAD SPEED CHANNEL - cannot use mutex in isr for ZEPHYR so instead ISR writes raw RPM with atomic & motor_control.c externs so reads directly
atomic_t g_motor_speed_atomic = ATOMIC_INIT(0);

// written in ISR, read in PID thread — must not tear on 32-bit boundary
static atomic_t last_cycle_atomic = ATOMIC_INIT(0);

static volatile int current_direction_ccw = 0;

/* Forward declaration */
static void hall_isr_callback(const struct device *dev,
                               struct gpio_callback *cb, uint32_t pins);

/* ========================================================================= *
 * INITIALIZATION                                                            *
 * ========================================================================= */
int bldc_driver_init(void) {
    LOG_INF("Initializing BLDC Hardware Driver...");

    /* --- Hall Sensor GPIOs --- */
    if (!gpio_is_ready_dt(&hall_u) ||
        !gpio_is_ready_dt(&hall_v) ||
        !gpio_is_ready_dt(&hall_w)) {
        LOG_ERR("Hall sensor GPIOs not ready!");
        return -ENODEV;
    }

    // GPIO_PULL_UP: hall sensors idle high; adjust to GPIO_PULL_DOWN if your
    // hardware uses active-high open-drain sensors with external pull-downs.
    gpio_pin_configure_dt(&hall_u, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&hall_v, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&hall_w, GPIO_INPUT | GPIO_PULL_UP);

    /* --- TIM1: Advanced Motor Control Timer --- */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    LL_TIM_SetPrescaler(TIM1, 0);
    LL_TIM_SetAutoReload(TIM1, TIM1_ARR - 1);
    LL_TIM_EnableARRPreload(TIM1);  // Preload ARR so updates are glitch-free

    // PWM Mode 1 on all three channels
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);

    // Preload CCR so writes take effect at the next update event, not mid-cycle
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);

    // SET OSSI AND OSSR SO OUTPUTS ARE ACTIVELY DRIVEN LOW WHEN DISABLED ( WHEN CCER BITS ARE CLEARED or MOE=0), not Hi-Z
    // WITHOUT THIS FLOAT GATE DRIVER INPUTS CAN CAUSE SHOOT-THROUGH when the motor is stopped or during an ESTOP
    LL_TIM_SetOffStates(TIM1, LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);

    // Dead-time prevents high and low side from conducting simultaneously
    LL_TIM_OC_SetDeadTime(TIM1, DEADTIME_TICKS);

    // ZERO ALL OF THE CCRs before enabling OUTPUTS - NO PULSE until bldc_set_pwm()
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    // Start counter and enable main output (MOE)
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);

    // Force an update event to latch all shadow registers immediately
    LL_TIM_GenerateEvent_UPDATE(TIM1);

    // APPLY INITIAL COMMUTATION FROM CURRENT HALL STATE (w/ out this no phase will be energised until the first hall edge fires)
    uint8_t init_step = (uint8_t)bldc_read_hall_state();
    // HALL EFFECT SENSOR READINGS HAS BE READING ATLEAST TWO OF THE STATE
    // SO IT CANNOT BE ZERO (when no hall edges are being triggered)
    // or 7 (when all hall edges are being triggered) -> only two edges should be triggered
    if (init_step != 0 && init_step != 7){
        uint8_t step = current_direction_ccw ? (init_step + 8) : init_step;
        bldc_set_commutation(step);
        LOG_INF("Initial commutation: hall=0x%X step=%u", init_step, step);
    } else {
        LOG_WRN("Invalid hall state at boot (0x%X) — check sensor wiring", init_step);
    }
    
    // Seed last_cycle_atomic so the PID thread timeout doesn't fire before
    // the motor has had a chance to move (avoids false timeout at startup)
    atomic_set(&last_cycle_atomic, (atomic_val_t)k_cycle_get_32());

    /* --- Hall Sensor Interrupts --- */
    gpio_pin_interrupt_configure_dt(&hall_u, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_v, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_w, GPIO_INT_EDGE_BOTH);

    gpio_init_callback(&hall_u_cb, hall_isr_callback, BIT(hall_u.pin));
    gpio_init_callback(&hall_v_cb, hall_isr_callback, BIT(hall_v.pin));
    gpio_init_callback(&hall_w_cb, hall_isr_callback, BIT(hall_w.pin));

    gpio_add_callback_dt(&hall_u, &hall_u_cb);
    gpio_add_callback_dt(&hall_v, &hall_v_cb);
    gpio_add_callback_dt(&hall_w, &hall_w_cb);

    LOG_INF("BLDC driver ready — PWM %u Hz, dead-time %u ns",
            CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / TIM1_ARR,
            (DEADTIME_TICKS * 1000U) / (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000U));

    return 0;
}

/* ========================================================================= *
 * HALL SENSOR ISR                                                           *
 * ========================================================================= */
static void hall_isr_callback(const struct device *dev,
                               struct gpio_callback *cb, uint32_t pins) {
    uint32_t now        = k_cycle_get_32();
    uint32_t past       = (uint32_t)atomic_get(&last_cycle_atomic);
    uint32_t dt_cycles  = now - past;   // Safe: unsigned wraps correctly

    uint32_t cycles_per_sec = sys_clock_hw_cycles_per_sec();

    // Hardware debounce: ignore edges faster than HALL_DEBOUNCE_US
    // This filters PCB noise without masking valid transitions up to ~25k RPM
    uint32_t dt_us = (uint32_t)(((uint64_t)dt_cycles * 1000000U) / cycles_per_sec);
    if (dt_us < HALL_DEBOUNCE_US) {
        return;
    }

    atomic_set(&last_cycle_atomic, (atomic_val_t)now);

    // Read and validate hall state — 0 (000) and 7 (111) are illegal
    uint8_t raw_step = (uint8_t)bldc_read_hall_state();
    if (raw_step == 0 || raw_step == 7) {
        // LOG_WRN("Invalid hall state: %d — possible sensor fault or wiring issue", raw_step);
        return;
    }

    // Apply direction offset: CCW commutation table lives at cases 9–14 (+8)
    uint8_t step = current_direction_ccw ? (raw_step + 8) : raw_step;
    bldc_set_commutation(step);

    // RPM = (cycles_per_sec / dt_cycles) * (60 / steps_per_rev)
    // steps_per_rev = POLE_PAIRS * 6 = 48  =>  60/48 = 5/4
    // Use 64-bit to prevent overflow on faster clocks (e.g. 72MHz * 5 = 360M > 2^32)
    int32_t mech_rpm = (int32_t)(((uint64_t)cycles_per_sec * 5ULL)
                                 / ((uint64_t)dt_cycles   * 4ULL));

    atomic_set(&g_motor_speed_atomic, (atomic_val_t)(current_direction_ccw ? -mech_rpm : mech_rpm));
}

/* ========================================================================= *
 * SENSOR READS                                                              *
 * ========================================================================= */
int bldc_read_hall_state(void) {
    int u = gpio_pin_get_dt(&hall_u);
    int v = gpio_pin_get_dt(&hall_v);
    int w = gpio_pin_get_dt(&hall_w);
    // Pack into 3-bit value: [U=bit2, V=bit1, W=bit0]
    return (u << 2) | (v << 1) | w;
}

/* ========================================================================= *
 * MOTOR ACTUATION                                                           *
 * ========================================================================= */
void bldc_set_pwm(int pulse) {
    if (pulse > TIM1_ARR) pulse = TIM1_ARR;
    if (pulse < 0)        pulse = 0;

    // Direct CCR writes; values are double-buffered via preload — glitch-free
    TIM1->CCR1 = (uint32_t)pulse;
    TIM1->CCR2 = (uint32_t)pulse;
    TIM1->CCR3 = (uint32_t)pulse;
}

void bldc_set_commutation(uint8_t step) {
    // Disable all high-side (CCxE) and low-side (CCxNE) outputs first
    TIM1->CCER &= ~(TIM_CCER_CC1E  | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E  | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E  | TIM_CCER_CC3NE);

    // Enable the correct phase pair for this hall step.
    // Hardware dead-time (DEADTIME_TICKS) prevents shoot-through automatically.
    switch (step) {
        /* --- CLOCKWISE (raw hall state 1–6) --- */
        case 1:  TIM1->CCER |= TIM_CCER_CC3E  | TIM_CCER_CC2NE; break; // W+ V-
        case 5:  TIM1->CCER |= TIM_CCER_CC1E  | TIM_CCER_CC2NE; break; // U+ V-
        case 4:  TIM1->CCER |= TIM_CCER_CC1E  | TIM_CCER_CC3NE; break; // U+ W-
        case 6:  TIM1->CCER |= TIM_CCER_CC2E  | TIM_CCER_CC3NE; break; // V+ W-
        case 2:  TIM1->CCER |= TIM_CCER_CC2E  | TIM_CCER_CC1NE; break; // V+ U-
        case 3:  TIM1->CCER |= TIM_CCER_CC3E  | TIM_CCER_CC1NE; break; // W+ U-

        /* --- COUNTER-CLOCKWISE (raw + 8 offset, cases 9–14) --- */
        case 9:  TIM1->CCER |= TIM_CCER_CC1E  | TIM_CCER_CC2NE; break; // U+ V-
        case 13: TIM1->CCER |= TIM_CCER_CC3E  | TIM_CCER_CC2NE; break; // W+ V-
        case 12: TIM1->CCER |= TIM_CCER_CC3E  | TIM_CCER_CC1NE; break; // W+ U-
        case 14: TIM1->CCER |= TIM_CCER_CC2E  | TIM_CCER_CC1NE; break; // V+ U-
        case 10: TIM1->CCER |= TIM_CCER_CC2E  | TIM_CCER_CC3NE; break; // V+ W-
        case 11: TIM1->CCER |= TIM_CCER_CC1E  | TIM_CCER_CC3NE; break; // U+ W-

        default:
            // Should never reach here — caller must validate step before calling
            LOG_ERR("bldc: invalid commutation step %d", step);
            break;
    }
}

// DUTY CYCLE CONVERSION
int bldc_percent_to_pulse(float percent_duty_cycle) {
    // TIM1_ARR = 3200 (64MHz / 3200 = 20kHz)
    // Scale factor = TIM1_ARR / 100.0 = 32.0
    int pulse = (int)(percent_duty_cycle * 32.0f);

    if (pulse > TIM1_ARR) pulse = TIM1_ARR;
    if (pulse < 0)        pulse = 0;

    return pulse;
}

/* ========================================================================= *
 * GETTERS / SETTERS                                                         *
 * ========================================================================= */
uint32_t bldc_get_last_cycle_count(void) {
    return (uint32_t)atomic_get(&last_cycle_atomic);
}

void bldc_set_direction(int ccw) {
    current_direction_ccw = ccw;
}