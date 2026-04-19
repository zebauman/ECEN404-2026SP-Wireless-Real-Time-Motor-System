#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/atomic.h>

extern atomic_t g_motor_speed_atomic;

/* ========================================================================= *
 * BLDC DRIVER — Public API                                                  *
 * ========================================================================= */

/** @brief Initialise GPIOs, TIM1, and hall sensor ISRs. Call once at boot.
 *  @return 0 on success, negative errno on failure.
 */
int bldc_driver_init(void);

/** @brief Set PWM pulse width directly (0 – TIM1_ARR).
 *  @param pulse Raw timer compare value; clamped internally.
 */
void bldc_set_pwm(int pulse);

/** @brief Apply the correct phase switching pattern for the given hall step.
 *  @param step  Hall state (1–6 CW, 9–14 CCW). Invalid states are ignored.
 */
void bldc_set_commutation(uint8_t step);

/** @brief Read the three hall sensor GPIOs and return a 3-bit state (0–7).
 *  @return Combined state: (U<<2)|(V<<1)|W. Caller must reject 0 and 7.
 */
int bldc_read_hall_state(void);

/** @brief Convert a PWM duty-cycle percentage to a raw timer pulse value.
 *  @param percent_duty_cycle  0.0 – 100.0 %
 *  @return Raw pulse value clamped to [0, TIM1_ARR].
 */
int bldc_percent_to_pulse(float percent_duty_cycle);

/** @brief Return the CPU cycle count captured at the last valid hall edge.
 *  @note  Returns an atomic snapshot; safe to call from any thread.
 */
uint32_t bldc_get_last_cycle_count(void);

/** @brief Set motor rotation direction.
 *  @param ccw  0 = clockwise, 1 = counter-clockwise.
 */
void bldc_set_direction(int ccw);

void bldc_set_bootstrap(void);

void bldc_set_running(void);

void bldc_set_commutation_with_duty(uint8_t hall_state, uint32_t pulse);
#endif /* BLDC_DRIVER_H */