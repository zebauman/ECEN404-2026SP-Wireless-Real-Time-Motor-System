#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/**
 * @brief Initializes PWM, ADC, PID, and starts the motor threads
 * @return 0 on success, negative error code otherwise
 */
int motor_control_init(void);

void motor_control_set_kp(float kp);
void motor_control_set_ki(float ki);
void motor_control_set_ilimit(float iLimit);

#endif