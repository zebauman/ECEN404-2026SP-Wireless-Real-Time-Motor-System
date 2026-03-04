#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/hwinfo.h>
#include <stdio.h>

#include "bluetooth.h"
#include "watchdog.h"
#include "motor.h"
#include "bldc_driver.h"
#include "motor_control.h"

#ifdef CONFIG_MOTOR_SIM
#include "motor_sim.h"
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);


int main(void)
{
    LOG_INF("Starting BLDC Hardware Motor Control Application");    

    // Initialize the Motor Data Structures (Safe API Vault)
    motor_boot(); 

    // Initialize Real Hardware (PWM & ADC peripherals)
    int hw_err = bldc_driver_init();
    if (hw_err != 0) {
        LOG_ERR("Hardware init failed! Check Device Tree. (err %d)", hw_err);
        // Depending on your safety requirements, you might want to halt here.
        return 0; 
    }

    // Start the Real Motor Control Threads (Commutation, Hall Monitor, PID)
    motor_control_init();

    #ifdef CONFIG_MOTOR_SIM
        motor_sim_init();
    #endif

    // Initialize Bluetooth
    int err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }

    // Register Callbacks & Start Watchdog
    bt_conn_cb_register(&conn_callbacks);
    watchdog_init();

    LOG_INF("System Boot Complete. Waiting for Bluetooth connection...");

    return 0;
}