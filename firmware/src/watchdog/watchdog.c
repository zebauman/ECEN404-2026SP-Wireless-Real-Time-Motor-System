#include "watchdog.h"
#include <zephyr/logging/log.h>
#include "motor.h"

LOG_MODULE_REGISTER(watchdog, LOG_LEVEL_INF);

// WATCHDOG TIMEOUT 2 SECONDS
#define WATCHDOG_TIMEOUT_MS 2000

static struct k_work_delayable watchdog_work;  // WORKQUEUE THREAD -> THREAD THAT FIRES AFTER DELAY

// EMERGENCY STOP FUNCTION IF WATCHDOG EXPIRES -> WILL HALT MOTOR
static void watchdog_expired(struct k_work *work){
    LOG_ERR("Watchdog Timer Expired - Connection Lost - HALTING MOTOR.");

    motor_set_sync_warning(true);
    // motor_trigger_estop();
    
    LOG_INF("MOTOR HALTED");
}

// INIT WATCHDOG
void watchdog_init(void){
    k_work_init_delayable(&watchdog_work, watchdog_expired);
    LOG_INF("WATCHDOG INITIALIZED");
}

// RESET THE DELAYED THREAD
void watchdog_kick(void){
    k_work_reschedule(&watchdog_work, K_MSEC(WATCHDOG_TIMEOUT_MS));
}

void watchdog_stop(void){
    k_work_cancel_delayable(&watchdog_work);
    LOG_INF("WATCHDOG STOPPED");
}