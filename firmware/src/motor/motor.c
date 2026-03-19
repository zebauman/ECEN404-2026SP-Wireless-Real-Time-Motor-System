#include "motor.h"
#include <zephyr/kernel.h> // REQUIRED for k_mutex
#include <string.h>
#include <stdbool.h>

static struct motor_stats m_stats;
static struct k_mutex m_stats_lock;

/* PRIVATE HELPERS (ASSUME THAT THE CALLER ALREADY LOCKED THE MUTEX)*/

/** @brief SET OR CLEAR SPECIFIC DIAGONISTIC FLAGS */
static void _motor_set_flag_unlocked(uint8_t flag, bool active){
    if(active){
        m_stats.motor_status |= (flag & MOTOR_FLAG_MASK);
    } else{
        m_stats.motor_status &= ~(flag & MOTOR_FLAG_MASK);
    }
}

/** @brief Update the motor's internal state (keep flags) */
static void _motor_set_state(uint8_t new_state){
    m_stats.motor_status = (m_stats.motor_status & MOTOR_FLAG_MASK) | (new_state & MOTOR_STATE_MASK);
}

/** @brief SET THE TARGETED/DESIRED MOTOR STATE - ONLY THE LOWER NIBBLES (NO FLAGS)*/
static void _motor_set_target_state(uint8_t new_state){
    m_stats.target_state = new_state & MOTOR_STATE_MASK;
}

/* PUBLIC API */

void motor_boot(void){
    k_mutex_init(&m_stats_lock);
    motor_init();
}

void motor_init(void){
    k_mutex_lock(&m_stats_lock, K_FOREVER);

    memset(&m_stats, 0, sizeof(m_stats)); // WIPE ALL THE DATA TO ZERO (EVEN PRE-EXISTING DATA)
    _motor_set_state(MOTOR_STATE_STOPPED);

    _motor_set_flag_unlocked(MOTOR_FLAG_SYNC_BAD,  false);
    _motor_set_flag_unlocked(MOTOR_FLAG_OVERHEAT,  false);
    _motor_set_flag_unlocked(MOTOR_FLAG_STALL,     false);

    k_mutex_unlock(&m_stats_lock);
}

/* --- ACTUAL MOTOR STATUS SETTERS (CALLED BY MOTOR THREAD) --- */

void motor_set_speed(int32_t rpm){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    m_stats.current_speed = rpm;    // SHOULD BE CORRECT VALUE SINCE PASSED DIRECTLY FROM MOTOR LOGIC

    k_mutex_unlock(&m_stats_lock);
}

void motor_set_filtered_speed(int32_t rpm){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    m_stats.filtered_speed = rpm;
    k_mutex_unlock(&m_stats_lock);
}

void motor_set_position(int32_t degrees){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    m_stats.current_position = degrees; // SHOULD BE CORRECT VALUE SINCE PASSED DIRECTLY FROM MOTOR LOGIC
    _motor_set_state(MOTOR_STATE_RUNNING_POS);

    k_mutex_unlock(&m_stats_lock);
}

void motor_set_sync_warning(bool active){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    _motor_set_flag_unlocked(MOTOR_FLAG_SYNC_BAD, active);
    k_mutex_unlock(&m_stats_lock);
}

void motor_set_overheat_warning(bool active){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    _motor_set_flag_unlocked(MOTOR_FLAG_OVERHEAT, active);
    k_mutex_unlock(&m_stats_lock);
}

void motor_set_stall_warning(bool active){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    _motor_set_flag_unlocked(MOTOR_FLAG_STALL, active);
    k_mutex_unlock(&m_stats_lock);

}

void motor_trigger_estop(){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    _motor_set_state(MOTOR_STATE_ESTOP);
    _motor_set_target_state(MOTOR_STATE_ESTOP);
    m_stats.target_speed = 0;
    k_mutex_unlock(&m_stats_lock);
}


void motor_set_target_speed(int32_t rpm){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    if(rpm > RPM_MAX) rpm = RPM_MAX;
    if(rpm < RPM_MIN) rpm = RPM_MIN;

    m_stats.target_speed = rpm;

    if(rpm != 0){
        _motor_set_target_state(MOTOR_STATE_RUNNING_SPEED);
    } else{
        _motor_set_target_state(MOTOR_STATE_STOPPED);
    }

    k_mutex_unlock(&m_stats_lock);

}

void motor_set_target_position(int32_t degrees){
    k_mutex_lock(&m_stats_lock, K_FOREVER);

    m_stats.target_position = degrees % 360;
    _motor_set_target_state(MOTOR_STATE_RUNNING_POS);

    k_mutex_unlock(&m_stats_lock);
}


// GETTERS
uint8_t motor_get_full_status(void){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    uint8_t val = m_stats.motor_status;
    k_mutex_unlock(&m_stats_lock);
    return val;
}

bool motor_is_sync_bad(void){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    bool val = m_stats.motor_status & MOTOR_FLAG_SYNC_BAD;
    k_mutex_unlock(&m_stats_lock);
    return val;
}

bool motor_is_overheated(void){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    bool val = m_stats.motor_status & MOTOR_FLAG_OVERHEAT;
    k_mutex_unlock(&m_stats_lock);
    return val;
}

bool motor_is_stall(void){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    bool val = m_stats.motor_status & MOTOR_FLAG_STALL;
    k_mutex_unlock(&m_stats_lock);
    return val;
}


int32_t motor_get_speed(void){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    int32_t val = m_stats.current_speed;
    k_mutex_unlock(&m_stats_lock);
    return val;
}

int32_t motor_get_filtered_speed(void){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    int32_t val = m_stats.filtered_speed;
    k_mutex_unlock(&m_stats_lock);
    return val;
}


int32_t motor_get_position(void){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    int32_t val = m_stats.current_position;
    k_mutex_unlock(&m_stats_lock);
    return val;

}

uint8_t motor_get_target_state(void){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    uint8_t val = m_stats.target_state;
    k_mutex_unlock(&m_stats_lock);
    return val;
}

int32_t motor_get_target_speed(void){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    int32_t val = m_stats.target_speed;
    k_mutex_unlock(&m_stats_lock);
    return val;
}

int32_t motor_get_target_position(void){
    k_mutex_lock(&m_stats_lock, K_FOREVER);
    int32_t val = m_stats.target_position;
    k_mutex_unlock(&m_stats_lock);
    return val;
}