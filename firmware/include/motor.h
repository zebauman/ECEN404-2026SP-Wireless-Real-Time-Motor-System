#ifndef MOTOR_H_
#define MOTOR_H_

#include <zephyr/types.h>
#include <stdbool.h>

#define RPM_MAX     6000
#define RPM_MIN     -6000

// MOTOR STATUS ARCHITECTURE

// LOWER NIBBLE: MUTUALLY EXCLUSIVE MOTOR STATES (BITS 0-3) - WHAT IS THE MOTOR DOING (THIS IS THE ACTUAL TRUE STATE OF THE MOTOR)
#define MOTOR_STATE_STOPPED			0x00	// 0000
#define MOTOR_STATE_RUNNING_SPEED	0x01	// 0001 - MOTOR IS ATTEMPTING TO REACH A CERTAIN RPM
#define MOTOR_STATE_RUNNING_POS		0x02	// 0010 - MOTOR IS ATTEMPTING TO REACH A CERTAIN POS
#define MOTOR_STATE_ESTOP			0x03	// 0011 (EMERGENCY STOP)
#define MOTOR_STATE_RESTART			0x04	// 0100 (SOFT START/CALIBRATING)
#define MOTOR_STATE_FAULT			0x05	// 0101 (HARDWARE FAILURE)

#define MOTOR_STATE_MASK			0x0F	// 0000 1111 (ISOLATE THE MOTOR STATE)

// UPPER NIBBLE: DIAGNOSTIC FLAGS (BITS 4-7) - CHECK IF THERE ARE ANY WARNINGS/ISSUES REGARDING THE MOTOR
#define MOTOR_FLAG_SYNC_BAD			0x10	// 0001 0000 
#define MOTOR_FLAG_OVERHEAT			0x20	// 0010 0000
#define MOTOR_FLAG_STALL			0x40	// 0100 0000
/** @todo ADD FLAG FOR MOTOR STALL -> ACTUAL RPM = 0, TARGET != 0; motor cannot move*/


#define MOTOR_FLAG_MASK				0xF0	// 1111 0000 - ISOLATE FLAGS



struct motor_stats{

	// SET POINTS - TARGETED VALUES FOR THE MOTOR
	uint8_t target_state;		// STATE THAT WE WANT THE MOTOR TO BE IN
	int32_t target_speed;		// REQUESTED RPM
	int32_t target_position;	// REQUESTED ANGLE

	// ACTUAL MOTOR STATS - FEEDBACK FROM THE HARDWARE, ACTUAL READINGS
	// STATUS OF MOTOR [FLAGS (4 bits) | STATE (4-bits)]
	uint8_t motor_status;

	// CURRENT SPEED OF MOTOR (RPM)
	int32_t current_speed;

	// CURRENT POSITION OF MOTOR (Degrees)
	int32_t current_position;

	// FILTERED SPEED OF MOTOR (RPM) 
	int32_t filtered_speed;
};


// PUBLIC API - MOTOR CONTROL

/** @brief Called from main() only and inits the motor stats and the mutex - only called once */
void motor_boot(void);

/** @brief Initialize motor hardware and clear stats (safe to call multiple times)*/
void motor_init(void);

// ACTUAL MOTOR STAT SETTERS
/** @brief SET THE MOTOR'S RPM (THIS IS THE ACTUAL & TRUE VALUE OF THE MOTOR) */
void motor_set_speed(int32_t rpm);

void motor_set_filtered_speed(int32_t rpm);


/** @brief SET THE MOTOR'S POSITION (THIS IS THE ACTUAL VALUE OF THE MOTOR) */
void motor_set_position(int32_t degrees);

void motor_set_sync_warning(bool active);
void motor_set_overheat_warning(bool active);
void motor_set_stall_warning(bool active);

/** @brief SET THE MOTOR INTO AN EMERGENCY STOP -> SET TARGET/ACTUAL STATE TO ESTOP AND TARGET SPEED TO 0 RPM*/
void motor_trigger_estop(void);

// TARGETED SETTERS
/** @brief SET THE DESIRED MOTOR RPM (STILL NEED TO SET THE TARGET STATE) */
void motor_set_target_speed(int32_t rpm);

/** @brief SET THE DESIRED MOTOR POSITION (STILL NEED TO SET THE TARGET STATE) */
void motor_set_target_position(int32_t degrees);




// PUBLIC API - GETTERS

// ACTUAL MOTOR STAT GETTERS
// STATUS
uint8_t motor_get_full_status(void);
bool motor_is_sync_bad(void);
bool motor_is_overheated(void);
bool motor_is_stall(void);

// TELEMETRY
int32_t motor_get_speed(void);
int32_t motor_get_filtered_speed(void);
int32_t motor_get_position(void);

// TARGETED MOTOR STAT GETTERS
uint8_t motor_get_target_state(void);
int32_t motor_get_target_speed(void);
int32_t motor_get_target_position(void);

#endif