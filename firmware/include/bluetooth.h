#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdbool.h>
#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

/* ========================================================================= *
 * COMPANY & SERVICE IDENTIFIERS                                             *
 * ========================================================================= */

#define MY_COMPANY_ID 0x706D   // Manufacturer-specific data company ID

/* 128-bit service and characteristic UUIDs */
#define BT_UUID_MOTOR_SERVICE_VAL \
    BT_UUID_128_ENCODE(0xc52081ba, 0xe90f, 0x40e4, 0xa99f, 0xccaa4fd11c15)

#define BT_UUID_MOTOR_CMD_VAL \
    BT_UUID_128_ENCODE(0xd10b46cd, 0x412a, 0x4d15, 0xa7bb, 0x092a329eed46)

#define BT_UUID_MOTOR_TELEMETRY_VAL \
    BT_UUID_128_ENCODE(0x17da15e5, 0x05b1, 0x42df, 0x8d9d, 0xd7645d6d9293)

#define BT_UUID_MOTOR_HEARTBEAT_VAL \
    BT_UUID_128_ENCODE(0x2215d558, 0xc569, 0x4bd1, 0x8947, 0xb4fd5f9432a0)

/* ========================================================================= *
 * BLE COMMAND OPCODES                                                       *
 * Sent as the first byte of a write to the CMD characteristic.             *
 * ========================================================================= */
typedef enum {
    MOTOR_MODE_OFF      = 0x00,
    MOTOR_MODE_INIT     = 0x01,
    MOTOR_MODE_SPEED    = 0x02,
    MOTOR_MODE_POSITION = 0x03,
} motor_cmd_t;

/* ========================================================================= *
 * APPLICATION CONTEXT                                                       *
 * ========================================================================= */

/** Internal BLE state. Do not access directly outside bluetooth.c. */
struct motor_app_ctx {
    volatile bool    notification_enabled;   // True once client subscribes to telemetry
    uint8_t          heartbeat_val;          // Last heartbeat counter value from phone
};

/* ========================================================================= *
 * PUBLIC API                                                                *
 * ========================================================================= */

/** @brief Bluetooth ready callback — pass to bt_enable().
 *  Initialises context, starts advertising, and logs status.
 */
void bt_ready(int err);

/** @brief Send a telemetry notification to the connected device.
 *  No-op if notifications are not enabled.
 */
void motor_notify_telemetry(void);

/** @brief Return the last heartbeat counter received from the phone. */
uint8_t bt_get_heartbeat(void);

/** @brief Return true if the client has subscribed to telemetry notifications. */
bool bt_is_notify_enabled(void);

/** Connection callbacks — must be registered in main.c via
 *  bt_conn_cb_register(&conn_callbacks).
 */
extern struct bt_conn_cb conn_callbacks;

#endif /* BLUETOOTH_H */