#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/hwinfo.h>
#include <string.h>

#include "bluetooth.h"
#include "watchdog.h"
#include "motor.h"
#include "motor_control.h"

LOG_MODULE_REGISTER(bluetooth, LOG_LEVEL_INF);

/* ========================================================================= *
 * CONSTANTS                                                                 *
 * ========================================================================= */
#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* Manufacturer specific data: 2-byte company ID + 6-byte unique device ID */
#define MSD_LEN (2 + 6)

/* ========================================================================= *
 * MODULE STATE                                                              *
 * ========================================================================= */
static struct motor_app_ctx motor_ctx;

/* Skips the sync-slip check on the very first heartbeat after (re)connect,
 * preventing a false SYNC SLIP warning caused by the counter starting at an
 * arbitrary value on the phone side. */
static bool first_heartbeat = true;

static const struct bt_uuid_128 motor_srv_uuid      = BT_UUID_INIT_128(BT_UUID_MOTOR_SERVICE_VAL);
static const struct bt_uuid_128 motor_cmd_char_uuid = BT_UUID_INIT_128(BT_UUID_MOTOR_CMD_VAL);
static const struct bt_uuid_128 heartbeat_char_uuid = BT_UUID_INIT_128(BT_UUID_MOTOR_HEARTBEAT_VAL);
static const struct bt_uuid_128 motor_tel_char_uuid = BT_UUID_INIT_128(BT_UUID_MOTOR_TELEMETRY_VAL);

static uint8_t dev_id_le[6];
static uint8_t msd[MSD_LEN];

/* ========================================================================= *
 * TELEMETRY THREAD                                                          *
 * Pushes motor state to the connected BLE device at 10 Hz.                *
 * Only wakes Zephyr BT stack when a client is actually subscribed.        *
 * ========================================================================= */
static void telemetry_thread_fn(void *arg1, void *arg2, void *arg3)
{
    while (1) {
        if (motor_ctx.notification_enabled) {
            motor_notify_telemetry();
        }
        k_msleep(100);
    }
}

/* Stack sized for BLE notification path + logging overhead */
K_THREAD_DEFINE(telemetry_tid, 1536, telemetry_thread_fn,
                NULL, NULL, NULL, 7, 0, 0);

/* ========================================================================= *
 * ADVERTISING HELPERS                                                       *
 * ========================================================================= */
static void build_ids(void)
{
    int ret = hwinfo_get_device_id(dev_id_le, sizeof(dev_id_le));
    if (ret < 0) {
        LOG_ERR("hwinfo_get_device_id failed (err %d)", ret);
        memset(dev_id_le, 0, sizeof(dev_id_le));
    }
    sys_put_le16(MY_COMPANY_ID, &msd[0]);
    memcpy(&msd[2], dev_id_le, sizeof(dev_id_le));
}

/* ========================================================================= *
 * GATT WRITE CALLBACKS                                                      *
 * ========================================================================= */

/** Motor command characteristic write handler.
 *  Packet layout: [cmd: 1 byte][value: 4 bytes LE] = 5 bytes minimum.
 */
static ssize_t write_motor(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len,
                            uint16_t offset, uint8_t flags)
{
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len < 5) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *data = (const uint8_t *)buf;
    uint8_t  cmd = data[0];
    int32_t  val = (int32_t)sys_get_le32(&data[1]);

    switch ((motor_cmd_t)cmd) {
        case MOTOR_MODE_SPEED:
            motor_set_target_speed(val);
            break;
        case MOTOR_MODE_POSITION:
            motor_set_target_position(val);
            break;
        case MOTOR_MODE_INIT:
            motor_init();
            break;
        case MOTOR_MODE_OFF:
            motor_set_target_speed(0);
            break;
        case MOTOR_MODE_SET_KP:
            float new_kp;
            memcpy(&new_kp, &val, sizeof(float));
            motor_control_set_kp(new_kp);
            break;
        case MOTOR_MODE_SET_KI:
            float new_ki;
            memcpy(&new_ki, &val, sizeof(float));
            motor_control_set_ki(new_ki);
            break;
        case MOTOR_MODE_SET_ILIMIT:
            float new_iLimit;
            memcpy(&new_iLimit, &val, sizeof(float));
            motor_control_set_ilimit(new_iLimit);
            break;
        default:
            LOG_WRN("Unknown motor command: 0x%02X", cmd);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    return (ssize_t)len;
}

/** Heartbeat characteristic write handler.
 *  Packet layout: [counter: 1 byte].
 *  The phone increments the counter on every write. A diff of 1 = healthy,
 *  diff > 1 = packets were skipped (BLE congestion or app backgrounded).
 */
static ssize_t write_heartbeat(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len,
                                uint16_t offset, uint8_t flags)
{
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len < 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *data = (const uint8_t *)buf;
    uint8_t new_val = data[0];

    // Skip the sync check on the very first packet — the phone's counter can
    // start at any value so diff against our initialised 0 is meaningless.
    if (first_heartbeat) {
        first_heartbeat = false;
        motor_ctx.heartbeat_val = new_val;
        watchdog_kick();
        return (ssize_t)len;
    }

    uint8_t diff = new_val - motor_ctx.heartbeat_val; // Wraps correctly (uint8)

    if (diff == 0) {
        // Identical value — stale duplicate, do not kick watchdog
        LOG_WRN("Stale heartbeat (val=%u)", new_val);
        return (ssize_t)len;
    } else if (diff > 1) {
        // Gap detected — phone app may have been backgrounded or congested
        motor_set_sync_warning(true);
        LOG_WRN("BLE sync slip: expected +1, got +%u", diff);
    } else {
        // diff == 1: perfect
        motor_set_sync_warning(false);
    }

    motor_ctx.heartbeat_val = new_val;
    watchdog_kick();

    return (ssize_t)len;
}

/* ========================================================================= *
 * CCC CALLBACK                                                              *
 * ========================================================================= */
static void motor_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    motor_ctx.notification_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Telemetry notifications %s",
            motor_ctx.notification_enabled ? "enabled" : "disabled");
}


/* ========================================================================= *
 * GATT SERVICE DEFINITION                                                   *
 *                                                                           *
 * Attribute table layout (attrs[] index):                                  *
 *  [0] Primary service declaration                                         *
 *  [1] CMD characteristic declaration                                      *
 *  [2] CMD characteristic value          <- write_motor()                  *
 *  [3] Heartbeat characteristic declaration                                *
 *  [4] Heartbeat characteristic value    <- write_heartbeat()              *
 *  [5] Telemetry characteristic declaration                                *
 *  [6] Telemetry characteristic value    <- bt_gatt_notify target          *
 *  [7] Telemetry CCC descriptor                                            *
 * ========================================================================= */
BT_GATT_SERVICE_DEFINE(motor_svc,
    BT_GATT_PRIMARY_SERVICE(&motor_srv_uuid),

    BT_GATT_CHARACTERISTIC(&motor_cmd_char_uuid.uuid,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, write_motor, NULL),

    BT_GATT_CHARACTERISTIC(&heartbeat_char_uuid.uuid,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, write_heartbeat, NULL),

    BT_GATT_CHARACTERISTIC(&motor_tel_char_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),

    BT_GATT_CCC(motor_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);


/* ========================================================================= *
 * TELEMETRY NOTIFICATION                                                    *
 * Packet layout: [status: 1B][filtered_speed: 4B LE][position: 4B LE]    *
 * ========================================================================= */
static inline void pack_telemetry(uint8_t out[9])
{
    out[0] = motor_get_full_status();
    sys_put_le32((uint32_t)motor_get_filtered_speed(), &out[1]);
    sys_put_le32((uint32_t)motor_get_position(),       &out[5]);
}

void motor_notify_telemetry(void)
{
    if (!motor_ctx.notification_enabled) {
        return;
    }

    uint8_t telemetry_data[9];
    pack_telemetry(telemetry_data);

    /* attrs[6] = telemetry characteristic value — see table above */
    int err = bt_gatt_notify(NULL, &motor_svc.attrs[6],
                             telemetry_data, sizeof(telemetry_data));
    if (err && err != -ENOTCONN) {
        // -ENOTCONN is expected when the link drops between the notification_enabled
        // check and the notify call; anything else is worth logging.
        LOG_WRN("Telemetry notify failed (err %d)", err);
    }
}

/* ========================================================================= *
 * BLUETOOTH INIT & ADVERTISING                                              *
 * ========================================================================= */
void bt_ready(int err)
{
    if (err) {
        LOG_ERR("bt_enable failed (err %d)", err);
        return;
    }

    motor_ctx.heartbeat_val      = 0;
    motor_ctx.notification_enabled = false;

    LOG_INF("Bluetooth initialised");

    build_ids();

    /* Advertising interval: 0x20–0x40 = 20ms–40ms */
    struct bt_le_adv_param adv_param = {
        .options      = BT_LE_ADV_OPT_CONNECTABLE,
        .interval_min = 0x20,
        .interval_max = 0x40,
        .peer         = NULL,
    };

    /* AD payload: flags + 128-bit service UUID + manufacturer data = ~29 bytes */
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS,
                      BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                      BT_UUID_MOTOR_SERVICE_VAL),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, msd, sizeof(msd)),
    };

    /* Scan response: full device name */
    const struct bt_data sd[] = {
        BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    };

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Advertising started as \"%s\"", DEVICE_NAME);
}

/* ========================================================================= *
 * CONNECTION CALLBACKS                                                      *
 * ========================================================================= */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    LOG_INF("BLE connected");
    first_heartbeat = true;   // Reset sync check for new connection
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("BLE disconnected (reason %u)", reason);
    motor_ctx.notification_enabled = false;
    first_heartbeat = true;   // Reset for next connection
    watchdog_stop();
    motor_set_target_speed(0);
}

struct bt_conn_cb conn_callbacks = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* ========================================================================= *
 * PUBLIC GETTERS                                                            *
 * ========================================================================= */
uint8_t bt_get_heartbeat(void)
{
    return motor_ctx.heartbeat_val;
}

bool bt_is_notify_enabled(void)
{
    return motor_ctx.notification_enabled;
}