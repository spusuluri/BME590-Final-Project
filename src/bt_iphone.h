#ifndef BT_IPHONE_H
#define BT_IPHONE_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/services/bas.h>

/* UUID of the Remote Service */
// Blur Project ID: 007 (3rd entry)
// Blur BLE MFG ID = 0x03DF (4th entry)
#define BT_UUID_REMOTE_SERV_VAL \
	    BT_UUID_128_ENCODE(0xe2ea0000, 0xe29b, 0x0007, 0x03DF, 0xc8543272fc23)

/* UUID of the Data Characteristic */
#define BT_UUID_REMOTE_DATA_CHRC_VAL \
	    BT_UUID_128_ENCODE(0xe2ea0001, 0xe29b, 0x0007, 0x03DF, 0xc8543272fc23)

/* UUID of the Message Characteristic */
#define BT_UUID_REMOTE_MESSAGE_CHRC_VAL \
	    BT_UUID_128_ENCODE(0xe2ea0002, 0xe29b, 0x0007, 0x03DF, 0xc8543272fc23)

#define BT_UUID_REMOTE_SERVICE 			BT_UUID_DECLARE_128(BT_UUID_REMOTE_SERV_VAL)
#define BT_UUID_REMOTE_DATA_CHRC 		BT_UUID_DECLARE_128(BT_UUID_REMOTE_DATA_CHRC_VAL)
#define BT_UUID_REMOTE_MESSAGE_CHRC 	BT_UUID_DECLARE_128(BT_UUID_REMOTE_MESSAGE_CHRC_VAL)

enum bt_data_notifications_enabled {
	BT_DATA_NOTIFICATIONS_ENABLED,
	BT_DATA_NOTIFICATIONS_DISABLED,
};

struct bt_remote_srv_cb {
	void (*notif_changed)(enum bt_data_notifications_enabled status);
	void (*data_rx)(struct bt_conn *conn, const uint8_t *const data, uint16_t len);
};
/* Function declarations for bt_iphone.c*/
void set_data(uint8_t *mic_data);
int send_data_notification(struct bt_conn *conn, uint8_t *value, uint16_t length);
int bluetooth_init(struct bt_conn_cb *bt_cb, struct bt_remote_srv_cb *remote_cb);
#endif