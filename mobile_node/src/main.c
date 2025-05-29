/*
 * Copyright (c) 2018 Henrik Brix Andersen <henrik@brixandersen.dk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 #include <zephyr/kernel.h>
 #include <zephyr/sys/util.h>
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/sys/printk.h>
 #include <zephyr/types.h>
 #include <stddef.h>
 #include <errno.h>
 
 #include <zephyr/bluetooth/bluetooth.h>
 #include <zephyr/bluetooth/hci.h>
 #include <zephyr/bluetooth/conn.h>
 #include <zephyr/bluetooth/uuid.h>
 #include <zephyr/bluetooth/gatt.h>
 #include <zephyr/sys/byteorder.h>

 static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
 static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
 static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
 
 #ifndef IBEACON_RSSI
 #define IBEACON_RSSI 0xC8
 #endif

 uint8_t nodeAddr[13][6] = {
	{0xF5, 0x75, 0xFE, 0x85, 0x34, 0x67},	// A
	{0xE5, 0x73, 0x87, 0x06, 0x1E, 0x86},	// B
	{0xCA, 0x99, 0x9E, 0xFD, 0x98, 0xB1},	// C
	{0xCB, 0x1B, 0x89, 0x82, 0xFF, 0xFE},	// D
	{0xD4, 0xD2, 0xA0, 0xA4, 0x5C, 0xAC},	// E
	{0xC1, 0x13, 0x27, 0xE9, 0xB7, 0x7C},	// F
	{0xF1, 0x04, 0x48, 0x06, 0x39, 0xA0},	// G
	{0xCA, 0x0C, 0xE0, 0xDB, 0xCE, 0x60},	// H
};

 uint8_t ibeacon_payload[25] = {
	0x00, 0x4C, /* Apple */
	0x02, 0x15, /* iBeacon */
	0x55, 0x55, 0x55, 0x55, /* UUID[15..12] */
	0x55, 0x55, /* UUID[11..10] */
	0x55, 0x55, /* UUID[9..8] */
	0x55, 0x55, /* UUID[7..6] */
	0x55, 0x55, 0x55, 0x55, 0x55, 0x55, /* UUID[5..0] */
	0x00, 0x00, /* Major */
	0x00, 0x00, /* Minor */
	IBEACON_RSSI /* Calibrated RSSI @ 1m */
 };

 bt_addr_le_t address;

 int8_t rssiArr[14] = {0};
 
 /*
  * Set iBeacon demo advertisement data. These values are for
  * demonstration only and must be changed for production environments!
  *
  * UUID:  18ee1516-016b-4bec-ad96-bcb96d166e97
  * Major: 0
  * Minor: 0
  * RSSI:  -56 dBm
  */
 static struct bt_data ad[] = {
	 BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	 BT_DATA(BT_DATA_MANUFACTURER_DATA, ibeacon_payload, sizeof(ibeacon_payload))
 };
 
 static void bt_ready(int err) {
	 if (err) {
		 printk("Bluetooth init failed (err %d)\n", err);
		 return;
	 }
 
	 printk("Bluetooth initialized\n");
 
	 /* Start advertising */
	 err = bt_le_adv_start(BT_LE_ADV_PARAM(BT_LE_ADV_OPT_SCANNABLE | BT_LE_ADV_OPT_USE_IDENTITY, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL), ad, ARRAY_SIZE(ad), NULL, 0);
	 if (err) {
		 printk("Advertising failed to start (err %d)\n", err);
		 return;
	 }
 
	 printk("iBeacon started\n");
 }

 void update_ad(uint8_t b0, int8_t b1, uint8_t b2, int8_t b3) {
	//printk("Updating ad data...\r\n");
    // Update the RSSI value in the array
    ibeacon_payload[20] = b0;  // Example update
    ibeacon_payload[21] = b1;  // Example update
    ibeacon_payload[22] = b2;  // Example update
    ibeacon_payload[23] = b3;  // Example update

	bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
 }
 
 uint8_t get_ibeacon_index(const bt_addr_le_t *addr) {
	//printk("Getting index...\r\n");
	for (uint8_t i = 0; i < 13; i++) {
		for (uint8_t j = 0; j < 6; j++) {
			if (nodeAddr[i][j] == addr->a.val[5 - j]) {
				if (j == 5) {
					// thats the one
					//printk("Matching iBeacon found: %d\r\n", i);
					return i;
				}
			} else {
				break;
			}
		}
	}
	return 0xFF;
 }

 static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
	struct net_buf_simple *ad) {
	if (type != BT_GAP_ADV_TYPE_ADV_IND) {
		return;
	}

	uint8_t rssiIndex = get_ibeacon_index(addr);
	if (rssiIndex == 0xFF) {
		//printk("iBeacon not in list\r\n");
		return;
	} else {
		//printk("Received RSSI: %d\r\n", rssi);
		rssiArr[get_ibeacon_index(addr)] = rssi;
	}
 }

static void start_scan(void) {
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

 int main(void) {
	gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_ACTIVE);
	 int err;
 
	 //printk("Starting iBeacon Demo\n");
 
	 /* Initialize the Bluetooth Subsystem */
	 bt_id_create(&address, NULL);
	 err = bt_enable(bt_ready);

	 start_scan();

	 while (1) {
		//printk("Sent RSSI: %d\r\n", rssiArr[0]);
		for (int i = 0; i < 8; i++) {
			update_ad((uint8_t)(i << 1), rssiArr[i << 1], (uint8_t)((i << 1) + 1), rssiArr[(i << 1) + 1]);
			k_msleep(10);
		}
	 }

	 return 0;
 }
 