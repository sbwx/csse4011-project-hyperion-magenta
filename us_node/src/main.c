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

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifndef IBEACON_RSSI
#define IBEACON_RSSI 0xC8
#endif

#define STACK_SIZE 1024
#define TRANSMIT_PRIORITY 6
#define SENSING_PRIORITY 7
#define IBEACON_LENGTH 25
#define UINT32_IN_CHAR 4
#define IBEACON_START 20
#define RSSI_LEN 4
#define CHAR_IN_BITS 8
#define THREE_CHARS_IN_BITS CHAR_IN_BITS * 3
#define TWO_CHARS_IN_BITS CHAR_IN_BITS * 2
#define CHAR_IN_UINT32 0x000000FF
#define BLUETOOTH_ADVERTISING_SETTINGS BT_LE_ADV_PARAM(    \
     BT_LE_ADV_OPT_SCANNABLE | BT_LE_ADV_OPT_USE_IDENTITY, \
     BT_GAP_ADV_FAST_INT_MIN_2,                            \
     BT_GAP_ADV_FAST_INT_MAX_2,                            \
     NULL), ad, ARRAY_SIZE(ad), NULL, 0                    \

K_MSGQ_DEFINE(sensorQueue, sizeof(float), 1, 1);
const struct device* ultrasonicDev = DEVICE_DT_GET(DT_NODELABEL(ultrasonic));

uint8_t ibeacon_payload[IBEACON_LENGTH] = {
    0x00, 0x4C, /* Apple */
    0x02, 0x15, /* iBeacon */
    0x99, 0x99, 0x99, 0x99, /* UUID[15..12] */
    0x99, 0x99, /* UUID[11..10] */
    0x99, 0x99, /* UUID[9..8] */
    0x99, 0x99, /* UUID[7..6] */
    0x99, 0x99, 0x99, 0x99, 0x99, 0x99, /* UUID[5..0] */
    0x00, 0x00, /* Major */
    0x00, 0x00, /* Minor */
    IBEACON_RSSI /* Calibrated RSSI @ 1m */
};

bt_addr_le_t address;
 
static struct bt_data ad[] = {
 BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
 BT_DATA(BT_DATA_MANUFACTURER_DATA, ibeacon_payload, sizeof(ibeacon_payload))
};

// Convert float to a uint32_t
// Adapted from Q_SQRT function
#define FLOAT_TO_UINT32(f, i) {	\
    float tempFloat = f;	\
    i = *(uint32_t*)&tempFloat;	\
}

void separate_u32(uint32_t x, uint8_t* buf) {
	buf[0] = (uint8_t)((x >> THREE_CHARS_IN_BITS) & CHAR_IN_UINT32);
	buf[1] = (uint8_t)((x >> TWO_CHARS_IN_BITS) & CHAR_IN_UINT32);
	buf[2] = (uint8_t)((x >> CHAR_IN_BITS) & CHAR_IN_UINT32);
	buf[3] = (uint8_t)(x & CHAR_IN_UINT32);
}
 
static void bt_ready(int err)
 {
	 if (err) {
		 printk("Bluetooth init failed (err %d)\n", err);
		 return;
	 }
 
	 printk("Bluetooth initialized\n");
 
	 /* Start advertising */
	 err = bt_le_adv_start(BLUETOOTH_ADVERTISING_SETTINGS);
	 if (err) {
		 printk("Advertising failed to start (err %d)\n", err);
		 return;
	 }
 
	 printk("iBeacon started\n");
}

void update_ad(uint8_t data[RSSI_LEN]) {
    
    // Update the RSSI value in the array
	for (int i = 0; i < RSSI_LEN; i++) {
        ibeacon_payload[IBEACON_START + i] = data[i];
	}

	bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
}

void sensor_thread()
{

	if (!device_is_ready(ultrasonicDev)) {
        printk("Device isn't ready!\r\n");
        return;
    }

	struct sensor_value ultrasonicDist;
    float sensorValue = 0.f;

	while (1) {

        // Get the ultrasonic node sensor reading
		sensor_sample_fetch(ultrasonicDev);
		sensor_channel_get(ultrasonicDev, SENSOR_CHAN_DISTANCE, &ultrasonicDist);

        // Ultrasonic node distance
		sensorValue = sensor_value_to_float(&ultrasonicDist);
        
        // Overwrite the element in the queue
        k_msgq_purge(&sensorQueue);
        k_msgq_put(&sensorQueue, &sensorValue, K_NO_WAIT);
        k_msleep(50);
    }
}

void transmit_thread()
{
	/* Initialize the Bluetooth Subsystem */
	bt_id_create(&address, NULL);
	int err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

    // Ultrasonic sensor value
    float sensorValue = 0.f; 

    // Temp variable for storing
    // the float before splitting
	uint32_t u32 = 0;

    // Buffer to store the 4 uint8_t's
    // in after splitting from uint32_t
	uint8_t floatBuf[UINT32_IN_CHAR] = {0};
    
    while (1) {
        k_msgq_get(&sensorQueue, &sensorValue, K_FOREVER);
		
        // Convert the data to a uint32_t
        FLOAT_TO_UINT32(sensorValue, u32);

        // Split the uint32_t into uint8_t's
        // for transmission
		separate_u32(u32, floatBuf);
        
        // Update the advertisement data
        k_sched_lock();
        update_ad(floatBuf);
        k_sched_unlock();
        
		k_msleep(10);
    }
}

K_THREAD_DEFINE(sensingId, STACK_SIZE, sensor_thread, NULL, NULL, NULL, SENSING_PRIORITY, 0, 0);
K_THREAD_DEFINE(transmitId, STACK_SIZE, transmit_thread, NULL, NULL, NULL, TRANSMIT_PRIORITY, 0, 0);
