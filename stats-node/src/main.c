/* main.c - Application main entry point */

/*
* Copyright (c) 2015-2016 Intel Corporation
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <zephyr/types.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/shell/shell.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>
#include <math.h>
#include <string.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>

struct ibeacon_node {
    struct rbnode rb_n;
    char name[32];
	char mac[18];
	uint16_t major;
	uint16_t minor;
	float x;
	float y;
	char left[32];
	char right[32];
};

// function prototype for scanning
static void start_scan(void);

// uart shell device pointer
const struct device* dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));

// macro for converting uint32 to float while preserving bit order
#define UINT32_TO_FLOAT(i, f) {	\
    uint32_t tempInt = i;	\	
    f = *(float*)&tempInt;	\
}

 // helper function for joining 8 uint8s into one uint32
uint32_t join_u32(uint8_t* fArray) {
	return (((uint32_t)fArray[0] << 24) |
			((uint32_t)fArray[1] << 16) |
			((uint32_t)fArray[2] << 8) 	|
			((uint32_t)fArray[3]) );
}

// return 0 if not mobile mac address
uint8_t check_mobile(const bt_addr_le_t *addr) {
	// mac address for mobile node (THINGY52)
	uint8_t mobileAddr[6] = {0xFE, 0xC0, 0x04, 0xFD, 0x5D, 0xC5};
	for (uint8_t i = 0; i < 6; i++) {
		if (addr->a.val[i] == mobileAddr[i]) {
			if (i == 5) {
				// thats the one
				//printk("Matching iBeacon found\r\n");
				return 1;
			}
		} else {
			break;
		}
	}
	return 0;
 }
 
// callback function for when device found
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad) { 
	 // only check indirect scannable ads
	 if (type != BT_GAP_ADV_TYPE_ADV_SCAN_IND) {
		// check scannable not connectable
		 return;
	 }
	 // check if mac matches mobile
	 if (check_mobile(addr)) {
		//printk("MOBILE FOUND\r\n");
		bt_data_parse(ad, parse_mobile_cb, NULL);
		return;
	 }
	 // check if mac matches ultrasonic
	 if (check_ultrasonic(addr)) {
		//printk("ULTRASONIC FOUND\r\n");
		bt_data_parse(ad, parse_us_cb, NULL);
		return;
	 }

	 //printk("ADDR: %x:%x:%x:%x:%x:%x\r\n", addr->a.val[0], addr->a.val[1], addr->a.val[2], addr->a.val[3], addr->a.val[4], addr->a.val[5]);
 
}
 
 // function that sets up bt scanning
static void start_scan(void) {
	 int err;
 
	 err = bt_le_scan_start(BT_LE_SCAN_PASSIVE_CONTINUOUS, device_found);
	 if (err) {
		 printk("Scanning failed to start (err %d)\n", err);
		 return;
	 }
 
	 printk("Scanning successfully started\n");
 }

 // helper function that rounds float to nearest 0.5
 float round_to_half(float x) {
	return round(x * 2.0) / 2.0;
}

lv_obj_t* draw_circle_outline() 
{

    static lv_obj_t* circle;
    circle = lv_obj_create(lv_scr_act());
    lv_obj_set_scrollbar_mode(circle, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(circle, 8, 8);
    lv_obj_set_pos(circle, 64-6, 32-6);
    lv_obj_set_style_bg_color(circle, (lv_color_t)LV_COLOR_MAKE(255, 255, 255), 0);
    lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, 0);

    // Set border color and width
    lv_obj_set_style_border_color(circle, (lv_color_t)LV_COLOR_MAKE(0,0,0), LV_PART_MAIN);
    lv_obj_set_style_border_width(circle, 1, LV_PART_MAIN);
    
    return circle;

 }

/* Draw Rectangle
* 
* Draws a rectangle at a given (x,y)
*
*/
lv_obj_t* draw_rect(int x, int y)
{
    static lv_obj_t* node;
    node = lv_obj_create(lv_scr_act());
    lv_obj_set_size(node, 8, 8);
    lv_obj_set_pos(node, x, y);
    lv_obj_set_style_bg_color(node, (lv_color_t)LV_COLOR_MAKE(0,0,0), 0);
	return node;
}

int main(void) {

	int err;
 
	 // attempt to enable bt
	 err = bt_enable(NULL);
	 if (err) {
		 printk("Bluetooth init failed (err %d)\n", err);
		 return 0;
	 }
	 printk("Bluetooth initialized\n");
 
	 // start scanning
	 start_scan();

     static lv_obj_t* node1;
     node1 = draw_rect(160 - 8, 120 - 8);
   
     while (1) {
        lv_timer_handler();
		k_msleep(500);
	 }
	 return 0;
}
