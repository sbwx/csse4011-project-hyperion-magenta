#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

#define STACK_SIZE 4096
#define SCREEN_THREAD_PRIORITY 8
#define BT_THREAD_PRIORITY 9

void screen_thread();
void bt_thread();

K_THREAD_DEFINE(screen_tid, STACK_SIZE, screen_thread, NULL, NULL, NULL, SCREEN_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(bt_tid, STACK_SIZE, bt_thread, NULL, NULL, NULL, BT_THREAD_PRIORITY, 0, 0);

K_MSGQ_DEFINE(lifeQ, sizeof(uint8_t), 1, 1);
K_MSGQ_DEFINE(comboQ, sizeof(uint8_t), 1, 1);
K_MSGQ_DEFINE(scoreQ, sizeof(uint8_t), 1, 1);

LV_IMG_DECLARE(matthew);
LV_IMG_DECLARE(presidentsnow);

void display_matthew() {
	lv_obj_t *img = lv_img_create(lv_scr_act());  // create image object
	lv_img_set_src(img, &presidentsnow);               // set the image source
	lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
}

// function to display game over text
void game_over(uint32_t color) {
	// get active screen object
    lv_obj_t *screen = lv_scr_act();

    //lv_obj_clean(screen);

    // create text label
    lv_obj_t *label = lv_label_create(screen);
    lv_label_set_text(label, "GAME OVER");

    // set font
    lv_obj_set_style_text_font(label, &lv_font_unscii_16, LV_PART_MAIN);

    // set text color
    lv_obj_set_style_text_color(label, lv_color_hex(color), LV_PART_MAIN);

    // center label
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    // update screen
    lv_task_handler();
}

// function to display game over text
void update_lives(uint8_t lives) {
	// get active screen object
    lv_obj_t *screen = lv_scr_act();

    // create text label
    lv_obj_t *lifeLabel = lv_label_create(screen);

	// create string with '|'s corresponding to num lives
	char lifeStr[3] = "";
	for (int i = 0; i < lives; i++) {
		strcat(lifeStr, "|");
	}

	// set text
    lv_label_set_text(lifeLabel, lifeStr);

    // set font
    lv_obj_set_style_text_font(lifeLabel, &lv_font_unscii_16, LV_PART_MAIN);

    // set text color
    lv_obj_set_style_text_color(lifeLabel, lv_color_hex(0xff5555), LV_PART_MAIN);

    // center label
    lv_obj_align(lifeLabel, LV_ALIGN_TOP_LEFT, 8, 10);
}

// function to display combo meter
void update_combo(uint32_t combo) {
	// get active screen object
    lv_obj_t *screen = lv_scr_act();

    // create text label
    lv_obj_t *comboLabel = lv_label_create(screen);

	char comboStr[10] = "";
	sprintf(comboStr, "x%d", combo);

	// set text
    lv_label_set_text(comboLabel, comboStr);

    // set font
    lv_obj_set_style_text_font(comboLabel, &lv_font_unscii_16, LV_PART_MAIN);

    // set text color
    lv_obj_set_style_text_color(comboLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN);

    // center label
    lv_obj_align(comboLabel, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
}

// function to display combo meter
void update_score(uint32_t score) {
	// get active screen object
    lv_obj_t *screen = lv_scr_act();

    // create text label
    lv_obj_t *scoreLabel = lv_label_create(screen);

	char scoreStr[10] = "";
	sprintf(scoreStr, "%d", score);

	// set text
    lv_label_set_text(scoreLabel, scoreStr);

    // set font
    lv_obj_set_style_text_font(scoreLabel, &lv_font_unscii_16, LV_PART_MAIN);

    // set text color
    lv_obj_set_style_text_color(scoreLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN);

    // center label
    lv_obj_align(scoreLabel, LV_ALIGN_CENTER, 0, 0);
}

// wrapper function to update all stats, clearing screen
void update_stats(uint8_t lives, uint32_t score, uint32_t combo) {
    lv_obj_t *screen = lv_scr_act();
    lv_obj_clean(screen);

	update_lives(lives);
	update_score(score);
	update_combo(combo);
	
    // update screen
    lv_task_handler();
}

void screen_thread() {
	// get screen device
	const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	// check device ready
    if (!device_is_ready(display_dev)) {
        printk("Display not ready\n");
        return;
    }
    lv_obj_t *screen = lv_scr_act();

	// init screen
    display_blanking_off(display_dev);
    lv_obj_clean(screen);
	lv_obj_set_style_bg_color(screen, lv_color_hex(0x003a57), LV_PART_MAIN);

	uint8_t lives = 0;
	uint32_t score = 0;
	uint8_t combo = 0;

	uint32_t toggle = 0x00000000;

	k_msgq_get(&lifeQ, &lives, K_NO_WAIT);
	k_msgq_get(&scoreQ, &score, K_NO_WAIT);
	k_msgq_get(&comboQ, &combo, K_NO_WAIT);

	while (1) {
		if (lives > 0) {
			k_msgq_get(&lifeQ, &lives, K_NO_WAIT);
			k_msgq_get(&scoreQ, &score, K_NO_WAIT);
			k_msgq_get(&comboQ, &combo, K_NO_WAIT);
			update_stats(lives, (score * 100), combo);
			k_msleep(1);
		} else {
			k_msgq_get(&lifeQ, &lives, K_NO_WAIT);
			if (lives > 0) {
				continue;
			}
			lv_obj_clean(screen);
			display_matthew();
			game_over(toggle);
			toggle = 0x00FFFFFF - toggle;
			k_msleep(50);
		}
	}
	return;
}

 // ad parsing function for ultrasonic adv
 static bool parse_base_cb(struct bt_data *data, void *user_data) {
	uint8_t lives;
	uint8_t combo;
	uint8_t score;
	if (data->type == BT_DATA_MANUFACTURER_DATA && data->data_len >= 25) {
		const uint8_t *d = data->data;
		lives = d[20];
		printk("Lives:	%d\r\n", lives);
		combo = d[21];
		printk("Combo:	%d\r\n", combo);
		score = d[22];
		printk("Score:	%d\r\n", score);


		k_msgq_put(&lifeQ, &lives, K_NO_WAIT);

		k_msgq_put(&comboQ, &combo, K_NO_WAIT);

		k_msgq_put(&scoreQ, &score, K_NO_WAIT);
	}
	return true;
}

 // return 0 if not mobile mac address
 uint8_t check_base(const bt_addr_le_t *addr) {
	// mac address for mobile node (THINGY52)
	uint8_t baseAddr[6] = {0x97, 0x6D, 0xE6, 0x30, 0x15, 0xD3};
	for (uint8_t i = 0; i < 6; i++) {
		if (addr->a.val[i] == baseAddr[i]) {
			if (i == 5) {
				// thats the one
				printk("matching found\r\n");
				return 1;
			}
		} else {
			break;
		}
	}
	return 0;
 }

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
	struct net_buf_simple *ad) {
	if (type != BT_GAP_ADV_TYPE_ADV_SCAN_IND) {
		return;
	} 

	if (check_base(addr)) {
		bt_data_parse(ad, parse_base_cb, NULL);
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

void bt_thread(void) {
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);	
		return 0;
	}

	printk("Bluetooth initialized\n");

	start_scan();
	
	return;
}