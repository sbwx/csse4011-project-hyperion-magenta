#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/display.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/data/json.h>

#include "tones.h"
#include "imperial_march.h"

#define JSON_BUFFER_SIZE 100

// Sensor object for use with JSON descriptor
struct tagioObj {
    char* variable;
	char* value;
    char* unit;
};

// JSON object descriptor for generating JSON messages
static const struct json_obj_descr tagioObjDescriptor[] = {
    JSON_OBJ_DESCR_PRIM(struct tagioObj, variable, JSON_TOK_STRING),
    JSON_OBJ_DESCR_PRIM(struct tagioObj, value, JSON_TOK_STRING),
    JSON_OBJ_DESCR_PRIM(struct tagioObj, unit,  JSON_TOK_STRING)
};

char jsonDataBuf[JSON_BUFFER_SIZE] = {0};

#ifndef IBEACON_RSSI
#define IBEACON_RSSI 0xC8
#endif

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

#define UNCALIBRATED_RSSIS	{-45.f, -58.f, -55.f, -53.f, -47.f, -59.f, -62.f, -47.f, -63.f, -60.f, -67.f, -69.f, -59.f}

#define CALIBRATED_RSSIS	{-59.f, -68.f, -55.f, -68.f, -62.f, -56.f, -65.f, -57.f, -63.f, -60.f, -67.f, -69.f, -59.f}
#define BASE_COORDS {	{0.0f, 0.0f}, {1.5f, 0.0f}, {3.0f, 0.0f}, {3.0f, 2.0f}, {3.0f, 4.0f}, {1.5f, 4.0f}, {0.0f, 4.0f}, {0.0f, 2.0f}	}
#define ENVIRONMENTAL_CONSTANT 3.f

#define DIM 2
#define DIST_MAX_ENTRIES	8

/* Audio settings */
#define SAMPLE_RATE         44100
#define CHANNELS            2
#define BITS_PER_SAMPLE     16
#define BYTES_PER_SAMPLE    (BITS_PER_SAMPLE / 8)
#define BLOCK_SAMPLES       256
#define BLOCK_SIZE          (BLOCK_SAMPLES * CHANNELS * BYTES_PER_SAMPLE)

K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, 4, 4);

static const struct device *i2s_dev;

typedef struct KalmanFilter {
    float measuredErr;
    float estimatedErr;
    float noise;
    float currEstimate;
    float lastEstimate;
    float kalmanGain;
}KalmanFilter;

int8_t stupidArr[8] = {0};
float nodes[8][2] = BASE_COORDS;

#define STRIP_NODE		DT_ALIAS(led_strip)

#if DT_NODE_HAS_PROP(DT_ALIAS(led_strip), chain_length)
#define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)
#else
#error Unable to determine length of LED strip
#endif

//#define DELAY_TIME K_MSEC(CONFIG_SAMPLE_LED_UPDATE_DELAY)

#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

static const struct led_rgb colors[] = {
	RGB(0x08, 0x00, 0x00), /* red */
	RGB(0x00, 0x08, 0x00), /* green */
	RGB(0x00, 0x00, 0x08), /* blue */
	RGB(0x06, 0x00, 0x02), /* pink??? */
	RGB(0x02, 0x00, 0x06), /* pink??? */
};

typedef struct BlockPos {
    uint8_t x;
    uint8_t y;
}Block;

static struct led_rgb pixels[STRIP_NUM_PIXELS];

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

K_MSGQ_DEFINE(playerPos, sizeof(uint8_t), 1, 1);
K_MSGQ_DEFINE(blockQ, sizeof(Block*), 8, 1);

K_MSGQ_DEFINE(leftQ, sizeof(float), 1, 1);
K_MSGQ_DEFINE(midQ, sizeof(float), 1, 1);
K_MSGQ_DEFINE(rightQ, sizeof(float), 1, 1);

K_MSGQ_DEFINE(rssiQ, sizeof(int8_t*), 1, 1);

K_MSGQ_DEFINE(stateQ, sizeof(uint8_t), 1, 1);
K_MSGQ_DEFINE(lifeQ, sizeof(uint8_t), 1, 1);
K_MSGQ_DEFINE(scoreQ, sizeof(uint8_t), 1, 1);
K_MSGQ_DEFINE(comboQ, sizeof(uint8_t), 1, 1);

K_SEM_DEFINE(gameStart, 0, 1);
const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button1_cb_data;

#define STACK_SIZE 4096
#define SCREEN_THREAD_PRIORITY 8
#define BT_THREAD_PRIORITY 9
#define SPEAKER_THREAD_PRIORITY 7
#define TX_THREAD_PRIORITY 6

void screen_thread();
void bt_thread();
void speaker_thread();
void tx_thread();

K_THREAD_DEFINE(screen_tid, STACK_SIZE, screen_thread, NULL, NULL, NULL, SCREEN_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(bt_tid, STACK_SIZE, bt_thread, NULL, NULL, NULL, BT_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(speaker_tid, STACK_SIZE, speaker_thread, NULL, NULL, NULL, SPEAKER_THREAD_PRIORITY, 0, 0);
//K_THREAD_DEFINE(tx_tid, STACK_SIZE, tx_thread, NULL, NULL, NULL, TX_THREAD_PRIORITY, 0, 0);


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

void clear_pixels(uint8_t pos) {
    memset(&(pixels[pos]), 0x00, sizeof(struct led_rgb) * 2);
}

void clear_block_top(uint8_t x, uint8_t y) {
    if (y % 2 != 0)
        clear_pixels(255 + (2 * x) - (8 * (y + 2)) - 6 - 1);
    else
        clear_pixels(255 - (2 * x) - (8 * y) - 16 - 1);
}

void set_player_position(uint8_t x) {
    // Remove the 2 pixels for the bottom row
    memset(&(pixels[255 - (2 * x) - 1]), 0x00, sizeof(struct led_rgb) * 2);
    
    // Remove the 2 pixels for the top row
    memset(&(pixels[255 - 14 + (2 * x) - 1]), 0x00, sizeof(struct led_rgb) * 2);
    
    memcpy(&(pixels[255 - (2 * x)]), &colors[4], sizeof(struct led_rgb));
    memcpy(&(pixels[255 - (2 * x) - 1]), &colors[4], sizeof(struct led_rgb));
    memcpy(&(pixels[255 - 14 + (2 * x)]), &colors[4], sizeof(struct led_rgb));
    memcpy(&(pixels[255 - 14 + (2 * x) - 1]), &colors[4], sizeof(struct led_rgb));

}

void set_note(uint8_t x, uint8_t y) {
    if (y % 2 != 0) {
        // Remove the 2 pixels for the bottom row
        memset(&(pixels[255 - (2 * x) - (8 * y) - 8 - 1]), 0x00, sizeof(struct led_rgb) * 2);
        
        // Remove the 2 pixels for the top row
        memset(&(pixels[255 - 8 + (2 * x) - (8 * y) - 1 + 2]), 0x00, sizeof(struct led_rgb) * 2);
        
        memcpy(&(pixels[255 - (2 * x) - (8 * y) - 8]), &colors[2], sizeof(struct led_rgb));
        memcpy(&(pixels[255 - (2 * x) - (8 * y) - 1 - 8]), &colors[2], sizeof(struct led_rgb));
        memcpy(&(pixels[255 - 8 + (2 * x) - (8 * y) + 2]), &colors[2], sizeof(struct led_rgb));
        memcpy(&(pixels[255 - 8 + (2 * x) - (8 * y) - 1 + 2]), &colors[2], sizeof(struct led_rgb));

    } else {

        // Remove the 2 pixels for the bottom row
        memset(&(pixels[255 - (2 * x) - (8 * y) - 1]), 0x00, sizeof(struct led_rgb) * 2);
        
        // Remove the 2 pixels for the top row
        memset(&(pixels[255 - 14 + (2 * x) - (8 * y) - 1]), 0x00, sizeof(struct led_rgb) * 2);
        
        memcpy(&(pixels[255 - (2 * x) - (8 * y)]), &colors[2], sizeof(struct led_rgb));
        memcpy(&(pixels[255 - (2 * x) - (8 * y) - 1]), &colors[2], sizeof(struct led_rgb));
        memcpy(&(pixels[255 - 14 + (2 * x) - (8 * y)]), &colors[2], sizeof(struct led_rgb));
        memcpy(&(pixels[255 - 14 + (2 * x) - (8 * y) - 1]), &colors[2], sizeof(struct led_rgb));

    }
    
}

void set_bottom_pixels(const struct led_rgb* color) {
    // Clear the bottom 4 rows
    memset(&(pixels[208]), 0x00, sizeof(struct led_rgb) * 32);
    for (uint8_t i = 0; i < 8; i++)
        memcpy(&(pixels[208 + i]), color, sizeof(struct led_rgb));
    
    uint8_t pos;
    if(k_msgq_get(&playerPos, &pos, K_NO_WAIT)) {
        return;
    }
    memset(&(pixels[240]), 0x00, sizeof(struct led_rgb) * 16);
    set_player_position(pos);
}

void clear_rgb_matrix()
{
    memset(&(pixels[0]), 0x00, sizeof(struct led_rgb) * 255);
}

void button1_pressed(const struct device *dev, struct gpio_callback *cb) 
{
    k_sem_give(&gameStart);
}

void screen_thread(void) {
	size_t color = 0;
	int rc;
    uint8_t score = 0;
    uint8_t lives = 0;
    uint8_t combo = 0;

    gpio_pin_configure_dt(&button1, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button1_cb_data, button1_pressed, BIT(button1.pin));
	gpio_add_callback(button1.port, &button1_cb_data);

    // The colour to draw the strip as
    struct led_rgb* colour = &colors[3];

	if (!device_is_ready(strip)) {
		return;
	}

    while (1) {

        // If you're dead
        if (!lives) {
            k_msgq_purge(&blockQ);
            clear_rgb_matrix();

            // Attempt to take the sem for game start
            k_sem_take(&gameStart, K_FOREVER);
            
            // Init all values
            lives = 3;
            score = 0;
            combo = 0;

            
            k_msgq_purge(&lifeQ);
            k_msgq_put(&lifeQ, &lives, K_NO_WAIT);
           
            k_msgq_purge(&comboQ);
            k_msgq_put(&comboQ, &combo, K_NO_WAIT);
           
            k_msgq_purge(&scoreQ);
            k_msgq_put(&scoreQ, &score, K_NO_WAIT);
        }

        Block* latestBlock;
        uint8_t numBlocks = k_msgq_num_used_get(&blockQ);
        
        if (!k_msgq_peek_at(&blockQ, &latestBlock, numBlocks - 1) && numBlocks != 0) {
        
            // Check the block is hitting the line
            if (latestBlock->y < 27) {
                
                // Render a new block at the top of the screen
                Block* block = (Block*)k_malloc(sizeof(Block));
                block->x = sys_rand8_get() % 4;
                block->y = 30;

                k_msgq_put(&blockQ, &block, K_NO_WAIT);
            
            }
        } else if (numBlocks == 0) {
        
            Block* block = (Block*)k_malloc(sizeof(Block));
            block->x = sys_rand8_get() % 4;
            block->y = 30;

            k_msgq_put(&blockQ, &block, K_NO_WAIT);
        
        }
            
        for (uint8_t i = 0; i < 8; i++) {
            Block* currBlock;

            // If the block exists in the queue
            k_msgq_peek(&blockQ, &currBlock);
                
            if (currBlock->y < 4) {
                k_msgq_get(&blockQ, &currBlock, K_NO_WAIT);

                // Get the player position
                uint8_t pos = 0;
                k_msgq_peek(&playerPos, &pos);

                // We're in the right spot to hit the block
                if (pos == currBlock->x) {
                    // Set the strip colour to green
                    colour = &colors[1];
                    score++;
                    combo++;
                    k_msgq_purge(&scoreQ);
                    k_msgq_put(&scoreQ, &score, K_NO_WAIT);
                    
                    k_msgq_purge(&comboQ);
                    k_msgq_put(&comboQ, &combo, K_NO_WAIT);
                } else {
                    // Set the strip colour to red
                    colour = &colors[0];
                    combo = 0;
                    lives--;

                    k_msgq_purge(&lifeQ);
                    k_msgq_put(&lifeQ, &lives, K_NO_WAIT);

                    k_msgq_purge(&comboQ);
                    k_msgq_put(&comboQ, &combo, K_NO_WAIT);
                }

                k_free(currBlock);
                break;
                
            } else {
                // Set the strip colour to magenta
                colour = &colors[3];
            }
        }

        for (int i = 0; i < 8; i++) {

            Block* currBlock;

            if (!k_msgq_peek_at(&blockQ, &currBlock, i)) {
    
                set_note(currBlock->x, currBlock->y);
                clear_block_top(currBlock->x, currBlock->y); 
                
                set_bottom_pixels(colour);
                
                led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
            }
        }
        
        for (uint8_t i = 0; i < 8; i++) {
            Block* currBlock;

            // If the block exists in the queue
            if (!k_msgq_peek_at(&blockQ, &currBlock, i)) {
                (currBlock->y) -= 1;
            }
        }
        k_msleep(400);
    }
	return;
}

 uint8_t get_node(const bt_addr_le_t *addr) {
	uint8_t leftAddr[6] = {0x1E, 0xD2, 0xE4, 0x9F, 0x28, 0xEC};
	uint8_t rightAddr[6] = {0x90, 0x75, 0x13, 0xED, 0xBA, 0xD7};
	// mac address for mobile node (THINGY52)
	uint8_t mobileAddr[6] = {0xFE, 0xC0, 0x04, 0xFD, 0x5D, 0xC5};

	for (uint8_t i = 0; i < 6; i++) {
		if (addr->a.val[i] == leftAddr[i]) {
			if (i == 5) {
				// thats the one
				return 0;
			}
		} else {
			break;
		}
	}
    for (uint8_t i = 0; i < 6; i++) {
		if (addr->a.val[i] == rightAddr[i]) {
			if (i == 5) {
				// thats the one
				return 2;
			}
		} else {
			break;
		}
	}
    for (uint8_t i = 0; i < 6; i++) {
		if (addr->a.val[i] == mobileAddr[i]) {
			if (i == 5) {
				// thats the one
				return 3;
			}
		} else {
			break;
		}
	}

	return -1;
 }

// ad parsing function for ultrasonic adv
static bool parse_us_cb(struct bt_data *data, void *user_data) {
    // something
    uint8_t pos = *(uint8_t*)user_data;
    struct k_msgq* usNodes[3] = {&leftQ, &midQ, &rightQ};
	// temp variable to store individual 4 bytes from ultrasonic node transmission
	uint8_t float_part[4] = {0};
    float tempFloat;

	if (data->type == BT_DATA_MANUFACTURER_DATA && data->data_len >= 25) {
		const uint8_t *d = data->data;
		// store the 4 bytes making up uint32
		for (int i = 0; i < 4; i++) {
			float_part[i] = d[20 + i];
		}
		// join the 4 bytes to make uint32
        UINT32_TO_FLOAT(join_u32(float_part), tempFloat);
        k_msgq_put(usNodes[pos], &tempFloat, K_NO_WAIT);
	}
	return true;
}

// ad parsing function for ultrasonic adv
static bool parse_mobile_cb(struct bt_data *data, void *user_data) {
/*     int8_t* rssiArr;
    k_msgq_peek(&rssiQ, &rssiArr); */
	// temp variable to store individual 4 bytes from ultrasonic node transmission

	if (data->type == BT_DATA_MANUFACTURER_DATA && data->data_len >= 25) {
		const uint8_t *d = data->data;
		if ((d[20] < 8)) {
		    stupidArr[d[20]] = (int8_t)d[21];
        } 
        if (d[22] < 8) {
		    stupidArr[d[22]] = (int8_t)d[23];
        }
	}
	return true;
}

// callback function for when device found
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad) { 
    // only check indirect scannable ads
	if (type != BT_GAP_ADV_TYPE_ADV_SCAN_IND) {
		return;
	} 
    uint8_t pos = get_node(addr);
    if (pos < 0) {
        return;
    } else if (pos == 3) {
        bt_data_parse(ad, parse_mobile_cb, NULL);
        return;
    }
    bt_data_parse(ad, parse_us_cb, &pos);
}

float pick_closest(float one, float two) {
    return (one < two);
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

 // RSSI to distance calculation
 float rssi_to_dist(uint8_t nodeIndex, int8_t rssi) {
	float calibratedRssi[13] = CALIBRATED_RSSIS;
	//return (powf(((300000000.f * CALIBRATED_RSSI) / rssi), (1 / ENVIRONMENTAL_CONSTANT)));
	return (powf(10.f, (calibratedRssi[nodeIndex] - rssi) / (10.f * ENVIRONMENTAL_CONSTANT)));
 }

 void multilateration_new(float stations[8][2], double* r, double x[DIM]) {
    int valid_count = 0;
    float* valid_stations[DIST_MAX_ENTRIES];
    double valid_r[DIST_MAX_ENTRIES];

    //printk("Filtering valid stations...\n");
    for (int i = 0; i < DIST_MAX_ENTRIES; ++i) {
        if (stations[i] != NULL) {
            valid_stations[valid_count] = stations[i];
            valid_r[valid_count] = r[i];
/*             printk("  Station %d is valid: (%.2f, %.2f), distance: %.2f\n",
                   i, stations[i][0], stations[i][1], r[i]); */
            valid_count++;
        }
    }

    if (valid_count < DIM + 1) {
        //printk("Not enough valid stations. Need at least %d, got %d.\n", DIM + 1, valid_count);
        return;
    }

    double A[DIST_MAX_ENTRIES][DIM];
    double b[DIST_MAX_ENTRIES];

    //printk("Building A matrix and b vector...\n");
    for (int i = 1; i < valid_count; ++i) {
        double r_diff = valid_r[0] * valid_r[0] - valid_r[i] * valid_r[i];
        double c_diff = 0.0;

        for (int d = 0; d < DIM; ++d) {
            A[i - 1][d] = 2.0 * ((double)valid_stations[i][d] - (double)valid_stations[0][d]);
            c_diff += valid_stations[i][d] * valid_stations[i][d] -
                      valid_stations[0][d] * valid_stations[0][d];
        }

        b[i - 1] = r_diff + c_diff;

        //printk("  Row %d: A = [%.2f, %.2f], b = %.2f\n", i - 1, A[i - 1][0], A[i - 1][1], b[i - 1]);
    }

    int equations = valid_count - 1;

    double AtA[DIM][DIM] = {0};
    double Atb[DIM] = {0};

    //printk("Computing AtA and Atb...\n");
	const double epsilon = 1e-6;
	for (int i = 0; i < DIM; ++i) {
		for (int j = 0; j < DIM; ++j) {
			for (int k = 0; k < equations; ++k) {
				double weight = 1.0 / (valid_r[k + 1] * valid_r[k + 1] + epsilon);  // k+1 because A starts from index 1
				AtA[i][j] += weight * A[k][i] * A[k][j];
			}
			//printk("  AtA[%d][%d] = %.4f\n", i, j, AtA[i][j]);
		}

		for (int k = 0; k < equations; ++k) {
			double weight = 1.0 / (valid_r[k + 1] * valid_r[k + 1] + epsilon);
			Atb[i] += weight * A[k][i] * b[k];
		}
		//printk("  Atb[%d] = %.4f\n", i, Atb[i]);
	}

/*     for (int i = 0; i < DIM; ++i) {
        for (int j = 0; j < DIM; ++j) {
            for (int k = 0; k < equations; ++k) {
                AtA[i][j] += A[k][i] * A[k][j];
            }
            //printk("  AtA[%d][%d] = %.4f\n", i, j, AtA[i][j]);
        }
        for (int k = 0; k < equations; ++k) {
            Atb[i] += A[k][i] * b[k];
        }
        //printk("  Atb[%d] = %.4f\n", i, Atb[i]);
    } */

    double det = AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0];
    //printk("Determinant = %.8f\n", det);
    if (fabs(det) < 1e-10) {
        printk("Matrix is singular or nearly singular.\n");
        return;
    }

    x[0] = (Atb[0] * AtA[1][1] - Atb[1] * AtA[0][1]) / det;
    x[1] = (Atb[1] * AtA[0][0] - Atb[0] * AtA[1][0]) / det;

    //printk("Raw solution: x = [%.4f, %.4f]\n", x[0], x[1]);
}

float estimate(float measurement, KalmanFilter* kalman) {
    // Update Kalman Gain
    kalman->kalmanGain = kalman->estimatedErr / (kalman->estimatedErr + kalman->measuredErr);
    
    // Get the current estimate
    kalman->currEstimate = kalman->lastEstimate + kalman->kalmanGain * (measurement - kalman->lastEstimate);
    
    // Get the estimated error
    kalman->estimatedErr = (1.f - kalman->kalmanGain) * kalman->estimatedErr + fabsf(kalman->lastEstimate - kalman->currEstimate) * kalman->noise;
    // Update the estimates
    kalman->lastEstimate = kalman->currEstimate;

    return kalman->currEstimate;
}

void bt_thread() {
    float leftVal = 0;
    float rightVal = 0;
    uint8_t tempPos = 0;
    double coords[DIM] = {0.f};
    double distArr[8] = {0};

    uint8_t avIndex = 0;
    float leftWindow[3] = {0};
    float rightWindow[3] = {0};

    double multiY = 0;
    float rssiWindow[3] = {0};

    float distLWindow[3] = {0};
    float distRWindow[3] = {0};

    float leftDist = 0;
    float rightDist = 0;

    uint8_t lives = 0;
    uint8_t score = 0;
    uint8_t combo = 0;

    struct tagioObj fuck;

    KalmanFilter rssiKalman = { 
		.kalmanGain = 0.f,
		.lastEstimate = 0.f,
		.currEstimate = 0.f,
		.measuredErr = 0.5f,
		.estimatedErr = 1.f,
		.noise = 0.1f
	};

    KalmanFilter Kalman = { 
		.kalmanGain = 0.f,
		.lastEstimate = 0.f,
		.currEstimate = 0.f,
		.measuredErr = 0.5f,
		.estimatedErr = 1.f,
		.noise = 0.1f
	};

    KalmanFilter leftKalman = { 
		.kalmanGain = 0.f,
		.lastEstimate = 0.f,
		.currEstimate = 0.f,
		.measuredErr = 0.012f,
		.estimatedErr = 1.f,
		.noise = 0.1f
	};

    KalmanFilter rightKalman = { 
		.kalmanGain = 0.f,
		.lastEstimate = 0.f,
		.currEstimate = 0.f,
		.measuredErr = 0.012f,
		.estimatedErr = 1.f,
		.noise = 0.1f
	};

    KalmanFilter distLKalman = { 
		.kalmanGain = 0.f,
		.lastEstimate = 0.f,
		.currEstimate = 0.f,
		.measuredErr = 0.012f,
		.estimatedErr = 1.f,
		.noise = 0.1f
	};

    KalmanFilter distRKalman = { 
		.kalmanGain = 0.f,
		.lastEstimate = 0.f,
		.currEstimate = 0.f,
		.measuredErr = 0.012f,
		.estimatedErr = 1.f,
		.noise = 0.1f
	};

        /* Initialize the Bluetooth Subsystem */
        bt_id_create(&address, NULL);
        int err = bt_enable(bt_ready);
    
        if (err) {
            printk("Bluetooth init failed (err %d)\n", err);	
            return;
        }
    
        printk("Bluetooth initialized\n");
    
        start_scan();

    // teehee
/* 	int8_t* rssiArr = (int8_t*)k_calloc(DIST_MAX_ENTRIES, sizeof(int8_t));
    k_msgq_put(&rssiQ, &rssiArr, K_NO_WAIT);
    
    float** cunt = (float**)k_calloc(20, sizeof(float*));
    float fuck[13][2] = BASE_COORDS;

    for (uint8_t i = 0; i < 20; i++)
        cunt[i] = (float*)k_calloc(2, sizeof(float));

    for (uint8_t i = 0; i < 13; i++) {
        for (uint8_t j = 0; j < 2; j++) {
            cunt[i][j] = fuck[i][j];
        }
    } */

    while (1) {
        for (int i = 0; i < 8; i++) {
            distArr[i] = rssi_to_dist(i, stupidArr[i]);
            //printk("Dist %d:    %f\r\n", i, distArr[i]);
        }

        distLWindow[avIndex] = distArr[1];
        distRWindow[avIndex] = distArr[5];

        leftDist = 0;
        rightDist = 0;

        for (int i = 0; i < 3; i++) {
            leftDist += distLWindow[i];
            rightDist += distRWindow[i];
        }

        leftDist /= 3;
        rightDist /= 3;

        leftDist = estimate(leftDist, &distLKalman);
        rightDist = estimate(rightDist, &distRKalman);
        uint8_t rssiPos;

        //printk("Left Dist:  %f\r\n", leftDist);
        //printk("Right Dist:  %f\r\n", rightDist);

        if (pick_closest(leftDist, rightDist)) {
            rssiPos = (leftDist > 0.45);
        } else {
            rssiPos = 3 - (rightDist > 0.45);
        }
        //printk("RSSI Pos:   %d\r\n", rssiPos);

         // Pick the closest distance value (left node if 1)
/*         if (pick_closest(distArr[1], distArr[5] + 0.2)) {
            printk("Left node closer RSSI\r\n");
        } else {
            printk("Right node closer RSSI\r\n");
        } */

         multilateration_new(nodes, distArr, coords);
        multiY = coords[1];
        if (multiY > 4) {
			multiY = 4;
		} else if (multiY < 0) {
			multiY = 0;
		}

        rssiWindow[avIndex] = multiY;
        multiY = 0;

        for (int i = 0; i < 3; i++) {
            multiY += rssiWindow[i];
        }
        multiY /=3;

        //printk("Windowed:   %f\r\n", multiY);

        multiY = estimate(multiY, &rssiKalman);

        fuck.variable = (char*)k_malloc(sizeof(char) * 10);
			fuck.value = (char*)k_malloc(sizeof(char) * 15);
			fuck.unit = (char*)k_malloc(sizeof(char) * 4);

			// print int  to string
			//printk("Cumulative Distance:	%.3f\r\n", cumDist);
            sprintf(fuck.value, "%.3f", multiY);

			// Create the variable string for the JSON
            snprintf(fuck.variable, 10, "rssiY");
            
            // Set the units
            strcpy(fuck.unit, "m");
            
            // Encode the JSON
            json_obj_encode_buf(tagioObjDescriptor, 3, &fuck, jsonDataBuf, JSON_BUFFER_SIZE);
            
			// TODO uncomment this
            printk("%s\r\n", jsonDataBuf);
            memset(jsonDataBuf, 0, JSON_BUFFER_SIZE);

        //printk("Kalmanned:  %f\r\n", multiY);

        uint8_t multiPos = 0;

        // Round that bitch
        multiPos = (uint8_t)roundf(multiY);

        //printk("BLE y Coord:  %f\r\n", coords[1]);

        k_msgq_get(&leftQ, &leftVal, K_NO_WAIT);
        k_msgq_get(&rightQ, &rightVal, K_NO_WAIT);

        leftWindow[avIndex] = leftVal;
        rightWindow[avIndex] = rightVal;

        avIndex = (avIndex + 1) % 3;

        leftVal = 0;
        rightVal = 0;

        for (int i = 0; i < 3; i++) {
            leftVal += leftWindow[i];
            rightVal += rightWindow[i];
        }

        leftVal /= 3;
        rightVal /= 3;

        //printk("left estimate\r\n");
        leftVal = estimate(leftVal, &leftKalman);

        // print int  to string
        //printk("Cumulative Distance:	%.3f\r\n", cumDist);
        sprintf(fuck.value, "%.3f", leftVal);

        // Create the variable string for the JSON
        snprintf(fuck.variable, 10, "us-left");
        
        // Set the units
        strcpy(fuck.unit, "m");
        
        // Encode the JSON
        json_obj_encode_buf(tagioObjDescriptor, 3, &fuck, jsonDataBuf, JSON_BUFFER_SIZE);
        
        // TODO uncomment this
        printk("%s\r\n", jsonDataBuf);
        memset(jsonDataBuf, 0, JSON_BUFFER_SIZE);
        //printk("right estimate\r\n");
        rightVal = estimate(rightVal, &rightKalman);

        // print int  to string
        //printk("Cumulative Distance:	%.3f\r\n", cumDist);
        sprintf(fuck.value, "%.3f", rightVal);

        // Create the variable string for the JSON
        snprintf(fuck.variable, 10, "us-right");
        
        // Set the units
        strcpy(fuck.unit, "m");
        
        // Encode the JSON
        json_obj_encode_buf(tagioObjDescriptor, 3, &fuck, jsonDataBuf, JSON_BUFFER_SIZE);
        
        // TODO uncomment this
        printk("%s\r\n", jsonDataBuf);
        memset(jsonDataBuf, 0, JSON_BUFFER_SIZE);

        k_free(fuck.variable);
        k_free(fuck.unit);
        k_free(fuck.value);

        if (pick_closest(leftVal, rightVal)) {
            tempPos = (leftVal > 1);
        } else {
            tempPos = 3 - (rightVal > 1);
        }
        //printk("Ultrasonic Pos:    %d\r\n", tempPos);

        uint8_t fuckingPos = (uint8_t)roundf((tempPos + rssiPos) / 2);
        //printk("Average Pos: %d\r\n", fuckingPos);
 
        k_msgq_put(&playerPos, &fuckingPos, K_NO_WAIT);

        k_msgq_peek(&lifeQ, &lives);
        k_msgq_peek(&scoreQ, &score);
        k_msgq_peek(&comboQ, &combo);
        update_ad(lives, combo, score, 0);
        k_msleep(100);
    }
    return;
}

static void generate_square_wave(int16_t *buffer, size_t samples, float frequency, int16_t amplitude) {
    if (frequency == 0.0f) {
        // Generate silence
        memset(buffer, 0, samples * CHANNELS * BYTES_PER_SAMPLE);
        return;
    }

    static int toggle = 0;
    int samples_per_half_wave = SAMPLE_RATE / (2 * frequency);
    int count = 0;

    for (size_t i = 0; i < samples; i++) {
        if (count++ >= samples_per_half_wave) {
            toggle = !toggle;
            count = 0;
        }

        int16_t val = toggle ? amplitude : -amplitude;
        buffer[2 * i] = val;     // Left
        buffer[2 * i + 1] = val; // Right
    }
}

// Function to play one note at specified duration and amplitude
static void play_tone(struct tone note, int16_t amplitude) {
    uint32_t total_samples = (SAMPLE_RATE * note.duration_ms) / 1000;
    uint32_t blocks_needed = (total_samples + BLOCK_SAMPLES - 1) / BLOCK_SAMPLES;

    for (uint32_t i = 0; i < blocks_needed; i++) {
        void *block;
        if (k_mem_slab_alloc(&mem_slab, &block, K_FOREVER) == 0) {
            generate_square_wave(block, BLOCK_SAMPLES, note.frequency, amplitude);
            i2s_write(i2s_dev, block, BLOCK_SIZE);
        }
    }
}

/* Play a melody from an array of tones */
static void play_melody(const struct tone* melody, size_t len, int16_t amplitude) {
    for (size_t i = 0; i < len; i++) {
        play_tone(melody[i], amplitude);
    }
}

void speaker_thread(void) {
    printk("I2S melody playback\n");

    i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s0));
    if (!device_is_ready(i2s_dev)) {
        printk("I2S device not ready\n");
        return;
    }

    struct i2s_config config = {
        .word_size = BITS_PER_SAMPLE,
        .channels = CHANNELS,
        .format = I2S_FMT_DATA_FORMAT_I2S,
        .options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
        .frame_clk_freq = SAMPLE_RATE,
        .block_size = BLOCK_SIZE,
        .mem_slab = &mem_slab,
        .timeout = 1000,
    };

    if (i2s_configure(i2s_dev, I2S_DIR_TX, &config) < 0) {
        printk("Failed to configure I2S TX\n");
        return;
    }

    if (i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START) < 0) {
        printk("Failed to start I2S TX\n");
        return;
    }

    while (1) {
        play_melody(imperial_march, ARRAY_SIZE(imperial_march), 500);
        k_msleep(10);
    }
    return;
}

void tx_thread() {
    uint8_t state = 0;
    uint8_t lives = 0;
    uint8_t score = 0;
    uint8_t combo = 0;

    int err;

    /* Initialize the Bluetooth Subsystem */
    bt_id_create(&address, NULL);
    err = bt_enable(bt_ready);

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);	
		return;
	}

	printk("Bluetooth initialized\n");

	start_scan();

    while (1) {
        k_msgq_peek(&lifeQ, &lives);
        k_msgq_peek(&scoreQ, &score);
        k_msgq_peek(&comboQ, &combo);
        update_ad(lives, combo, score, 0);
        k_msleep(10);
    }
}