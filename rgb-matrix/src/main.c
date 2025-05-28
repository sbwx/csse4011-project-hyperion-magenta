#include <errno.h>
#include <string.h>

#define LOG_LEVEL 4
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>
#include <zephyr/random/random.h>

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

void clear_pixels(uint8_t pos)
{
    memset(&(pixels[pos]), 0x00, sizeof(struct led_rgb) * 2);
}

void clear_block_top(uint8_t x, uint8_t y)
{
    if (y % 2 != 0)
        clear_pixels(255 + (2 * x) - (8 * (y + 2)) - 6 - 1);
    else
        clear_pixels(255 - (2 * x) - (8 * y) - 16 - 1);
}

void set_player_position(uint8_t x)
{

    // Remove the 2 pixels for the bottom row
    memset(&(pixels[255 - (2 * x) - 1]), 0x00, sizeof(struct led_rgb) * 2);
    
    // Remove the 2 pixels for the top row
    memset(&(pixels[255 - 14 + (2 * x) - 1]), 0x00, sizeof(struct led_rgb) * 2);
    
    memcpy(&(pixels[255 - (2 * x)]), &colors[4], sizeof(struct led_rgb));
    memcpy(&(pixels[255 - (2 * x) - 1]), &colors[4], sizeof(struct led_rgb));
    memcpy(&(pixels[255 - 14 + (2 * x)]), &colors[4], sizeof(struct led_rgb));
    memcpy(&(pixels[255 - 14 + (2 * x) - 1]), &colors[4], sizeof(struct led_rgb));

}

void set_note(uint8_t x, uint8_t y)
{
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

void set_bottom_pixels(const struct led_rgb* color)
{
    // Clear the bottom 4 rows
    memset(&(pixels[208]), 0x00, sizeof(struct led_rgb) * 48);
    for (uint8_t i = 0; i < 8; i++)
        memcpy(&(pixels[208 + i]), color, sizeof(struct led_rgb));
    
    uint8_t pos = 0;
    k_msgq_get(&playerPos, &pos, K_NO_WAIT);

    set_player_position(pos);
}


int main(void)
{
	size_t color = 0;
	int rc;

	if (!device_is_ready(strip)) {
		return 0;
	}


    while (1) {
        Block* latestBlock;
        uint8_t numBlocks = k_msgq_num_used_get(&blockQ);
        
        if (!k_msgq_peek_at(&blockQ, &latestBlock, numBlocks - 1) && numBlocks != 0) {
        
            if (latestBlock->y < 27) {
                
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
                k_free(currBlock);
                
            }
        }


        for (int i = 0; i < 8; i++) {

            Block* currBlock;

            if (!k_msgq_peek_at(&blockQ, &currBlock, i)) {
    
                set_note(currBlock->x, currBlock->y);
                clear_block_top(currBlock->x, currBlock->y); 
                
                set_bottom_pixels(&colors[3]);
                
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
        k_msleep(50);
    }

	return 0;
}
