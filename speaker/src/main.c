#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>

#include "tones.h"
#include "imperial_march.h"

LOG_MODULE_REGISTER(i2s_playback, LOG_LEVEL_DBG);

/* Audio settings */
#define SAMPLE_RATE         44100
#define CHANNELS            2
#define BITS_PER_SAMPLE     16
#define BYTES_PER_SAMPLE    (BITS_PER_SAMPLE / 8)
#define BLOCK_SAMPLES       256
#define BLOCK_SIZE          (BLOCK_SAMPLES * CHANNELS * BYTES_PER_SAMPLE)

K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, 4, 4);

static const struct device *i2s_dev;

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

int main(void) {
    printk("I2S melody playback\n");

    i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s0));
    if (!device_is_ready(i2s_dev)) {
        printk("I2S device not ready\n");
        return -1;
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
        return -1;
    }

    if (i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START) < 0) {
        printk("Failed to start I2S TX\n");
        return -1;
    }

    printk("Playing melody...\n");

    play_melody(imperial_march, ARRAY_SIZE(imperial_march), 1000);

    struct tone test = {NOTE_F4, 350};

    return 0;
}
