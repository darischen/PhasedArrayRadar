/*
 * HB100 Phased Array Radar - ESP32-S3 Data Collector
 * 
 * Reads 4 HB100 IF outputs via ADC1 DMA at 10 kHz per channel.
 * Sends raw sample blocks over USB-CDC serial to host (Raspberry Pi 4).
 *
 * Hardware:
 *   HB100 #0 (farthest)  -> GPIO 7 (ADC1_CH6)
 *   HB100 #1             -> GPIO 6 (ADC1_CH5)
 *   HB100 #2             -> GPIO 5 (ADC1_CH4)
 *   HB100 #3 (closest)   -> GPIO 4 (ADC1_CH3)
 *   All HB100 GND pins   -> ESP32-S3 GND
 *   All HB100 5V pins    -> 5V supply
 *
 * Protocol:
 *   Each block: 4-byte sync (0xDEADBEEF) + 4-byte sample_count +
 *               (sample_count * 4 channels * 2 bytes) raw ADC data
 *   Channel order per sample: ch0, ch1, ch2, ch3
 *   ADC resolution: 12-bit (0-4095)
 *   Sample rate: 10,000 Hz per channel
 *   Block size: 1024 samples (~102.4 ms per block)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"

static const char *TAG = "RADAR";

/* --------------- Configuration --------------- */
#define NUM_CHANNELS      4
#define SAMPLES_PER_BLOCK 1024
#define SAMPLE_RATE_HZ    10000

/* ADC1 channel assignments (ESP32-S3 GPIO -> ADC1 channel mapping) */
/* GPIO 4 = ADC1_CH3, GPIO 5 = ADC1_CH4, GPIO 6 = ADC1_CH5, GPIO 7 = ADC1_CH6 */
static const adc_channel_t adc_channels[NUM_CHANNELS] = {
    ADC_CHANNEL_6,  /* ch0: GPIO 7, farthest left HB100 */
    ADC_CHANNEL_5,  /* ch1: GPIO 6 */
    ADC_CHANNEL_4,  /* ch2: GPIO 5 */
    ADC_CHANNEL_3,  /* ch3: GPIO 4, closest HB100 */
};

/*
 * DMA buffer sizing:
 * Each ADC conversion result is 4 bytes (adc_digi_output_data_t on S3).
 * We need SAMPLES_PER_BLOCK * NUM_CHANNELS conversions per block.
 * The DMA reads continuously; we pull from it in chunks.
 */
#define CONV_FRAME_SIZE   (NUM_CHANNELS * SOC_ADC_DIGI_RESULT_BYTES)
#define DMA_BUF_SIZE      (CONV_FRAME_SIZE * 64)

/* Sync word for host to find block boundaries */
static const uint8_t SYNC_WORD[4] = {0xDE, 0xAD, 0xBE, 0xEF};

/* --------------- Globals --------------- */
static adc_continuous_handle_t adc_handle = NULL;
static uint16_t sample_buf[SAMPLES_PER_BLOCK][NUM_CHANNELS];
static volatile bool data_ready = false;

/* --------------- ADC DMA Setup --------------- */
static bool IRAM_ATTR adc_conv_done_cb(adc_continuous_handle_t handle,
                                        const adc_continuous_evt_data_t *edata,
                                        void *user_data)
{
    /* Signal from ISR that a conversion frame is ready */
    return false; /* No high-priority task woken */
}

static esp_err_t adc_dma_init(void)
{
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = DMA_BUF_SIZE * 4,
        .conv_frame_size    = DMA_BUF_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_digi_pattern_config_t adc_pattern[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
        adc_pattern[i].atten     = ADC_ATTEN_DB_12;   /* Full 0-3.3V range */
        adc_pattern[i].channel   = adc_channels[i];
        adc_pattern[i].unit      = ADC_UNIT_1;
        adc_pattern[i].bit_width = ADC_BITWIDTH_12;
    }

    adc_continuous_config_t dig_cfg = {
        .pattern_num    = NUM_CHANNELS,
        .adc_pattern    = adc_pattern,
        .sample_freq_hz = SAMPLE_RATE_HZ * NUM_CHANNELS, /* Aggregate rate */
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,   /* ESP32-S3 format */
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));

    return ESP_OK;
}

/* --------------- Data Collection Task --------------- */

/*
 * Reads raw ADC DMA output and sorts samples into the channel-ordered buffer.
 * The ESP32-S3 ADC DMA output format (TYPE2) packs channel ID + value
 * into each result word. We sort by channel to get aligned multi-channel data.
 */
static void collect_block(void)
{
    uint32_t ret_num = 0;
    uint8_t raw_buf[DMA_BUF_SIZE];
    int sample_idx[NUM_CHANNELS] = {0};
    int total_complete = 0;

    /* Clear buffer */
    memset(sample_buf, 0, sizeof(sample_buf));

    while (total_complete < SAMPLES_PER_BLOCK) {
        esp_err_t ret = adc_continuous_read(adc_handle, raw_buf,
                                             DMA_BUF_SIZE, &ret_num, 100);
        if (ret != ESP_OK) {
            continue;
        }

        /* Parse DMA results */
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&raw_buf[i];

            /* TYPE2 format on ESP32-S3: unit + channel + data fields */
            int chan_raw = p->type2.channel;
            uint16_t val = p->type2.data;

            /* Map ADC channel number back to our 0-3 index */
            int ch_idx = -1;
            for (int c = 0; c < NUM_CHANNELS; c++) {
                if (adc_channels[c] == chan_raw) {
                    ch_idx = c;
                    break;
                }
            }

            if (ch_idx < 0 || sample_idx[ch_idx] >= SAMPLES_PER_BLOCK) {
                continue;
            }

            sample_buf[sample_idx[ch_idx]][ch_idx] = val;
            sample_idx[ch_idx]++;

            /* Track completion by the slowest channel */
            int min_samples = SAMPLES_PER_BLOCK;
            for (int c = 0; c < NUM_CHANNELS; c++) {
                if (sample_idx[c] < min_samples) {
                    min_samples = sample_idx[c];
                }
            }
            total_complete = min_samples;
        }
    }
}

/* --------------- USB Serial Output --------------- */

static void send_block_usb(void)
{
    uint32_t count = SAMPLES_PER_BLOCK;

    /* Sync word */
    usb_serial_jtag_write_bytes(SYNC_WORD, 4, portMAX_DELAY);

    /* Sample count (little-endian) */
    usb_serial_jtag_write_bytes((const uint8_t *)&count, 4, portMAX_DELAY);

    /* Raw sample data: SAMPLES_PER_BLOCK rows of 4 x uint16_t */
    usb_serial_jtag_write_bytes((const uint8_t *)sample_buf,
                                 SAMPLES_PER_BLOCK * NUM_CHANNELS * sizeof(uint16_t),
                                 portMAX_DELAY);
}

/* --------------- Main Task --------------- */

static void radar_task(void *arg)
{
    ESP_LOGI(TAG, "Starting ADC DMA...");
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    /* Let ADC stabilize */
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Collecting data. Block size: %d samples, %d channels, %d Hz",
             SAMPLES_PER_BLOCK, NUM_CHANNELS, SAMPLE_RATE_HZ);

    while (1) {
        collect_block();
        send_block_usb();
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "HB100 Phased Array Radar Collector");
    ESP_LOGI(TAG, "Channels: GPIO7(ch0) GPIO6(ch1) GPIO5(ch2) GPIO4(ch3)");

    /* Configure USB-CDC for output */
    usb_serial_jtag_driver_config_t usb_cfg = {
        .tx_buffer_size = 16384,
        .rx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    /* Initialize ADC with DMA */
    ESP_ERROR_CHECK(adc_dma_init());

    /* Start collection task on core 1 (core 0 handles USB) */
    xTaskCreatePinnedToCore(radar_task, "radar", 8192, NULL, 5, NULL, 1);
}
