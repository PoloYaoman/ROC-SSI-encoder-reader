#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>         // For memset
#include "freertos/queue.h" // For rmt_rx_done_event_data_t if using event queue

#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "roc_encoder.h"

#define DATA_PIN    25
#define CLOCK_PIN   26

#define B_PIN   32
#define G_PIN   33

#define BIT_COUNT               13
#define ENCODER_RESOLUTION_HZ   100000

#define RMT_RX_TIMEOUT_US    1000000   // 1 second RX timeout


static const char *TAG = "example";


// Buffer to store received RMT symbols
static rmt_symbol_word_t rx_symbols_buffer[BIT_COUNT]; // Max symbols channel can hold at once


static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    // return whether any task is woken up
    return high_task_wakeup == pdTRUE;
}

int grayToBinary(int n) {
    int b = 0;
    // Traverse all bits of Gray code
    while (n > 0) {
        // Build binary number step by step using XOR
        b ^= n;
        // Move to next bit
        n = n >> 1;
    }
    return b;
}


void app_main(void) {
    /* clock config */
    ESP_LOGI(TAG, "Initialize CLOCK RMT TX");
    rmt_channel_handle_t tx_clock = NULL;
    rmt_tx_channel_config_t tx_clock_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,   // select source clock
        .gpio_num = CLOCK_PIN,            // GPIO number
        .mem_block_symbols = 64,          // memory block size, 64 * 4 = 256 Bytes
        .resolution_hz = 1000000,
        .trans_queue_depth = 4,           // set the number of transactions that can pend in the background
        .flags.invert_out = false,        // do not invert output signal
        .flags.with_dma = false,          // do not need DMA backend
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_clock_config, &tx_clock));

    ESP_LOGI(TAG, "Set CLOCK RMT TX config");
    encoder_clock_config_t encoder_clock_config = {
        .resolution = ENCODER_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t encoder_clock = NULL;
    ESP_ERROR_CHECK(rmt_new_encoder_clock(&encoder_clock_config, &encoder_clock));

    rmt_carrier_config_t tx_carrier_cfg = {
        .duty_cycle = 0.5,                 // duty cycle 50%
        .frequency_hz = ENCODER_RESOLUTION_HZ,
        .flags.polarity_active_low = false, // carrier should be modulated to high level
    };
    // modulate carrier to TX channel
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_clock, &tx_carrier_cfg));

    rmt_transmit_config_t tx_config = {
        .loop_count = 1,
    };

    // enable the channels
    ESP_LOGI(TAG, "enable CLOCK RMT TX");
    ESP_ERROR_CHECK(rmt_enable(tx_clock));

    const static uint32_t clock_hz = 1;
    /* ------------ */


    /* data config */
    ESP_LOGI(TAG, "Create DATA RMT RX");
    rmt_rx_channel_config_t rx_data_config = {
        .gpio_num = DATA_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1200000,
        .mem_block_symbols = 64,
    };
    rmt_channel_handle_t rx_data = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_data_config, &rx_data));

    ESP_ERROR_CHECK(rmt_enable(rx_data));

    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_data, &cbs, receive_queue));

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 3187,
        .signal_range_max_ns = 12000*13
    };

    ESP_LOGI(TAG, "Starting RMT reception. Apply signal to GPIO %d", DATA_PIN);


    while(1) {
        ESP_ERROR_CHECK(rmt_transmit(tx_clock, encoder_clock, &clock_hz, sizeof(clock_hz), &tx_config));

        // Clear buffer before next reception
        memset(rx_symbols_buffer, 0, sizeof(rx_symbols_buffer));

        // Start reception
        // This call is blocking until data is received or timeout occurs.
        esp_err_t ret = rmt_receive(rx_data, rx_symbols_buffer, sizeof(rx_symbols_buffer), &receive_config);

        rmt_rx_done_event_data_t rx_data;
        xQueueReceive(receive_queue, &rx_data, portMAX_DELAY);

        ESP_LOGI(TAG, "Receiving...");
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Received symbols:");
            char word[12] = "";
            int bword = 0b0;
            int sig_len = 0;
            for (size_t i = 0; i < sizeof(rx_symbols_buffer) / sizeof(rmt_symbol_word_t); i++) {
                if (sig_len<13) {
                    for(int D0=rx_symbols_buffer[i].duration0; D0>0; D0-=12) {
                        strcat(word,"0");
                        bword = (bword << 1) | 0;
                        sig_len++;
                    }
                    for(int D1=rx_symbols_buffer[i].duration1; D1>0; D1-=12){
                        strcat(word,"1");
                        bword = (bword << 1) | 1;
                        sig_len++;
                    }
                }
            }
            bword = grayToBinary(bword);
            ESP_LOGI(TAG, "word : %s \t result : %d", word, bword);
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "RMT RX Timeout");
        } else {
            ESP_LOGE(TAG, "RMT RX Error: %s", esp_err_to_name(ret));
            break;
        }
        // Add a small delay to allow logging and prevent tight loop on continuous signal
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
