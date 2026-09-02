#include <stdint.h>
#include "driver/rmt_encoder.h"

typedef struct {
    uint32_t resolution; // Encoder resolution, in Hz
} encoder_clock_config_t;

esp_err_t rmt_new_encoder_clock(const encoder_clock_config_t *config, rmt_encoder_handle_t *ret_encoder);
