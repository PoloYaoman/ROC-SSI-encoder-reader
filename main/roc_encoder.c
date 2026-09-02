#include "esp_check.h"
#include "roc_encoder.h"

static const char *TAG = "ROC encoder";


typedef struct {
    rmt_encoder_t base;
    rmt_encoder_handle_t copy_encoder;
    uint32_t resolution;
} rmt_stepper_uniform_encoder_t;

static esp_err_t rmt_del_stepper_motor_uniform_encoder(rmt_encoder_t *encoder) {
    rmt_stepper_uniform_encoder_t *motor_encoder = __containerof(encoder, rmt_stepper_uniform_encoder_t, base);
    rmt_del_encoder(motor_encoder->copy_encoder);
    free(motor_encoder);
    return ESP_OK;
}

static size_t rmt_encode_stepper_motor_uniform(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state) {
    rmt_stepper_uniform_encoder_t *motor_encoder = __containerof(encoder, rmt_stepper_uniform_encoder_t, base);
    rmt_encoder_handle_t copy_encoder = motor_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    uint32_t target_freq_hz = *(uint32_t *)primary_data;
    uint32_t symbol_duration = motor_encoder->resolution / target_freq_hz / 2;
    rmt_symbol_word_t freq_sample = {
        .level0 = 0,
        .duration0 = symbol_duration,
        .level1 = 1,
        .duration1 = symbol_duration,
    };
    size_t encoded_symbols = copy_encoder->encode(copy_encoder, channel, &freq_sample, sizeof(freq_sample), &session_state);
    *ret_state = session_state;
    return encoded_symbols;
}

static esp_err_t rmt_reset_stepper_motor_uniform(rmt_encoder_t *encoder) {
    rmt_stepper_uniform_encoder_t *motor_encoder = __containerof(encoder, rmt_stepper_uniform_encoder_t, base);
    rmt_encoder_reset(motor_encoder->copy_encoder);
    return ESP_OK;
}

esp_err_t rmt_new_encoder_clock(const encoder_clock_config_t *config, rmt_encoder_handle_t *ret_encoder) {
    esp_err_t ret = ESP_OK;
    rmt_stepper_uniform_encoder_t *step_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid arguments");
    step_encoder = rmt_alloc_encoder_mem(sizeof(rmt_stepper_uniform_encoder_t));
    ESP_GOTO_ON_FALSE(step_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for stepper uniform encoder");
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &step_encoder->copy_encoder), err, TAG, "create copy encoder failed");

    step_encoder->resolution = config->resolution;
    step_encoder->base.del = rmt_del_stepper_motor_uniform_encoder;
    step_encoder->base.encode = rmt_encode_stepper_motor_uniform;
    step_encoder->base.reset = rmt_reset_stepper_motor_uniform;
    *ret_encoder = &(step_encoder->base);
    return ESP_OK;
err:
    if (step_encoder) {
        if (step_encoder->copy_encoder) {
            rmt_del_encoder(step_encoder->copy_encoder);
        }
        free(step_encoder);
    }
    return ret;
}
