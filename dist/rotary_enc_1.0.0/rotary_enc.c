#include "rotary_enc.h"

#include <inttypes.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "rotary_enc";

#define ROTARY_ENCODER_PCNT_HIGH_LIMIT 30000
#define ROTARY_ENCODER_PCNT_LOW_LIMIT (-30000)
#define ROTARY_ENCODER_GLITCH_FILTER_NS 10000

struct rotary_encoder_t {
    int pin_a;
    int pin_b;
    int32_t min_val;
    int32_t max_val;
    int32_t value;
    int32_t last_read_value;
    int last_pcnt_count;
    int64_t sub_count;
    uint8_t steps_per_count;
    bool rollover;
    bool unit_enabled;
    bool unit_started;
    pcnt_unit_handle_t pcnt_unit;
    pcnt_channel_handle_t pcnt_chan_a;
    pcnt_channel_handle_t pcnt_chan_b;
    SemaphoreHandle_t lock;
};

static int32_t clamp_position(int64_t value, int32_t min_val, int32_t max_val)
{
    if (value > max_val) {
        return max_val;
    }
    if (value < min_val) {
        return min_val;
    }
    return (int32_t)value;
}

static esp_err_t release_resources(rotary_encoder_handle_t dev)
{
    esp_err_t err;

    if (dev->unit_started) {
        err = pcnt_unit_stop(dev->pcnt_unit);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Gagal menghentikan PCNT: %s", esp_err_to_name(err));
            return err;
        }
        dev->unit_started = false;
    }

    if (dev->unit_enabled) {
        err = pcnt_unit_disable(dev->pcnt_unit);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Gagal menonaktifkan PCNT: %s", esp_err_to_name(err));
            return err;
        }
        dev->unit_enabled = false;
    }

    if (dev->pcnt_chan_a != NULL) {
        err = pcnt_del_channel(dev->pcnt_chan_a);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Gagal menghapus channel PCNT A: %s", esp_err_to_name(err));
            return err;
        }
        dev->pcnt_chan_a = NULL;
    }

    if (dev->pcnt_chan_b != NULL) {
        err = pcnt_del_channel(dev->pcnt_chan_b);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Gagal menghapus channel PCNT B: %s", esp_err_to_name(err));
            return err;
        }
        dev->pcnt_chan_b = NULL;
    }

    if (dev->pcnt_unit != NULL) {
        err = pcnt_del_unit(dev->pcnt_unit);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Gagal menghapus unit PCNT: %s", esp_err_to_name(err));
            return err;
        }
        dev->pcnt_unit = NULL;
    }

    return ESP_OK;
}

static void update_position(rotary_encoder_handle_t dev)
{
    int raw_count = 0;
    esp_err_t err = pcnt_unit_get_count(dev->pcnt_unit, &raw_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal membaca counter PCNT: %s", esp_err_to_name(err));
        return;
    }

    int64_t raw_delta = (int64_t)raw_count - dev->last_pcnt_count;
    if (raw_delta > INT32_MAX) {
        raw_delta -= (int64_t)UINT32_MAX + 1;
    } else if (raw_delta < INT32_MIN) {
        raw_delta += (int64_t)UINT32_MAX + 1;
    }
    dev->last_pcnt_count = raw_count;
    if (raw_delta == 0) {
        return;
    }

    dev->sub_count += raw_delta;
    int64_t val_diff = dev->sub_count / dev->steps_per_count;
    if (val_diff == 0) {
        return;
    }
    dev->sub_count -= val_diff * dev->steps_per_count;

    if (!dev->rollover) {
        dev->value = clamp_position((int64_t)dev->value + val_diff,
                                    dev->min_val,
                                    dev->max_val);
        return;
    }

    int64_t range = (int64_t)dev->max_val - dev->min_val + 1;
    int64_t offset = ((int64_t)dev->value - dev->min_val + val_diff) % range;
    if (offset < 0) {
        offset += range;
    }
    dev->value = (int32_t)((int64_t)dev->min_val + offset);
}

esp_err_t rotary_encoder_init(const rotary_encoder_config_t *config,
                              rotary_encoder_handle_t *out_handle)
{
    if (out_handle == NULL) {
        ESP_LOGE(TAG, "Handle keluaran tidak valid");
        return ESP_ERR_INVALID_ARG;
    }
    *out_handle = NULL;

    if (config == NULL ||
        !GPIO_IS_VALID_GPIO(config->pin_a) ||
        !GPIO_IS_VALID_GPIO(config->pin_b) ||
        config->pin_a == config->pin_b ||
        config->min_val > config->max_val) {
        ESP_LOGE(TAG, "Konfigurasi pin atau batas posisi tidak valid");
        return ESP_ERR_INVALID_ARG;
    }

    rotary_encoder_handle_t dev = calloc(1, sizeof(*dev));
    if (dev == NULL) {
        ESP_LOGE(TAG, "Gagal mengalokasikan handle encoder");
        return ESP_ERR_NO_MEM;
    }

    dev->pin_a = config->pin_a;
    dev->pin_b = config->pin_b;
    dev->min_val = config->min_val;
    dev->max_val = config->max_val;
    dev->value = clamp_position(config->initial_val, dev->min_val, dev->max_val);
    dev->last_read_value = dev->value;
    dev->rollover = config->rollover;
    dev->steps_per_count = config->steps_per_count == 0 ? 4 : config->steps_per_count;
    dev->lock = xSemaphoreCreateMutex();
    if (dev->lock == NULL) {
        ESP_LOGE(TAG, "Gagal membuat mutex encoder");
        free(dev);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = gpio_pullup_en(dev->pin_a);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal mengaktifkan pull-up GPIO %d: %s",
                 dev->pin_a, esp_err_to_name(err));
        goto fail;
    }
    err = gpio_pullup_en(dev->pin_b);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal mengaktifkan pull-up GPIO %d: %s",
                 dev->pin_b, esp_err_to_name(err));
        goto fail;
    }

    const pcnt_unit_config_t unit_config = {
        .high_limit = ROTARY_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ROTARY_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = 1,
    };
    err = pcnt_new_unit(&unit_config, &dev->pcnt_unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal membuat unit PCNT: %s", esp_err_to_name(err));
        goto fail;
    }

    const pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = ROTARY_ENCODER_GLITCH_FILTER_NS,
    };
    err = pcnt_unit_set_glitch_filter(dev->pcnt_unit, &filter_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal mengatur filter glitch PCNT: %s", esp_err_to_name(err));
        goto fail;
    }

    const pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = dev->pin_a,
        .level_gpio_num = dev->pin_b,
    };
    err = pcnt_new_channel(dev->pcnt_unit, &chan_a_config, &dev->pcnt_chan_a);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal membuat channel PCNT A: %s", esp_err_to_name(err));
        goto fail;
    }

    const pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = dev->pin_b,
        .level_gpio_num = dev->pin_a,
    };
    err = pcnt_new_channel(dev->pcnt_unit, &chan_b_config, &dev->pcnt_chan_b);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal membuat channel PCNT B: %s", esp_err_to_name(err));
        goto fail;
    }

    err = pcnt_channel_set_edge_action(dev->pcnt_chan_a,
                                       PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                       PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    if (err == ESP_OK) {
        err = pcnt_channel_set_level_action(dev->pcnt_chan_a,
                                            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    }
    if (err == ESP_OK) {
        err = pcnt_channel_set_edge_action(dev->pcnt_chan_b,
                                           PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                           PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    }
    if (err == ESP_OK) {
        err = pcnt_channel_set_level_action(dev->pcnt_chan_b,
                                            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal mengatur aksi quadrature PCNT: %s", esp_err_to_name(err));
        goto fail;
    }

    err = pcnt_unit_enable(dev->pcnt_unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal mengaktifkan unit PCNT: %s", esp_err_to_name(err));
        goto fail;
    }
    dev->unit_enabled = true;

    err = pcnt_unit_clear_count(dev->pcnt_unit);
    if (err == ESP_OK) {
        err = pcnt_unit_start(dev->pcnt_unit);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal menjalankan unit PCNT: %s", esp_err_to_name(err));
        goto fail;
    }
    dev->unit_started = true;

    *out_handle = dev;
    ESP_LOGI(TAG, "Encoder aktif pada GPIO A=%d, B=%d", dev->pin_a, dev->pin_b);
    return ESP_OK;

fail:
    if (release_resources(dev) != ESP_OK) {
        ESP_LOGE(TAG, "Sebagian resource PCNT tidak dapat dilepaskan saat rollback");
    }
    vSemaphoreDelete(dev->lock);
    free(dev);
    return err;
}

int32_t rotary_encoder_get_position(rotary_encoder_handle_t handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Handle encoder tidak valid");
        return 0;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    update_position(handle);
    int32_t value = handle->value;
    xSemaphoreGive(handle->lock);
    return value;
}

void rotary_encoder_set_position(rotary_encoder_handle_t handle, int32_t position)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Handle encoder tidak valid");
        return;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    esp_err_t err = pcnt_unit_clear_count(handle->pcnt_unit);
    if (err != ESP_OK) {
        xSemaphoreGive(handle->lock);
        ESP_LOGE(TAG, "Gagal menghapus counter PCNT saat mengatur posisi: %s",
                 esp_err_to_name(err));
        return;
    }

    handle->value = clamp_position(position, handle->min_val, handle->max_val);
    handle->last_read_value = handle->value;
    handle->sub_count = 0;
    handle->last_pcnt_count = 0;
    int32_t value = handle->value;
    xSemaphoreGive(handle->lock);

    ESP_LOGD(TAG, "Posisi encoder diatur ke %" PRId32, value);
}

int32_t rotary_encoder_get_delta(rotary_encoder_handle_t handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Handle encoder tidak valid");
        return 0;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    update_position(handle);

    int64_t delta = (int64_t)handle->value - handle->last_read_value;
    if (handle->rollover) {
        int64_t range = (int64_t)handle->max_val - handle->min_val + 1;
        if (delta > range / 2) {
            delta -= range;
        } else if (delta < -(range / 2)) {
            delta += range;
        }
    }

    handle->last_read_value = handle->value;
    if (delta > INT32_MAX) {
        delta = INT32_MAX;
    } else if (delta < INT32_MIN) {
        delta = INT32_MIN;
    }
    xSemaphoreGive(handle->lock);
    return (int32_t)delta;
}

void rotary_encoder_set_limits(rotary_encoder_handle_t handle,
                               int32_t min_val,
                               int32_t max_val)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Handle encoder tidak valid");
        return;
    }
    if (min_val > max_val) {
        ESP_LOGE(TAG, "Batas posisi tidak valid: min=%" PRId32 ", max=%" PRId32,
                 min_val, max_val);
        return;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    update_position(handle);
    handle->min_val = min_val;
    handle->max_val = max_val;
    handle->value = clamp_position(handle->value, min_val, max_val);
    xSemaphoreGive(handle->lock);
}

esp_err_t rotary_encoder_deinit(rotary_encoder_handle_t handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Handle encoder tidak valid");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = release_resources(handle);
    if (err != ESP_OK) {
        return err;
    }

    vSemaphoreDelete(handle->lock);
    free(handle);
    ESP_LOGI(TAG, "Resource encoder berhasil dilepaskan");
    return ESP_OK;
}
