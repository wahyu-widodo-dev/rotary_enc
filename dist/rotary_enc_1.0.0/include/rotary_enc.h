#ifndef ROTARY_ENC_H
#define ROTARY_ENC_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Konfigurasi rotary encoder berbasis PCNT.
 */
typedef struct {
    int pin_a;                  /*!< GPIO untuk sinyal A/CLK. */
    int pin_b;                  /*!< GPIO untuk sinyal B/DT; harus berbeda dari pin_a. */
    int32_t min_val;            /*!< Batas posisi minimum (inklusif). */
    int32_t max_val;            /*!< Batas posisi maksimum (inklusif). */
    int32_t initial_val;        /*!< Posisi awal; dijepit ke batas yang diberikan. */
    bool rollover;              /*!< Jika true, posisi berputar di antara batas. */
    uint8_t steps_per_count;    /*!< Transisi quadrature per langkah; 0 menggunakan default 4. */
} rotary_encoder_config_t;

/** Handle buram; buat dan lepaskan melalui fungsi lifecycle rotary encoder. */
typedef struct rotary_encoder_t *rotary_encoder_handle_t;

/**
 * @brief Membuat encoder PCNT dan mengonfigurasi GPIO serta pull-up internal.
 *
 * Arah kenaikan posisi bergantung pada urutan kabel A/B. Jika arah tidak
 * sesuai, tukar kedua sinyal.
 *
 * @param[in] config Konfigurasi pin, batas, dan jumlah transisi per langkah.
 * @param[out] out_handle Menerima handle jika pembuatan berhasil.
 * @return ESP_OK jika berhasil; kode error ESP-IDF jika parameter, alokasi,
 *         atau konfigurasi peripheral gagal.
 */
esp_err_t rotary_encoder_init(const rotary_encoder_config_t *config,
                              rotary_encoder_handle_t *out_handle);

/**
 * @brief Membaca posisi terbaru, termasuk hitungan PCNT yang belum diproses.
 *
 * Fungsi pembacaan dan pengaturan handle yang sama diserialisasi dan hanya
 * boleh dipanggil dari task, bukan ISR.
 *
 * @param[in] handle Handle encoder.
 * @return Posisi saat ini; 0 jika handle NULL.
 */
int32_t rotary_encoder_get_position(rotary_encoder_handle_t handle);

/** @brief Mengatur posisi dan membuang transisi encoder yang tertunda. */
void rotary_encoder_set_position(rotary_encoder_handle_t handle, int32_t position);

/**
 * @brief Mengambil delta posisi sejak pemanggilan sebelumnya.
 *
 * Saat rollover aktif, hasil memakai perubahan terpendek melintasi batas.
 *
 * @param[in] handle Handle encoder.
 * @return Delta posisi; 0 jika handle NULL.
 */
int32_t rotary_encoder_get_delta(rotary_encoder_handle_t handle);

/**
 * @brief Mengubah batas posisi; nilai saat ini dijepit ke rentang baru.
 *
 * Batas yang tidak valid (min > max) ditolak dan dilaporkan melalui log.
 */
void rotary_encoder_set_limits(rotary_encoder_handle_t handle,
                               int32_t min_val,
                               int32_t max_val);

/**
 * @brief Menghentikan PCNT dan melepaskan resource encoder.
 *
 * Hentikan seluruh penggunaan handle dari task lain sebelum memanggil fungsi ini.
 */
esp_err_t rotary_encoder_deinit(rotary_encoder_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif
