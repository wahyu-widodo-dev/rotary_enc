# Komponen Rotary Encoder

Driver rotary encoder quadrature berbasis peripheral PCNT untuk ESP-IDF 5.3.0
atau lebih baru. Komponen menghitung transisi menggunakan hardware, mengubahnya
menjadi posisi encoder, lalu menyediakan pembacaan posisi dan delta dari task
aplikasi.

## Fitur

- Pembacaan quadrature A/B dengan PCNT.
- Filter glitch hardware 10 us.
- Posisi awal dan batas minimum/maksimum yang dapat diatur.
- Mode rollover atau penjepitan nilai pada batas.
- Pengaturan resolusi melalui jumlah transisi per langkah.
- Sinkronisasi untuk operasi baca/tulis dari beberapa task.

## Koneksi perangkat keras

| Rotary encoder | ESP32 (contoh) |
|---|---|
| A / CLK | GPIO yang mendukung input |
| B / DT | GPIO berbeda yang mendukung input |
| Common | GND |

Komponen mengaktifkan pull-up internal pada kedua GPIO. Pull-up internal
bersifat lemah; gunakan pull-up eksternal jika diperlukan oleh encoder, board,
atau panjang kabel. Kedua sinyal harus kompatibel dengan tegangan GPIO SoC.
GPIO A dan B tidak boleh digunakan oleh driver lain selama encoder aktif.

Jika arah posisi terbalik dari yang diinginkan, tukar sambungan A dan B.

## Membuat dan menggunakan encoder

```c
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rotary_enc.h"

static const char *TAG = "encoder_example";

void app_main(void)
{
    const rotary_encoder_config_t config = {
        .pin_a = GPIO_NUM_18,
        .pin_b = GPIO_NUM_19,
        .min_val = 0,
        .max_val = 100,
        .initial_val = 0,
        .rollover = false,
        .steps_per_count = 4,
    };

    rotary_encoder_handle_t encoder = NULL;
    ESP_ERROR_CHECK(rotary_encoder_init(&config, &encoder));

    while (true) {
        int32_t change = rotary_encoder_get_delta(encoder);
        if (change != 0) {
            ESP_LOGI(TAG, "Posisi: %" PRId32, rotary_encoder_get_position(encoder));
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
```

Contoh pin hanya ilustrasi; pastikan GPIO tersedia dan sesuai untuk board/SoC.
Semua fungsi API dijalankan dari task, bukan ISR. Operasi baca dan pengaturan
pada handle yang sama diserialisasi internal. Hentikan penggunaan handle dari
task lain sebelum memanggil `rotary_encoder_deinit()`.
Contoh memanggil `rotary_encoder_get_delta()` berkala agar perubahan posisi
ditangani oleh aplikasi. Komponen mengaktifkan akumulasi PCNT untuk memperluas
hitungan melampaui batas counter perangkat keras.

## Konfigurasi

| Field | Default / batas | Keterangan |
|---|---|---|
| `pin_a`, `pin_b` | Wajib; GPIO berbeda | Sinyal quadrature A dan B |
| `min_val`, `max_val` | Wajib; `min_val <= max_val` | Batas posisi inklusif |
| `initial_val` | Dijepit ke batas | Posisi awal |
| `rollover` | `false` jika struct di-zero-initialize | `true` membungkus posisi melintasi batas |
| `steps_per_count` | `4` jika nilainya `0` | Banyaknya transisi quadrature per langkah posisi |

Nilai `steps_per_count` bergantung pada encoder dan detent mekanisnya. Coba
nilai `2` atau `4` jika posisi bertambah lebih dari sekali per detent atau
tidak bertambah pada setiap detent.

`rotary_encoder_get_delta()` mengambil perubahan sejak pemanggilan sebelumnya.
Pada mode rollover, delta dihitung sebagai perubahan terpendek melintasi
batas. Rentang posisi dapat meliputi seluruh rentang `int32_t`; delta yang
tidak dapat direpresentasikan sebagai `int32_t` dijepit ke batas tipe tersebut.

## Siklus hidup

Pembuatan mengalokasikan satu unit PCNT dan dua channel. Jika inisialisasi
gagal, error ESP-IDF diteruskan ke aplikasi dan komponen mencoba melepaskan
resource yang sudah dibuat. Setelah tidak digunakan lagi, lepaskan handle
dengan `rotary_encoder_deinit()`. Pull-up GPIO yang diaktifkan saat inisialisasi
tidak dikembalikan otomatis saat deinit.

## Distribusi

Komponen menggunakan PCNT dan GPIO driver ESP-IDF 5.3.0 atau lebih baru.
Manifest `idf_component.yml` memuat versi, deskripsi bahasa Indonesia,
dependensi IDF, dan lisensi MIT. Tambahkan URL repository setelah repository
GitHub publik untuk komponen ini tersedia.
