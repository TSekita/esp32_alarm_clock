#include "dht.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "DHT11";
static gpio_num_t dht_pin;

// マイクロ秒単位のdelay
static void delay_us(uint32_t us)
{
    esp_rom_delay_us(us);
}

void dht_init(gpio_num_t pin)
{
    dht_pin = pin;
    gpio_set_direction(dht_pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(dht_pin, 1); // 初期はHIGH
}

esp_err_t dht_read_data(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};

    // スタート信号送信
    gpio_set_direction(dht_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dht_pin, 0);
    delay_us(18000);  // 18ms Low
    gpio_set_level(dht_pin, 1);
    delay_us(30);
    gpio_set_direction(dht_pin, GPIO_MODE_INPUT);

    // 応答確認
    if (gpio_get_level(dht_pin) == 1) return ESP_FAIL;
    while (gpio_get_level(dht_pin) == 0); // 80us Low
    while (gpio_get_level(dht_pin) == 1); // 80us High

    // 40ビット受信
    for (int i = 0; i < 40; i++) {
        while (gpio_get_level(dht_pin) == 0); // start of bit
        uint32_t width = 0;
        while (gpio_get_level(dht_pin) == 1) {
            width++;
            delay_us(1);
        }
        // 26-28us = 0, 70us = 1
        data[i / 8] <<= 1;
        if (width > 40) data[i / 8] |= 1;
    }

    // チェックサム確認
    if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4]) {
        ESP_LOGE(TAG, "Checksum error");
        return ESP_FAIL;
    }

    *humidity = data[0];
    *temperature = data[2];
    return ESP_OK;
}
