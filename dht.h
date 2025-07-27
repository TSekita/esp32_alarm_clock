#ifndef DHT_H
#define DHT_H

#include "esp_err.h"
#include "driver/gpio.h"

// GPIO番号を指定して初期化
void dht_init(gpio_num_t pin);

// DHT11から温度・湿度を読み取る
esp_err_t dht_read_data(float *temperature, float *humidity);

#endif
