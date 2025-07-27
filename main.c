#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <string.h>

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define SNOOZE_SWITCH_GPIO GPIO_NUM_5  // GPIO pin for snooze switch
static QueueHandle_t gpio_evt_queue = NULL;
volatile bool snooze_enabled = false;

#define LCD_ADDR        0x27  // PCF8574のアドレス
#define RX8900_ADDR     0x32

#define LCD_BACKLIGHT   0x08
#define ENABLE          0x04
#define RS              0x01

// 割り込みハンドラ
static void IRAM_ATTR snooze_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


// 時計のアイコン用のカスタムキャラクター
uint8_t clock_char[8] = {
    0b00000,
    0b01110,
    0b10101,
    0b10101,
    0b10111,
    0b10001,
    0b01110,
    0b00000
};

uint8_t degree_char[8] = {
    0b01000,
    0b10100,
    0b01000,
    0b00111,
    0b01000,
    0b01000,
    0b00111,
    0b00000
};

uint8_t humidity_char[8] = {
    0b00100,
    0b00100,
    0b01110,
    0b11111,
    0b11111,
    0b11111,
    0b01110,
    0b00000
};

void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

void lcd_send_cmd(uint8_t cmd) {
    uint8_t upper = cmd & 0xF0;
    uint8_t lower = (cmd << 4) & 0xF0;
    uint8_t data_t[4];

    data_t[0] = upper | LCD_BACKLIGHT | ENABLE;
    data_t[1] = upper | LCD_BACKLIGHT;
    data_t[2] = lower | LCD_BACKLIGHT | ENABLE;
    data_t[3] = lower | LCD_BACKLIGHT;

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (LCD_ADDR << 1), true);
    i2c_master_write(handle, data_t, sizeof(data_t), true);
    i2c_master_stop(handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, handle, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(handle);
}

void lcd_send_data(uint8_t data) {
    uint8_t upper = data & 0xF0;
    uint8_t lower = (data << 4) & 0xF0;
    uint8_t data_t[4];

    data_t[0] = upper | LCD_BACKLIGHT | ENABLE | RS;
    data_t[1] = upper | LCD_BACKLIGHT | RS;
    data_t[2] = lower | LCD_BACKLIGHT | ENABLE | RS;
    data_t[3] = lower | LCD_BACKLIGHT | RS;

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (LCD_ADDR << 1), true);
    i2c_master_write(handle, data_t, sizeof(data_t), true);
    i2c_master_stop(handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, handle, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(handle);
}

void lcd_init(void) {
    lcd_send_cmd(0x33);
    lcd_send_cmd(0x32);
    lcd_send_cmd(0x28);
    lcd_send_cmd(0x0C);
    lcd_send_cmd(0x06);
    lcd_send_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void lcd_send_string(const char *str) {
    while (*str) {
        lcd_send_data((uint8_t)(*str));
        str++;
    }
}

uint8_t bcd_to_dec(uint8_t val) {
    return ((val >> 4) * 10) + (val & 0x0F);
}

uint8_t dec_to_bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

typedef struct {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int weekday;
} datetime_t;

esp_err_t rx8900_get_time(datetime_t *dt) {
    uint8_t data[7];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RX8900_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // 先頭アドレス
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RX8900_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + 6, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        dt->second = bcd_to_dec(data[0] & 0x7F);
        dt->minute = bcd_to_dec(data[1] & 0x7F);
        dt->hour   = bcd_to_dec(data[2] & 0x3F);
        dt->weekday= data[3] & 0x07;
        dt->day    = bcd_to_dec(data[4] & 0x3F);
        dt->month  = bcd_to_dec(data[5] & 0x1F);
        dt->year   = 2000 + bcd_to_dec(data[6] & 0xFF);
    }
    return ret;
}

const char* weekday_str[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};

int calc_weekday(int y, int m, int d) {
    if (m < 3) {
        m += 12;
        y -= 1;
    }
    int k = y % 100;
    int j = y / 100;
    int h = (d + 13*(m+1)/5 + k + k/4 + j/4 + 5*j) % 7;
    // h = 0: Saturday, so convert
    return (h + 6) % 7; // 0=Sunday
}

void display_datetime(datetime_t *dt) {
    char line1[17], line2[17];
    snprintf(line1, sizeof(line1), "%04d%02d%02d%s", dt->year, dt->month, dt->day, weekday_str[dt->weekday]);
    snprintf(line2, sizeof(line2), "%02d:%02d:%02d", dt->hour, dt->minute, dt->second);

    lcd_send_cmd(0x80); // 1行目
    lcd_send_string(line1);
    lcd_send_cmd(0xC0); // 2行目
    lcd_send_string(line2);
}

esp_err_t rx8900_set_time(datetime_t *dt) {
    uint8_t data[7];

    // 曜日を計算（0=Sun〜6=Sat）
    int weekday = calc_weekday(dt->year, dt->month, dt->day);

    data[0] = dec_to_bcd(dt->second);
    data[1] = dec_to_bcd(dt->minute);
    data[2] = dec_to_bcd(dt->hour);
    data[3] = weekday;
    data[4] = dec_to_bcd(dt->day);
    data[5] = dec_to_bcd(dt->month);
    data[6] = dec_to_bcd(dt->year - 2000);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RX8900_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // 秒レジスタから書き込み
    i2c_master_write(cmd, data, 7, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void lcd_create_custom_char(uint8_t location, uint8_t charmap[])
{
    // location: 0 to 7 (CGRAM slot number)
    location &= 0x07;

    // Send CGRAM address set command
    lcd_send_cmd(0x40 | (location << 3));

    // Write the character pattern in 8 bytes
    for (int i = 0; i < 8; i++) {
        lcd_send_data(charmap[i]);
    }
}

void init_snooze_switch(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SNOOZE_SWITCH_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE   // 立ち下がりエッジで割り込み
    };
    gpio_config(&io_conf);
}

void snooze_task(void *arg)
{
    uint32_t io_num;
    static bool last_state = false;

    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            vTaskDelay(pdMS_TO_TICKS(20));
            // スイッチの状態を読み取る
            bool current_state = (gpio_get_level(SNOOZE_SWITCH_GPIO) == 0); // プルアップなので0=ON

            if (current_state != last_state) {
                snooze_enabled = current_state;
                last_state = current_state;

                // LCDに表示（例: 2行目の9文字目）
                lcd_send_cmd(0xC0 + 9);
                if (snooze_enabled) {
                    lcd_send_data(0x02); // 時計アイコン
                } else {
                    lcd_send_data(' ');
                }
            }
        }
    }
}

void app_main(void) {
    i2c_master_init();
    lcd_init();
    lcd_create_custom_char(0, degree_char);  // Register degree symbol to slot 0
    lcd_create_custom_char(1, humidity_char);  // Register humidity symbol to slot 1
    lcd_create_custom_char(2, clock_char);  // Register clock icon to slot 2

    // 初回だけ設定
    datetime_t set_dt = {2025, 7, 27, 15, 29, 0};
    rx8900_set_time(&set_dt);

    datetime_t dt;

    init_snooze_switch();
    // 割り込み用キュー
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // 割り込みハンドラ登録
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SNOOZE_SWITCH_GPIO, snooze_isr_handler, (void*) SNOOZE_SWITCH_GPIO);

    // タスク起動
    xTaskCreate(snooze_task, "snooze_task", 2048, NULL, 10, NULL);
    while (1) {
        if (rx8900_get_time(&dt) == ESP_OK) {
            display_datetime(&dt);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1秒ごと更新
    }
}

