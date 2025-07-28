#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <string.h>
#include "dht.h" 
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define ALARM_ENABLE_SWITCH_GPIO GPIO_NUM_5  // GPIO pin for alarm enable switch
static QueueHandle_t gpio_evt_queue = NULL;
volatile bool alarm_enabled = false;

#define DHT_GPIO GPIO_NUM_4

#define LCD_ADDR        0x27  // PCF8574のアドレス
#define RX8900_ADDR     0x32

#define LCD_BACKLIGHT   0x08
#define ENABLE          0x04
#define RS              0x01

#define BUZZER_PIN 17
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT // 10ビット分解能
#define LEDC_DUTY       (1)             // 0.1%デューティ
#define LEDC_FREQUENCY  2000              // 2kHz

typedef enum {
    MODE_NORMAL,
    MODE_SET_YEAR,
    MODE_SET_MONTH,
    MODE_SET_DAY,
    MODE_SET_HOUR,
    MODE_SET_MINUTE,
    MODE_SET_SECOND
} clock_mode_t;

typedef struct {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int weekday;
} datetime_t;

volatile clock_mode_t clock_mode = MODE_NORMAL;
datetime_t setting_dt;  // 設定用バッファ

#define SET_ALARM_MODE GPIO_NUM_0
#define SET_MODE GPIO_NUM_18
#define BTN_INC  GPIO_NUM_19
#define BTN_DEC  GPIO_NUM_16

typedef struct {
    int hour;
    int minute;
} alarm_time_t;

volatile alarm_time_t alarm_time = {6, 30}; // 初期値: 06:30

typedef enum {
    MODE_NORMAL_A,      // 通常表示
    MODE_SET_ALARM_H, // アラーム時設定
    MODE_SET_ALARM_M  // アラーム分設定
} alarm_mode_t;

volatile alarm_mode_t alarm_mode = MODE_NORMAL_A;
datetime_t alarm_dt;  // アラーム時刻格納用

// ISRは同じようにキュー送信
static void IRAM_ATTR btn_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void init_buttons(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SET_ALARM_MODE) | (1ULL << SET_MODE) | (1ULL << BTN_INC) | (1ULL << BTN_DEC),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    gpio_isr_handler_add(SET_ALARM_MODE, btn_isr_handler, (void*) SET_ALARM_MODE);
    gpio_isr_handler_add(SET_MODE, btn_isr_handler, (void*) SET_MODE);
    gpio_isr_handler_add(BTN_INC, btn_isr_handler, (void*) BTN_INC);
    gpio_isr_handler_add(BTN_DEC, btn_isr_handler, (void*) BTN_DEC);
}

int melody[] = {880, 880, 880, 880, 880}; // ラーラーラーラーラー
int note_duration = 400;  // 1音400ms
int pause_duration = 100; // 音間の無音100ms

void buzzer_init(void)
{
    // タイマー設定
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // チャンネル設定
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BUZZER_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void play_soft_alarm() {
    for (int i = 0; i < 5; i++) {
        // 周波数設定
        ledc_set_freq(LEDC_MODE, LEDC_TIMER, melody[i]);
        // 音を鳴らす
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(note_duration));

        // 無音にする
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(pause_duration));
    }
}

// ブザー再生タスク
void buzzer_task(void *arg) {
    play_soft_alarm();  // ブザー再生
    vTaskDelete(NULL);  // 再生後タスク終了
}

// 割り込みハンドラ
static void IRAM_ATTR alarm_isr_handler(void *arg)
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

// カーソル移動関数
void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? col : (0x40 + col);
    lcd_send_cmd(0x80 | addr);
}

uint8_t bcd_to_dec(uint8_t val) {
    return ((val >> 4) * 10) + (val & 0x0F);
}

uint8_t dec_to_bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

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
    char line1[13], line2[9];
    snprintf(line1, sizeof(line1), "%04d%02d%02d%s", dt->year, dt->month, dt->day, weekday_str[dt->weekday]);
    snprintf(line2, sizeof(line2), "%02d:%02d:%02d", dt->hour, dt->minute, dt->second);

    lcd_set_cursor(0, 0); // 1行目
    lcd_send_string(line1);
    lcd_set_cursor(1, 0); // 2行目
    lcd_send_string(line2);
}

esp_err_t rx8900_set_time(datetime_t *dt) {
    uint8_t data[7];

    // 曜日を計算（0=Sun〜6=Sat）
    int weekday = calc_weekday(dt->year, dt->month, dt->day);

    data[0] = dec_to_bcd(dt->second);
    data[1] = dec_to_bcd(dt->minute);
    data[2] = dec_to_bcd(dt->hour);
    data[3] = (uint8_t)weekday;
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

void init_alarm_switch(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ALARM_ENABLE_SWITCH_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE   // 立ち下がりエッジで割り込み
    };
    gpio_config(&io_conf);
}

void alarm_task(void *arg)
{
    uint32_t io_num;
    static bool last_state = false;

    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            vTaskDelay(pdMS_TO_TICKS(20));
            // スイッチの状態を読み取る
            bool current_state = (gpio_get_level(ALARM_ENABLE_SWITCH_GPIO) == 0); // プルアップなので0=ON

            if (current_state != last_state) {
                alarm_enabled = current_state;
                last_state = current_state;

                // LCDに表示（例: 2行目の9文字目）
                lcd_set_cursor(1, 9);
                if (alarm_enabled) {
                    lcd_send_data(0x02); // 時計アイコン
                } else {
                    lcd_send_data(' ');
                }
            }
        }
    }
}

void clock_task(void *arg) {
    datetime_t dt;
    while (1) {
        if (clock_mode == MODE_NORMAL) {
            if (rx8900_get_time(&dt) == ESP_OK) {
                display_datetime(&dt);
            }
        } else {
            // 設定中はsetting_dtを表示
            display_datetime(&setting_dt);
            // 設定中の項目にカーソル点滅を追加するならここで制御
        }
        if (alarm_enabled &&
            dt.hour == alarm_time.hour &&
            dt.minute == alarm_time.minute &&
            dt.second == 0) {
            xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 5, NULL);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void dht_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DHT_GPIO), // 対象ピン
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,  // オープンドレイン
        .pull_up_en = GPIO_PULLUP_ENABLE,   // プルアップ有効
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

// ----------------- 温湿度表示用関数 -----------------
void display_temp_humidity(float temp, float hum)
{
    char buf_temp[17];
    char buf_hum[17];
    // 2行目の11文字目から温湿度表示
    snprintf(buf_temp, sizeof(buf_temp), "%2.0f", temp);
    lcd_set_cursor(0, 13);
    lcd_send_string(buf_temp);
    lcd_send_data(0x00);

    snprintf(buf_hum, sizeof(buf_hum), "%2.0f%%", hum);
    lcd_set_cursor(1, 12);
    lcd_send_data(0x01);
    lcd_send_string(buf_hum);
}

// ----------------- 温湿度読み取りタスク -----------------
void dht_task(void *arg)
{
    while (1) {
        float temperature = 0.0f, humidity = 0.0f;

        vTaskDelay(pdMS_TO_TICKS(2000));
        if (dht_read_data(&temperature, &humidity) == ESP_OK) {
            display_temp_humidity(temperature, humidity);
        } else {
            lcd_set_cursor(0, 13);
            lcd_send_string("---");
            lcd_set_cursor(1, 12);
            lcd_send_string("---");
        }
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10秒ごと更新
    }
}

void button_task(void *arg) {
    uint32_t io_num;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            vTaskDelay(pdMS_TO_TICKS(100)); // チャタリング対策

            if (gpio_get_level(io_num) == 0) { // プルアップなので0=押下
                if (io_num == SET_MODE) {
                    // モード切替
                    if (clock_mode == MODE_NORMAL) {
                        // 現在のRTC時間を読み出して編集バッファへ
                        rx8900_get_time(&setting_dt);
                        clock_mode = MODE_SET_YEAR;
                    } else {
                        // 次の項目へ進む
                        if (clock_mode == MODE_SET_SECOND) {
                            // 最後の項目 → RTCに反映
                            rx8900_set_time(&setting_dt);
                            clock_mode = MODE_NORMAL;
                        } else {
                            clock_mode++;
                        }
                    }
                } else if (io_num == SET_ALARM_MODE) {
                    // モード切替
                    if (alarm_mode == MODE_NORMAL_A) {
                        rx8900_set_time(&alarm_dt);
                        alarm_mode = MODE_SET_ALARM_H;
                    } else {
                        // 次の項目へ進む
                        if (alarm_mode == MODE_SET_ALARM_M) {
                            rx8900_set_time(&alarm_dt);
                            alarm_mode = MODE_NORMAL_A;
                        } else {
                            alarm_mode++;
                        }
                    }
                } else if (io_num == BTN_INC) {
                    // 値増加
                    if (clock_mode != MODE_NORMAL) {
                        switch (clock_mode) {
                            case MODE_SET_YEAR:
                                setting_dt.year++;
                                if (setting_dt.year > 2099) setting_dt.year = 2000;
                                break;
                            case MODE_SET_MONTH:
                                setting_dt.month++;
                                if (setting_dt.month > 12) setting_dt.month = 1;
                                break;
                            case MODE_SET_DAY:
                                setting_dt.day++;
                                if (setting_dt.day > 31) setting_dt.day = 1;
                                break;
                            case MODE_SET_HOUR:
                                setting_dt.hour++;
                                if (setting_dt.hour > 23) setting_dt.hour = 0;
                                break;
                            case MODE_SET_MINUTE:
                                setting_dt.minute++;
                                if (setting_dt.minute > 59) setting_dt.minute = 0;
                                break;
                            case MODE_SET_SECOND:
                                setting_dt.second++;
                                if (setting_dt.second > 59) setting_dt.second = 0;
                                break;
                            default: break;
                        }
                    } else if (alarm_mode != MODE_NORMAL_A) {
                        switch (alarm_mode) {
                            case MODE_SET_ALARM_H:
                                alarm_dt.hour++;
                                if (alarm_dt.hour > 23) alarm_dt.hour = 0;
                                break;
                            case MODE_SET_ALARM_M:
                                alarm_dt.minute++;
                                if (alarm_dt.minute > 59) alarm_dt.minute = 0;
                                break;
                            default: break;
                        }
                    }
                } else if (io_num == BTN_DEC) {
                    // 値減少
                    if (clock_mode != MODE_NORMAL) {
                        switch (clock_mode) {
                            case MODE_SET_YEAR:
                                setting_dt.year--;
                                if (setting_dt.year < 2000) setting_dt.year = 2099;
                                break;
                            case MODE_SET_MONTH:
                                setting_dt.month--;
                                if (setting_dt.month < 1) setting_dt.month = 12;
                                break;
                            case MODE_SET_DAY:
                                setting_dt.day--;
                                if (setting_dt.day < 1) setting_dt.day = 31;
                                break;
                            case MODE_SET_HOUR:
                                setting_dt.hour--;
                                if (setting_dt.hour < 0) setting_dt.hour = 23;
                                break;
                            case MODE_SET_MINUTE:
                                setting_dt.minute--;
                                if (setting_dt.minute < 0) setting_dt.minute = 59;
                                break;
                            case MODE_SET_SECOND:
                                setting_dt.second--;
                                if (setting_dt.second < 0) setting_dt.second = 59;
                                break;
                            default: break;
                        }
                    } else if (alarm_mode != MODE_NORMAL_A) {
                        switch (alarm_mode) {
                            case MODE_SET_ALARM_H:
                                alarm_dt.hour--;
                                if (alarm_dt.hour < 0) alarm_dt.hour = 23;
                                break;
                            case MODE_SET_ALARM_M:
                                alarm_dt.minute--;
                                if (alarm_dt.minute < 0) alarm_dt.minute = 59;
                                break;
                            default: break;
                        }
                    }
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
    datetime_t set_dt = {2025, 7, 27, 15, 29, 0, 0}; // {year, month, day, hour, minute, second, weekday}
    rx8900_set_time(&set_dt);

    // GPIO4 をプルアップ設定
    dht_gpio_init();
    dht_init(GPIO_NUM_4); // DHT11をGPIO4に接続

    init_alarm_switch();
    buzzer_init();
    // 割り込み用キュー
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // 割り込みハンドラ登録
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ALARM_ENABLE_SWITCH_GPIO, alarm_isr_handler, (void*) ALARM_ENABLE_SWITCH_GPIO);

    // タスク起動
    xTaskCreate(alarm_task, "snooze_task", 2048, NULL, 10, NULL);
    xTaskCreate(clock_task, "clock_task", 2048, NULL, 10, NULL);
    // 温湿度読み取りタスク起動
    xTaskCreate(dht_task, "dht_task", 2048, NULL, 5, NULL);
    // ブザーは別タスクで鳴らす
    //xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 5, NULL);
    init_buttons();
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
}