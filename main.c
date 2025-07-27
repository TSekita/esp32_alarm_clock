#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define LCD_ADDR 0x27  // PCF8574のアドレス

#define LCD_BACKLIGHT   0x08
#define ENABLE          0x04
#define RS              0x01

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

void app_main() {
    i2c_master_init();
    lcd_init();
    lcd_send_cmd(0x80);       // 1行目先頭
    lcd_send_string("Hello, World!");
    lcd_send_cmd(0xC0);       // 2行目先頭
    lcd_send_string("LCD1602 Test");
}
