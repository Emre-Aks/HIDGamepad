#include "drivers.h"

void set_led(int state, uint8_t R, uint8_t G, uint8_t B)
{
    /* If the addressable LED is enabled */
    if (state == 1) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, R, G, B);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}
void configure_led(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_NUM_8,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}
uint16_t read_buttons(void) {
    uint16_t levels = 0;
    //enncode
    levels |= (gpio_get_level(GPIO_NUM_0) << 0);//a
    levels |= (gpio_get_level(GPIO_NUM_1) << 1);//b
    levels |= (gpio_get_level(GPIO_NUM_2) << 2);//x
    levels |= (gpio_get_level(GPIO_NUM_6) << 3);//y
    levels |= (gpio_get_level(GPIO_NUM_7) << 4);//L
    //levels |= (gpio_get_level(GPIO_NUM_8) << 5);//R (currently led)
    levels |= (gpio_get_level(GPIO_NUM_9) << 6);//start
    levels |= (gpio_get_level(GPIO_NUM_10) << 7);//select
    //levels |= (gpio_get_level(GPIO_NUM_20) << 8);//LT
    //levels |= (gpio_get_level(GPIO_NUM_21) << 9);//RT
    
    return levels;
}
void configure_buttons(void) {
    //reset pins
    gpio_reset_pin(GPIO_NUM_0);//a
    gpio_reset_pin(GPIO_NUM_1);//a
    gpio_reset_pin(GPIO_NUM_2);//a
    gpio_reset_pin(GPIO_NUM_6);//a
    gpio_reset_pin(GPIO_NUM_7);//a
    //gpio_reset_pin(GPIO_NUM_8);//a (currently led)
    gpio_reset_pin(GPIO_NUM_9);//a
    gpio_reset_pin(GPIO_NUM_10);//a
    //gpio_reset_pin(GPIO_NUM_20);//a
    //gpio_reset_pin(GPIO_NUM_21);//a
    
    //set as input
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_6, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_INPUT);
    //gpio_set_direction(GPIO_NUM_8, GPIO_MODE_INPUT); (currently led)
    gpio_set_direction(GPIO_NUM_9, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_10, GPIO_MODE_INPUT);
    //gpio_set_direction(GPIO_NUM_20, GPIO_MODE_INPUT);
    //gpio_set_direction(GPIO_NUM_21, GPIO_MODE_INPUT);
}
uint16_t read_stick(void) {
    return 1;
}
uint16_t read_mic(void) {
    return 1;
}
void configure_adc(void) {
    //configure adc 1 and 2 on gpio 4 and 5
}
void configure_i2c(void) {
    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = GPIO_NUM_19;
    config.scl_io_num = GPIO_NUM_18;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = 400000;
    i2c_param_config(0, &config);
    i2c_driver_install(0, config.mode, 0, 0, 0);
}
void configure_MPU6050(void) {

}
uint16_t read_i2c_device(void) {
    uint8_t reg_address = 0x40;
    uint8_t reg_address2 = 0x3F;
    uint8_t data = 0;
    uint8_t data2 = 0;
    i2c_master_write_read_device(0, 0x68, &reg_address, 1, &data, 2, 1000 / portTICK_PERIOD_MS);
    i2c_master_write_read_device(0, 0x68, &reg_address2, 1, &data, 2, 1000 / portTICK_PERIOD_MS);
    return (data | (data2 << 8));
}