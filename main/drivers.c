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
void configure_led(void)//static?
{
    //1 led on pin 8
    led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_NUM_8,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10mHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}
uint8_t read_buttons(void) {
    uint8_t levels = 0;
    //enncode
    levels |= (gpio_get_level(GPIO_NUM_0) << 0);//a
    levels |= (gpio_get_level(GPIO_NUM_1) << 1);//b
    levels |= (gpio_get_level(GPIO_NUM_2) << 2);//x
    levels |= (gpio_get_level(GPIO_NUM_6) << 3);//y
    levels |= (gpio_get_level(GPIO_NUM_7) << 4);//L
    //levels |= (gpio_get_level(GPIO_NUM_8) << 5);//R (currently led)
    levels |= (gpio_get_level(GPIO_NUM_9) << 6);//start
    levels |= (gpio_get_level(GPIO_NUM_10) << 7);//select
    
    return levels;
}
void configure_buttons(void) {//static?
    //reset pins
    gpio_reset_pin(GPIO_NUM_0);//a
    gpio_reset_pin(GPIO_NUM_1);//b
    gpio_reset_pin(GPIO_NUM_2);//x
    gpio_reset_pin(GPIO_NUM_6);//y
    gpio_reset_pin(GPIO_NUM_7);//L
    //gpio_reset_pin(GPIO_NUM_8);//R (currently led)
    gpio_reset_pin(GPIO_NUM_9);//select
    gpio_reset_pin(GPIO_NUM_10);//start
    
    //set as input
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);//a
    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_INPUT);//b
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);//x
    gpio_set_direction(GPIO_NUM_6, GPIO_MODE_INPUT);//y
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_INPUT);//L
    //gpio_set_direction(GPIO_NUM_8, GPIO_MODE_INPUT);//R (currently led)
    gpio_set_direction(GPIO_NUM_9, GPIO_MODE_INPUT);//start
    gpio_set_direction(GPIO_NUM_10, GPIO_MODE_INPUT);//select
    //gpio_set_direction(GPIO_NUM_20, GPIO_MODE_INPUT);
    //gpio_set_direction(GPIO_NUM_21, GPIO_MODE_INPUT);
}
uint16_t read_stick(void) {
    return 1;
}
uint8_t read_mic(void) {
    int8_t readbuf;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, (int*)&readbuf);
    return (uint8_t)readbuf;//one bit of precision is lost, since bitwidth is 9, only returns positive readings
}
void configure_adc(void) {
    //initialize adc1
    static adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    //configure channel 4
    static adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config);
}
void configure_i2c(void) {//static?
    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = GPIO_NUM_19;
    config.scl_io_num = GPIO_NUM_18;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = 400000;
    config.clk_flags = 0;
    i2c_param_config(0, &config);
    i2c_driver_install(0, config.mode, 0, 0, 0);
}
void configure_MPU6050(void) {//static?
    //-------------------------------------------------------------
    //  start  |  address + write  |  byte 1  |  byte 2  |  stop  |  
    //-------------------------------------------------------------
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);//start
    i2c_master_write_byte(cmd, (0x68 << 1) | I2C_MASTER_WRITE, 0x1);//address + write
    i2c_master_write_byte(cmd, 0x6B, 0x1);//register address for reset (byte 1)
    i2c_master_write_byte(cmd, 0x00, 0x1);//reset value at 0x6B (byte 2)
    i2c_master_stop(cmd);//stop
    int err = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}
uint8_t read_MPU6050_accel(int16_t* accel_bufr) {
    uint8_t write_bufr = 0x3B;//address of accelerometer data
    //read the three 16 bit axis as 6, 8 bit 2s comp numbers in big endian
    uint8_t err = i2c_master_write_read_device(0, 0x68, &write_bufr, 1, (uint8_t*) accel_bufr, 6, 1000/portTICK_PERIOD_MS);
    //flip to little endian, convert to m/s^2
    accel_bufr[0] = ((int16_t)(((accel_bufr[0] & 0xff00) >> 8) | ((accel_bufr[0] & 0x00ff) << 8)))/ 258;//x
    accel_bufr[1] = ((int16_t)(((accel_bufr[1] & 0xff00) >> 8) | ((accel_bufr[1] & 0x00ff) << 8)))/ 258;//y
    accel_bufr[2] = ((int16_t)(((accel_bufr[2] & 0xff00) >> 8) | ((accel_bufr[2] & 0x00ff) << 8)))/ 258;//z

    return err;
}

uint8_t read_MPU6050_gyro(int16_t* gyro_bufr) {
    uint8_t write_bufr = 0x43;//address of accelerometer data
    //read the three 16 bit axis as 6, 8 bit 2s comp numbers in big endian
    uint8_t err = i2c_master_write_read_device(0, 0x68, &write_bufr, 1, (uint8_t*) gyro_bufr, 6, 1000/portTICK_PERIOD_MS);
    //flip to little endian, convert to deg/sec
    gyro_bufr[0] = ((int16_t)(((gyro_bufr[0] & 0xff00) >> 8) | ((gyro_bufr[0] & 0x00ff) << 8)))/ 258;//x
    gyro_bufr[1] = ((int16_t)(((gyro_bufr[1] & 0xff00) >> 8) | ((gyro_bufr[1] & 0x00ff) << 8)))/ 258;//y
    gyro_bufr[2] = ((int16_t)(((gyro_bufr[2] & 0xff00) >> 8) | ((gyro_bufr[2] & 0x00ff) << 8)))/ 258;//z

    return err;
}