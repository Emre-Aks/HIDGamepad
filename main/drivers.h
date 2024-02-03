#pragma once
#include "stdint.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static led_strip_handle_t led_strip;
static adc_oneshot_unit_handle_t adc1_handle;

void set_led(int state, uint8_t R, uint8_t G, uint8_t B);
void configure_led(void);
uint8_t read_buttons(void);
void configure_buttons(void);
uint16_t read_stick(void);
uint8_t read_mic(void);
void configure_adc(void);
void configure_i2c(void);
void configure_MPU6050(void);
uint8_t read_MPU6050_accel(int16_t* accel_bufr);
uint8_t read_MPU6050_gyro(int16_t* gyro_bufr);