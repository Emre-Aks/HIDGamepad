#pragma once
#include "stdint.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static led_strip_handle_t led_strip;

void set_led(int state, uint8_t R, uint8_t G, uint8_t B);
void configure_led(void);
uint16_t read_buttons(void);
void configure_buttons(void);
uint16_t read_stick(void);
uint16_t read_mic(void);
void configure_adc(void);
void configure_i2c(void);
void configure_MPU6050(void);
uint16_t read_i2c_device(void);