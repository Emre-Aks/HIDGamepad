#include <stdio.h>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "NimBLEDevice.h"
#include "NimBLEHIDDevice.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

//globals
NimBLEAdvertising* advertizer;
NimBLECharacteristic* characteristic;
NimBLEServer* server;
static led_strip_handle_t led_strip;
static uint8_t reportMap[] = {// This looks like { report id [00000000] 8 buttons [00000000] 8 buttons [00000000] }
    //controller report map
    0x05, 0x01,  //  Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,  //  Usage (Game Pad)
    0xA1, 0x01,  // Collection (Application)
    0x85, 0x04,  //  Report ID (4)
    0x05, 0x09,  //  Usage Page (Button)
    0x19, 0x01,  //  Usage Minimum (Button 1)
    0x29, 0x10,  //  Usage Maximum (Button 16)
    0x15, 0x00,  //  Logical Minimum (0)
    0x25, 0x01,  //  Logical Maximum (1)
    0x75, 0x01,  //  Report Size (1)
    0x95, 0x10,  //  Report Count (1)
    0x81, 0x02,  //  Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,        // End Collection
};
TaskHandle_t* xHandleButtons = NULL;
TaskHandle_t* xHandleAnalog = NULL;
TaskHandle_t* xHandleI2C = NULL;

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
void configure_bluetooth(void) {
    NimBLEDevice::init("very_cool_gamepad");// init BLE stack

    //Create the BLE server that hosts the service
    server = NimBLEDevice::createServer();

    //create HID device service on server
    NimBLEHIDDevice* HIDDevice = new NimBLEHIDDevice(server);

    HIDDevice->hidInfo(0x00, 0x01);

    //optional stuff
    HIDDevice->setBatteryLevel(69);

    HIDDevice->reportMap(reportMap, sizeof(reportMap));//the report map is an array of 25 bytes

    //Characteristic for sending input reports to client (global because used in tasks)
    characteristic = HIDDevice->inputReport(4);

    //start the hid service
    HIDDevice->startServices();

    //advertizer for advertizing to clients (global because used in tasks)
    advertizer = server->getAdvertising();
    advertizer->setAppearance(HID_GAMEPAD);
    advertizer->addServiceUUID(HIDDevice->hidService()->getUUID());
    advertizer->start();
}
void re_advertize(void) {

    //kill the input reading tasks call advertizing function
    //re-start the input reading tasks after a connection is formed
}
void vTaskAnalogRead(void *pvParameter) {
    for (;;) {
    }
}
void vTaskButtonsRead(void* pvParameter) {
    configure_buttons();
    uint16_t buffer;
    for (;;) {
        buffer = read_buttons();
        //vTaskSuspend();//suspend tasks that share characteristic
        characteristic->setValue(buffer);//pointer issue?
        characteristic->notify();
        //xTaskResume();//resume tasks that share characteristic
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
void vTaskI2CRead(void* pvParameter) {
    configure_i2c();
    for (;;) {
        printf("accel 8 bit z was: %u \n", read_i2c_device());
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
void vTaskBlink(void* pvParameter) {
    configure_led();
    bool state = true;
    for (;;) {
        set_led(state, 1,1,1);
        state = !state;
        vTaskDelay(1000/portTICK_PERIOD_MS);
        set_led(state, 1,1,1);
        state = !state;
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
extern "C" void app_main(void)
{   
    //Start bluetooth stack in RTOS
    configure_bluetooth();
    //wait for connection, blue status led (main is a task so this doesnt block bt activities)
    configure_led();
    bool state = true;
    while (server->getConnectedCount() == 0) {
        set_led(state,0,0,10);
        state = !state;
        vTaskDelay(1000/portTICK_PERIOD_MS);
        set_led(state,0,0,10);
        state = !state;
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

    //connection seccuded, green solid status led
    set_led(1,0,10,0);
    //create and start read tasks
    //xTaskCreate(&vTaskBlink, "BlinkTask", 6000, NULL, 1, NULL);
    //xTaskCreate(&vTaskButtonsRead, "ButtonTask", 6000, NULL, 1, NULL);
    xTaskCreate(&vTaskI2CRead, "I2CTask", 6000, NULL, 1, NULL);

    //done with main task
    vTaskSuspend(NULL);//once setup is complete, our other tasks handle everything else, main can be suspended
}

//=======================================================
// configure_led();
//     bool state = true;
//     for (;;) {
//         set_led(state, 10,0,0);
//         state = !state;
//         vTaskDelay(1000/portTICK_PERIOD_MS);
//         set_led(state, 10,0,0);
//         state = !state;
//         vTaskDelay(1000/portTICK_PERIOD_MS);
//     }

// NimBLEDevice::init("emrea_gamepad");// init BLE stack

    // //Create the BLE server that hosts the service
    // NimBLEServer* server = NimBLEDevice::createServer();
    // server->setCallbacks(new ServerCallbacks);

    // //create HID device service on server
    // NimBLEHIDDevice* HIDDevice = new NimBLEHIDDevice(server);


    // HIDDevice->hidInfo(0x00, 0x01);

    // //optional stuff
    // //HIDDevice->setBatteryLevel(100);

    // HIDDevice->reportMap(reportMap, sizeof(reportMap));//the report map is an array of 25 bytes

    // //Characteristic for sending input reports to client (global because used in tasks)
    // //NimBLECharacteristic* characteristic;
    // characteristic = HIDDevice->inputReport(4);

    // //start the hid service
    // HIDDevice->startServices();

    // //advertizer for advertizing to clients (global because used in tasks)
    // //NimBLEAdvertising* advertizer;
    // advertizer = server->getAdvertising();
    // advertizer->setAppearance(HID_GAMEPAD);
    // advertizer->addServiceUUID(HIDDevice->hidService()->getUUID());
    // advertizer->start();
// #define lowByte(w) ((uint8_t) ((w) & 0xff))
// #define highByte(w) ((uint8_t) ((w) >> 8))
// pnp values and hidinfo
// uint16_t vid = 0xe502;
// uint16_t pid = 0xbbab;
// uint16_t guid = 0x0110;

// uint8_t high = highByte(pid);
// uint8_t low = lowByte(pid);
// pid = low << 8 | high;

// high = highByte(guid);
// low = lowByte(guid);
// guid = low << 8 | high;

// high = highByte(vid);
// low = lowByte(vid);
// vid = low << 8 | high;

//HIDDevice->pnp(0x01, vid, pid, guid);
//_guidVersion(0x0110)
//_vid(0xe502),
//_pid(0xbbab),
