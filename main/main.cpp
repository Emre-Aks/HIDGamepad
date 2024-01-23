#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "NimBLEDevice.h"
#include "NimBLEHIDDevice.h"
#include "drivers.h"

//globals
NimBLEAdvertising* advertizer;
NimBLECharacteristic* characteristic;
NimBLEServer* server;
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