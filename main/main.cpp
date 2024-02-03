#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "NimBLEDevice.h"
#include "NimBLEHIDDevice.h"
extern "C" {
    #include "drivers.h"
}
#include "BLE.h"

//globals
volatile uint8_t inputBufr[10] = {0,0,0,0,0,0,0,0,0,0};
TaskHandle_t* xHandleButtons = NULL;
TaskHandle_t* xHandleAnalog = NULL;
TaskHandle_t* xHandleMPU6050 = NULL;

void vTaskAnalogRead(void *pvParameter) {
    BLE* BLEInstance = (BLE*)pvParameter;
    configure_adc();

    for (;;) {
        inputBufr[7] = read_mic();
        BLEInstance->characteristic->setValue((uint8_t*)&inputBufr, 10);
        BLEInstance->characteristic->notify();
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
void vTaskButtonsRead(void* pvParameter) {
    BLE* BLEInstance = (BLE*)pvParameter;
    configure_buttons();

    for (;;) {
        inputBufr[0] = read_buttons();
        BLEInstance->characteristic->setValue((uint8_t*)&inputBufr, 10);
        BLEInstance->characteristic->notify();
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
void vTaskMPU6050Read(void* pvParameter) {
    BLE* BLEInstance = (BLE*)pvParameter;

    int16_t accel_data[3];
    int16_t gyro_data[3];

    configure_i2c();
    configure_MPU6050();

    for (;;) {
        //read accel
        if (read_MPU6050_accel(accel_data) != ESP_OK) {
            printf("BAD JUJU HAPPENED!\n");
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
        //read gyro
        if (read_MPU6050_gyro(gyro_data) != ESP_OK) {
            printf("BAD JUJU HAPPENED!\n");
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
        //add to buffer and notify
        for (int i = 0; i < 3; i++) {
            inputBufr[i+1] = accel_data[i];
            inputBufr[i+4] = gyro_data[i];
        }
        BLEInstance->characteristic->setValue((uint8_t*)inputBufr, 10);
        BLEInstance->characteristic->notify();
    }
}
extern "C" void app_main(void)
{
    BLE BLEInstance = BLE();
    //Start bluetooth stack in RTOS
    BLEInstance.configure_bluetooth();
    //wait for connection, blue status led (main is a task so this doesnt block bt activities)
    configure_led();
    bool state = true;
    while (BLEInstance.server->getConnectedCount() == 0) {
        set_led(state,0,0,10);
        state = !state;
        vTaskDelay(1000/portTICK_PERIOD_MS);
        set_led(state,0,0,10);
        state = !state;
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

    //connection succeded, green solid status led
    set_led(1,0,10,0);
    //create and start read tasks
    xTaskCreate(&vTaskAnalogRead, "AnalogTask", 6000, &BLEInstance, 1, xHandleAnalog);
    xTaskCreate(&vTaskButtonsRead, "ButtonTask", 6000, &BLEInstance, 1, xHandleButtons);
    xTaskCreate(&vTaskMPU6050Read, "I2CTask", 12000, &BLEInstance, 1, xHandleMPU6050);

    //done with setup, above tasks handle all functionality
    vTaskSuspend(NULL);
}