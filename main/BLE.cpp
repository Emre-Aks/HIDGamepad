#include "BLE.h"
void BLE::configure_bluetooth(void) {
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
void BLE::re_advertize(void) {

    //kill the input reading tasks call advertizing function
    //re-start the input reading tasks after a connection is formed
}