#include "NimBLEDevice.h"
#include "NimBLEHIDDevice.h"
#include "stdint.h"
class BLE {
public:
    BLE(){};
    ~BLE(){};
    
    NimBLEAdvertising* advertizer;
    NimBLECharacteristic* characteristic;
    NimBLEServer* server;

    void configure_bluetooth(void);
    void re_advertize(void);
private:
    uint8_t reportMap[55] = {
                //---------------------------------------------------------------------------------------------
        //| buttons | accel x | accel y | accel z | gyro x | gyro y | gyro z | stick x | stick y | mic |
        //---------------------------------------------------------------------------------------------
        0x05, 0x01,//USAGE_PAGE (Generic Desktop)
        0x09, 0x05,//USAGE (Game Pad)
        0xA1, 0x01,//COLLECTION (Application)
        0x85, 0x04,//REPORT_ID (4)
        0x05, 0x09,//USAGE_PAGE (Button)
        0x19, 0x01,//USAGE_MINIMUM (Button 1)
        0x29, 0x08,//USAGE_MAXIMUM (Button 8)
        0x15, 0x00,//LOGICAL_MINIMUM (0)	
        0x25, 0x01,//LOGICAL_MAXIMUM (1)
        0x75, 0x01,//REPORT_SIZE (1)	
        0x95, 0x08,//REPORT_COUNT (8)
        0x81, 0x02,//INPUT (Data,Var,Abs)
        0x05, 0x01,//USAGE_PAGE (Generic Desktop)
        0x15, 0x81,//LOGICAL_MINIMUM (-127)
        0x25, 0x7F,//LOGICAL_MAXIMUM (127)
        0x09, 0x33,//USAGE (Rx)
        0x09, 0x34,//USAGE (Ry)
        0x09, 0x35,//USAGE (Rz)
        0x09, 0x40,//USAGE (Vx)
        0x09, 0x41,//USAGE (Vy)
        0x09, 0x42,//USAGE (Vz)
        0x09, 0x30,//USAGE (X)
        0x09, 0x31,//USAGE (Y)
        0x09, 0x36,//USAGE (Slider)
        0x75, 0x08,//REPORT_SIZE (8)
        0x95, 0x09,//REPORT_COUNT (9)
        0x81, 0x02,//INPUT (Data,Var,Abs)	
        0xC0       //END_COLLECTION
    };
};