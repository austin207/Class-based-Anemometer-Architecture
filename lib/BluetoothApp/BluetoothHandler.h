#ifndef BLUETOOTH_HANDLER_H
#define BLUETOOTH_HANDLER_H

#include <Arduino.h>
#include <BluetoothSerial.h>
#include "BluetoothCommand.h"

class BluetoothHandler {
public:
    BluetoothHandler();
    void initialize(const char* deviceName);
    bool isConnected();
    String receiveCommand();
    BluetoothSerial& getSerial();
private:
    BluetoothSerial SerialBT;
};

#endif // BLUETOOTH_HANDLER_H
