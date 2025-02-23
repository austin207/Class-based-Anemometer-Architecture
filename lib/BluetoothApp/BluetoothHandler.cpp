#include "BluetoothHandler.h"
#include <Arduino.h>

BluetoothHandler::BluetoothHandler() {
}

void BluetoothHandler::initialize(const char* deviceName) {
    SerialBT.begin(deviceName);
    Serial.println("Bluetooth Started. Waiting for connection...");
}

bool BluetoothHandler::isConnected() {
    return SerialBT.hasClient();
}

String BluetoothHandler::receiveCommand() {
    if (SerialBT.available()) {
        String receivedCommand = SerialBT.readStringUntil('\n');
        receivedCommand.trim();
        Serial.print("Received Command: ");
        Serial.println(receivedCommand);
        return receivedCommand;
    }
    return "";
}

BluetoothSerial& BluetoothHandler::getSerial() {
    return SerialBT;
}
