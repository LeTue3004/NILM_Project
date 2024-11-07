#ifndef WifiConnecting_h
#define WifiConnecting_h

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <WiFi.h>

class WifiConnecting {
public:
    WifiConnecting();
    void begin(const char* deviceName);
    void loop();
    void connectToWiFi(const String& ssid, const String& password);

private:
    void sendBluetoothMessages();
    void handleBluetoothInput();
    
    BluetoothSerial SerialBT;
    unsigned long previousMillis;
    const long interval;
    bool wifiConnected;
};

#endif