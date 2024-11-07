#include "WifiConnecting.h"

WifiConnecting::WifiConnecting() 
    : previousMillis(0), interval(5000), wifiConnected(false) {}

void WifiConnecting::begin(const char* deviceName) {
    SerialBT.begin(deviceName);
    Serial.println("Initializing ESP32...");
}

void WifiConnecting::loop() {
    sendBluetoothMessages();
    handleBluetoothInput();
}

void WifiConnecting::sendBluetoothMessages() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Gửi thông báo nếu chưa kết nối Wi-Fi sau mỗi 5 giây
        if (!wifiConnected) {
            SerialBT.println("Hello from ESP32!");
            SerialBT.println("Please enter Wifi_ID and Password with format:");
            SerialBT.println("Wifi_ID,Password");
            Serial.println("Waiting to enter Wifi information...");
        } else {
            Serial.println("Wi-Fi connected!");
        }
    }
}

void WifiConnecting::connectToWiFi(const String& ssid, const String& password) {
    Serial.println("Connecting to Wifi...");
    Serial.print("SSID: "); Serial.println(ssid);

    WiFi.begin(ssid.c_str(), password.c_str());
    unsigned long startTime = millis();

    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - startTime > 10000) { 
            SerialBT.println("Connection failed. Please check the Wi-Fi ID and password.");
            Serial.println("Connection failed after 10 seconds.");
            return; 
        }

        Serial.println("Not connected, waiting...");
        SerialBT.println("Connecting..........");
        delay(500);
    }

    // Send successful message
    SerialBT.println("ESP32 connected to Wi-Fi!");
    SerialBT.println("Closing Bluetooth connection!");
    wifiConnected = true; 
    Serial.print("IP Address: "); Serial.println(WiFi.localIP());
    
    // Waiting 5s to close Bluetooth
    delay(5000);
    SerialBT.end(); 
}

void WifiConnecting::handleBluetoothInput() {
    if (SerialBT.available()) {
        String value = SerialBT.readStringUntil('\n');
        value.trim(); // Loại bỏ các ký tự trắng ở đầu và cuối
        if (value.length() > 0) {
            int commaIndex = value.indexOf(','); 
            if (commaIndex > 0) {
                String ssid = value.substring(0, commaIndex); 
                String password = value.substring(commaIndex + 1); 
                connectToWiFi(ssid, password);
            } else {
                Serial.println("Invalid format, please try again.");
            } 
        }
    }
}