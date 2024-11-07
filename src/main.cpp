#include <Arduino.h>
#include <BluetoothSerial.h>
#include <WiFi.h>
#include "SPI.h"
#include <TFT_eSPI.h>

BluetoothSerial SerialBT;
TFT_eSPI tft = TFT_eSPI();

String ssid = "";
String password = "";
bool wifiConnected = false;
bool bluetoothDisplayed = false;

const long interval = 5000; // Thời gian chờ 5 giây

const char* devices[] = {
    "Led", "Fan", "Refrigerator", "Air Purifier", "Fruit juicer", "Hair dryer", "Laptop Charger", "iPhone Charger"
};

const float power[] = {5, 46, 75, 38, 150, 2300, 45, 18}; // Thay đổi giá trị thực tế

#define MAX_IMAGE_WIDTH 240
#define COLOR_WHITE 0xFFFF
#define COLOR_BLACK 0x0000

void customDelay(unsigned long delayTime) {
    unsigned long startMillis = millis(); 
    while (millis() - startMillis < delayTime) {
    }
}

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32 Bluetooth Connection Established");
    Serial.println("Initializing ESP32...");
    tft.setRotation(1);
    tft.begin();
    tft.fillScreen(COLOR_BLACK);
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(2);
    tft.drawCentreString("Connecting.....", 120, 80 + 5, 1);
    tft.drawCentreString("Active Devices", 120, 80 + 25, 1);

    tft.setTextSize(1);
    tft.drawCentreString("Device", 60, 80 + 45, 2);
    tft.drawCentreString(" : ", 120, 80 + 45, 2);
    tft.drawCentreString("Power", 180, 80 + 45, 2);
}

void sendBluetoothMessages() {
    if (!SerialBT.connected()) {
        Serial.println("Connecting....");
        bluetoothDisplayed = false;
    } else if (!wifiConnected) {
        if (!bluetoothDisplayed) {
            tft.setTextColor(COLOR_WHITE);
            tft.setTextSize(2);
            tft.fillRect(0, 100 + 5, tft.width(), 20, COLOR_BLACK);
            tft.drawCentreString("Bluetooth connected!", 120, 80 + 5, 1);
            bluetoothDisplayed = true;
        }
        SerialBT.println("Hello from ESP32!");
        SerialBT.println("Please enter Wifi_ID and Password in format: Wifi_ID,Password");
        Serial.println("Waiting to enter Wifi information...");
    } else {
        Serial.println("Wi-Fi connected!");
    }
}

void connectToWiFi() {
    Serial.println("Connecting to Wifi...");
    Serial.print("SSID: "); Serial.println(ssid);

    WiFi.mode(WIFI_STA);
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

    SerialBT.println("ESP32 connected to Wi-Fi!");
    Serial.println("Wifi connected!");
    SerialBT.println("Closing Bluetooth connection!");
    wifiConnected = true;
    Serial.print("IP Address: "); Serial.println(WiFi.localIP());
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(2);
    tft.fillRect(0, 100 + 5, tft.width(), 20, COLOR_BLACK);
    tft.drawCentreString("Wifi connected!", 120, 80 + 5, 1);

    // Dừng Bluetooth sau 3 giây
    customDelay(3000);
    SerialBT.end();
}

void handleBluetoothInput() {
    if (SerialBT.available()) {
        String value = SerialBT.readStringUntil('\n');
        value.trim();
        int commaIndex = value.indexOf(',');
        if (commaIndex > 0) {
            ssid = value.substring(0, commaIndex);
            password = value.substring(commaIndex + 1);
            connectToWiFi();
        } else {
            Serial.println("Invalid format, please try again.");
        }
    }
}

void loop() {
    static unsigned long beginMillis = 0;
    if (millis() - beginMillis > interval) {
        beginMillis = millis();
        if (!wifiConnected) {
            sendBluetoothMessages();
            handleBluetoothInput();
        } else {
            Serial.println("Wifi connected!");
            }

        float sumPower = 0;
        for (int i = 0; i < 8; i++)
        {
            float presentPower = 0;
            bool isActive = true;
            // check device is active.
            if (isActive)
            {
                presentPower = power[i];
            }
            presentPower = power[i];
            sumPower += presentPower;
            // Print power consumption for each device
            tft.setTextColor(COLOR_WHITE);
            tft.setTextSize(1);

            tft.drawCentreString(devices[i], 60, 100 + 45 + 17 * i, 2);
            tft.drawCentreString(" : ", 120, 100 + 45 + 17 * i, 2);

            char powerString[50];
            snprintf(powerString, sizeof(powerString), " %d W", (int)presentPower);
            tft.drawCentreString(powerString, 180, 100 + 45 + 17 * i, 2);
        }

        tft.drawCentreString("Total", 60, 100 + 45 + 17 * 8, 2);

        tft.drawCentreString(" : ", 120, 100 + 45 + 17 * 8, 2);
        char sumPowerString[50];
        snprintf(sumPowerString, sizeof(sumPowerString), " %d W", (int)sumPower);
        tft.drawCentreString(sumPowerString, 180, 100 + 45 + 17 * 8, 2);
    }
}

