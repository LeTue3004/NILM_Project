#include <Arduino.h>
#include <BluetoothSerial.h>
#include <WiFi.h>
#include "SPI.h"
#include <TFT_eSPI.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
HardwareSerial SerialPort(2);
// Use RX2, TX2 
// MQTT Broker information
const char* mqtt_broker = "fish.rmq.cloudamqp.com";
const int mqtt_port = 1883; 
const char* mqtt_username = "muxzhxjv:muxzhxjv"; 
const char* mqtt_password = "pMrQHB43ZQsjHKudlYcQPzcI2y7P9IwC"; 
// Khai báo đối tượng WiFi và MQTT client
WiFiClient espClient;
PubSubClient client(espClient);
int num = 0;

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

void setupMQTT() {
  client.setServer(mqtt_broker, mqtt_port);
  
  while (!client.connected()) {
    
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
    } else {
      delay(100);
    }
  }
  Serial.println("Server connected!");
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
  tft.fillRect(0, 80 + 5, tft.width(), 20, COLOR_BLACK);
  tft.drawCentreString("Server connected!", 120, 80 + 5, 1);
}

void setup() {
    Serial.begin(115200);
    SerialPort.begin(115200, SERIAL_8N1, 16, 17);// UART comunication

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
            tft.fillRect(0, 80 + 5, tft.width(), 20, COLOR_BLACK);
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
    tft.fillRect(0, 80 + 5, tft.width(), 20, COLOR_BLACK);
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
String invert_label_encode(int encoded_value) {
    String binary_string = String(encoded_value, BIN);
    while (binary_string.length() < 8) {
        binary_string = "0" + binary_string;  // Bổ sung các số 0 để đủ 8 bit
    }
    return binary_string;
}
String received = ""; // String received from esp32 
String binary_string  = "00000000"; // String received from esp32 to binary
String totalPower = "0";
void loop() {
    customDelay(10000);// 10s per time
    // esp32 communication
    if (SerialPort.available()) { // Kiểm tra xem có dữ liệu đến không
        received = SerialPort.readStringUntil('\n'); // Đọc dữ liệu đến ký tự newline
        received.trim();
        int commaIndex = received.indexOf(',');
        if (commaIndex > 0) {
            String encoded_devices = received.substring(0, commaIndex);
            int encoded_value = encoded_devices.toInt();
            totalPower = received.substring(commaIndex + 1);
        Serial.print("Received: "); // In ra thông báo
        Serial.println(received);    // In dữ liệu nhận được lên Serial Monitor
        binary_string = invert_label_encode(encoded_value);
        }
    }
    if (!wifiConnected) {
        sendBluetoothMessages();
        handleBluetoothInput();
    } else if (!client.connected()){
        Serial.println("Wifi connected!");
        setupMQTT();
    } else {
        client.loop();
        client.publish("data/1", received.c_str()); // Thay "your_topic" bằng tên chủ đề thực tế
        customDelay(1000);
        }
    float sumPower = 0;
    
    for (int i = 0; i < 8; i++)
    {
        float presentPower = 0;
        
        if (binary_string[i] == '1') // check device is active.
        {
            presentPower = power[i];
        }
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

