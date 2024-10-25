#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "BluetoothSerial.h"
#include "EEPROM.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Macro
#define EEPROM_SIZE 50
#define LENGTH(x) (strlen(x) + 1)
#define MAX_LENGTH 20
#define SSID_ADDRESS 0
#define PASSWORD_ADRESS (SSID_ADDRESS + MAX_LENGTH + 1)
#define MESSAGE_LENGTH 1024
#define VREF 3.3  // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point

// Connecting
const int deviceId = 4;
String hostname = "http://aqua-iot.pro/api/v1/sensordatas";
String twinurl = "http://aqua-iot.pro/api/v1/devices/" + String(deviceId);
const char *JSON_MIME = "application/json";
const char *TEXT_MIME = "text/plain";
HTTPClient http;
BluetoothSerial SerialBT;
char *ssid_ap = NULL;
char *pass_ap = NULL;

// Variables
float temperatureValue = 0;
float phValue = 0;
float tdsValue = 0;
int tdsBuffer[SCOUNT]; // store the TDS samples in the array, read from ADC
int pHBuffer[SCOUNT];
int ADC_Index = 0;
DynamicJsonDocument doc(MESSAGE_LENGTH);

// GPIOs
#define temperPin 4
#define tdsPin 33
#define pHPin 34
#define testPin 23
#define waterPin 5
#define valvePin 18
#define pumpPin 14
#define filtPin 15
#define LED 2
OneWire _1Wire(temperPin); // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature tempSensor(&_1Wire);

// Timer stamps
const int sendingPeriod = 4000U; // Khoang thoi gian giua cac lan gui du lieu len server
const int pollingCycle = 2000U;  // Thoi gian request lenh dieu khien tu server
const int printInterval = 2000U;
const int sampleInterval = 100U;  // Chu ky lay mau
const int updateInterval = 4000U; // Thoi gian tinh gia tri trung binh tu tap mau

// Flags
bool pumpEN = true;
bool filtEN = true;
bool MODE_AUTOMATE = true;
bool _BTstarted = false;
bool tried_to_reconnect = false;
bool full = false;