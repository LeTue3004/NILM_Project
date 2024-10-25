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
#define SSID_ADDRESS 10
#define PASSWORD_ADRESS (SSID_ADDRESS + MAX_LENGTH + 1)
#define MESSAGE_LENGTH 1024
#define VREF 3.3  // analog reference voltage(Volt) of the ADC
#define SCOUNT 40 // sum of sample point

// Connecting
const int deviceId = 7;
String hostname = "https://aqua-iot.pro/api/v1/sensordatas";
String twinurl = "https://aqua-iot.pro/api/v1/devices/" + String(deviceId);
String imAlive = "https://aqua-iot.pro/api/v1/keepconnection";
const char *JSON_MIME = "application/json";
const char *TEXT_MIME = "text/plain";
HTTPClient http;
BluetoothSerial SerialBT;
String ssid_ap = "";
String pass_ap = "";

// Variables
float desiredTemper = 25;

float temperatureValue = 0;
float phValue = 0;
int tdsValue = 0;

float TemperArr[180];
float phArr[180];
int tdsArr[180];
int tempCount = 0;

int tdsBuffer[SCOUNT]; // store the TDS samples in the array, read from ADC
int pHBuffer[SCOUNT];
int ADC_Index = 0;
DynamicJsonDocument doc(MESSAGE_LENGTH);
float bias = 0.5;
int attempt = 0;

// GPIOs
#define pHPin 34
#define tdsPin 35
#define temperPin 32
#define valvePin 33
#define waterPin 25

#define pumpPin 23
#define filtPin 22
#define heatPin 21
#define chillPin 19
#define butt1Pin 18
#define butt2Pin 5
// #define butt3Pin 17
// #define butt4Pin 16

#define testPin 14

OneWire _1Wire(temperPin); // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature tempSensor(&_1Wire);

// Timer stamps
const int sendingPeriod = 60000U; // Khoang thoi gian giua cac lan gui du lieu len server
const int pollingCycle = 5000U;   // Thoi gian request lenh dieu khien tu server
const int warranty = 10000U;       // Keep Alive
const int sampleInterval = 1000U;  // Chu ky lay mau
const int updateInterval = 30000U; // Thoi gian tinh gia tri trung vi tu tap mau
const int printTimer = 100000U;

// Flags
bool btn1State = true;
bool btn2State = true;
bool btn3State = true;
bool btn4State = true;
bool released_1 = true;
bool released_2 = true;
bool pumpEN = true;
bool filtEN = true;
bool chillEN = false;
bool heatEN = false;
bool MODE_AUTOMATE = false;
bool alreadyRunning = false;
bool triedToReconnect = false;
bool gotNewWifi = false;
bool t0 = false;
bool t1 = false;
bool t2 = false;
bool debug = false;

// Function declaration
int getMedianNum(int bArray[]);
void takeSample();
void updateValue();
void writeString(const char *toStore, int startAddr);
String readStringFromFlash(int startAddr);
void wifiConnect(char *ssid, char *pass);

// Setup
void setup()
{
  // Set pin mode
  pinMode(temperPin, INPUT);
  pinMode(pHPin, INPUT);
  pinMode(tdsPin, INPUT);
  pinMode(waterPin, INPUT_PULLUP);
  pinMode(testPin, OUTPUT);
  pinMode(filtPin, OUTPUT);
  pinMode(valvePin, INPUT_PULLUP);
  pinMode(pumpPin, OUTPUT);
  pinMode(chillPin, OUTPUT);
  pinMode(heatPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(butt1Pin, INPUT_PULLUP);
  pinMode(butt2Pin, INPUT_PULLUP);
  // pinMode(butt3Pin, INPUT_PULLUP);
  // pinMode(butt4Pin, INPUT_PULLUP);

  // Start the serial monitor
  Serial.begin(115200);
  Serial.println(F("Initializing..."));

  EEPROM.begin(EEPROM_SIZE); // Init EEPROM

  SerialBT.begin("BecaAqua"); // Bluetooth device name
  Serial.println("Bluetooth started!");
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

// Main loop
void loop()
{

  static unsigned long lastTime = millis();
  static unsigned long lastPoll = millis();
  static unsigned long keppAlive = millis();
  static unsigned long updateTimePoint = millis();
  static unsigned long sampleTimePoint = millis();
  static unsigned long printTimePoint = millis();
  static unsigned long t0_TimePoint = 0;
  static unsigned long t1_TimePoint = 0;
  static unsigned long downDuration = 0;
  static unsigned long upDuration = 0;
  static unsigned long levelTimePoint = 0;

  // Sampling
  if (millis() - sampleTimePoint > sampleInterval)
  {
    sampleTimePoint = millis();
    takeSample();
  }
  if (millis() - updateTimePoint > updateInterval)
  {
    updateTimePoint = millis();
    updateValue();
  }

  // Controll water
  int overflow = digitalRead(valvePin);
  bool lowerflow = !digitalRead(waterPin);
  // Serial.print(overflow);
  // Serial.print("---");
  // Serial.println(lowerflow);

  if (overflow)
  {
    pumpEN = false;
    filtEN = true;

    if (debug)
    {
      debug = false;
      // Serial.print("-------");
      // Serial.print(millis() - levelTimePoint);
      // Serial.println("-------");
    }
    t0 = true;
    t0_TimePoint = millis();

    if (t1)
    {
      t2 = true;
      upDuration = t0_TimePoint - t1_TimePoint;
    }

    // Serial.print("[UpperValve]: ");
    // Serial.print(t0);
    // Serial.print("---");
    // Serial.println(t0_TimePoint);
  }
  if (lowerflow)
  {
    pumpEN = true;
    filtEN = false;

    if (t0)
    {
      t1 = true;
      t1_TimePoint = millis();
      downDuration = t1_TimePoint - t0_TimePoint;
      // Serial.print("[DownValve]: ");
      // Serial.print(t1);
      // Serial.print("---");
      // Serial.println(t1_TimePoint);
    }
  }
  if (t2 && (millis() - t0_TimePoint) > (downDuration / 2))
  {
    // Serial.print("[PLEASE]: ");
    // Serial.print(t0);
    // Serial.print("---");
    // Serial.println(millis() - t0_TimePoint);
    pumpEN = true;
    t0 = false;
    t1 = false;
    t2 = false;
    debug = true;
    levelTimePoint = millis();

    // Serial.println("OUT: ");
    // Serial.print(downDuration);
    // Serial.print("-----------");
    // Serial.print("IN: ");
    // Serial.println(upDuration);
  }

  // Control heater and chiller
  if (MODE_AUTOMATE)
  {
    digitalWrite(LED_BUILTIN, HIGH);

    if (temperatureValue > desiredTemper + bias)
    {
      chillEN = true;
    }
    if (temperatureValue < desiredTemper - bias && !lowerflow)
    {
      heatEN = true;
    }
    if (temperatureValue >= desiredTemper)
    {
      heatEN = false;
    }
    if (temperatureValue <= desiredTemper)
    {
      chillEN = false;
    }
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);

    btn1State = digitalRead(butt1Pin);
    btn2State = digitalRead(butt2Pin);
    // btn3State = digitalRead(butt3Pin);
    // btn4State = digitalRead(butt4Pin);

    // pumpEN = !btn1State;
    // filtEN = !btn2State;
    // heatEN = !btn1State ? !btn1State : heatEN;
    // chillEN = !btn2State ? !btn2State : chillEN;

    if (!btn1State)
    {
      bool temp = heatEN ? 0 : 1;
      heatEN = released_1 ? temp : heatEN;
      released_1 = false;
    }
    else
    {
      released_1 = true;
    }
    if (!btn2State)
    {
      bool temp = chillEN ? 0 : 1;
      chillEN = released_2 ? temp : chillEN;
      released_2 = false;
    }
    else
    {
      released_2 = true;
    }
  }

  digitalWrite(pumpPin, !pumpEN);
  digitalWrite(filtPin, !filtEN);
  digitalWrite(heatPin, !heatEN);
  digitalWrite(chillPin, !chillEN);
  // Serial.print(pumpEN);
  // Serial.print(filtEN);
  // Serial.print(heatEN);
  // Serial.println(chillEN);
  // Serial.println(digitalRead(buttPin));

  // Use Bluetooth to config Wifi
  if (SerialBT.available())
  {
    doc.clear();
    String hello = SerialBT.readStringUntil('\n');
    deserializeJson(doc, hello);
    JsonObject object = doc.as<JsonObject>();
    bool hasMethod = object.containsKey("method");
    if (hasMethod)
    {
      String method = object["method"];
      if (method.compareTo("ConnectToWifi") == 0)
      {
        if (object.containsKey("SSID") && object.containsKey("Password"))
        {
          String ssidBT = doc["SSID"];
          String passBT = doc["Password"];
          if (ssidBT != NULL && passBT != NULL)
          {
            // char *ssid_ap = new char[ssidBT.length() + 1];
            // char *pass_ap = new char[passBT.length() + 1];
            // strcpy(ssid_ap, ssidBT.c_str());
            // strcpy(pass_ap, passBT.c_str());
            Serial.println(hello);
            Serial.println(passBT);
            wifiConnect(&ssidBT[0], &passBT[0]);
            if (WiFi.status() == WL_CONNECTED)
            {
              SerialBT.println("WiFi connected");
              if (attempt < 10)
              {
                gotNewWifi = true;
              }
            }
            else
            {
              SerialBT.println("WiFi not connected");
            }
            triedToReconnect = true;
          }
        }
        else
        {
          SerialBT.println("[+] WiFi not connected, Missing SSID or Password!");
        }
      }
      else
      {
        SerialBT.println("[+] WiFi not connected, Invalid method name!");
      }
    }
    else
    {
      SerialBT.println("[+] WiFi not connected, Invalid string!");
    }
  }

  // Server communication
  if (WiFi.status() != WL_CONNECTED)
  {
    if (!triedToReconnect)
    {
      Serial.println("[+] Wifi disconnected!");
      SerialBT.println("Wifi not connected!");
      ssid_ap = readStringFromFlash(SSID_ADDRESS);
      pass_ap = readStringFromFlash(PASSWORD_ADRESS);
      wifiConnect(&ssid_ap[0], &pass_ap[0]);
      triedToReconnect = true;
    }
  }
  else
  {
    if (!alreadyRunning)
    {
      alreadyRunning = true;
      Serial.println(ssid_ap);
      Serial.printf("IP Address: ");
      Serial.println(WiFi.localIP());
      SerialBT.print("WiFi connected, IP address: ");
      SerialBT.println(WiFi.localIP());
    }
    if (triedToReconnect && gotNewWifi)
    {
      triedToReconnect = false;
      Serial.print("Saving password... ");
      writeString(&ssid_ap[0], SSID_ADDRESS);
      writeString(&pass_ap[0], PASSWORD_ADRESS);
      Serial.println("Done!");
    }

    if ((millis() - keppAlive) > warranty)
    {
      keppAlive = millis();
      doc.clear();
      doc["deviceId"] = deviceId;
      String aliveString = "";
      serializeJson(doc, aliveString);
      http.begin(imAlive.c_str());
      http.addHeader("Content-Type", JSON_MIME);
      http.addHeader("Connection", "keep-alive");
      http.addHeader("Accept", JSON_MIME);
      int respCode = http.POST(aliveString);
      if (respCode > 0)
      {
        //Serial.print("------------");
        String payload = http.getString();
        //Serial.println(payload);
      }
      else
      {
        //Serial.println("Ko gui duoc");
      }
      http.end();
    }

    /*  Phần 1: Gửi dữ liệu lên server
        Cấu trúc tin nhắn gửi dữ liệu:
        {
          "deviceId": int,
          "data": string (dạng json)
        }
    */
    if ((millis() - lastTime) > sendingPeriod)
    {
      lastTime = millis();
      //Serial.println("\n[+] Starting post method to send data:");
      // Tạo document để thực hiện serialize và deserialize json, xem thêm tại: https://arduinojson.org/
      // Bước 1: tạo chuỗi JSON cho trường data
      doc.clear();
      doc["nhietdo"] = serialized(String(temperatureValue, 1));
      doc["TDS"] = tdsValue;
      doc["pH"] = serialized(String(phValue, 1));
      String data_str = "";
      serializeJson(doc, data_str);
      // Bước 2: tạo tin nhắn dưới dạng JSON
      doc.clear();
      doc["deviceId"] = deviceId;
      doc["data"] = data_str;
      String sensorData = "";
      serializeJson(doc, sensorData);
      // Bước 3: Setup http client, bao gồm tên miền gửi lên và header
      http.begin(hostname.c_str());
      http.addHeader("Content-Type", JSON_MIME);
      http.addHeader("Connection", "keep-alive");
      http.addHeader("Accept", JSON_MIME);
      // Bước 4: Gửi dữ liệu lên, method là POST rồi in ra
      int respCode = http.POST(sensorData);
      //Serial.print("[+] Sending: ");
      //Serial.println(sensorData);
      if (respCode > 0)
      {
        //Serial.print("[+] Http Response code: ");
        //Serial.println(respCode);
        String payload = http.getString();
        //Serial.print("[+] Payload: ");
        //Serial.println(payload);
      }
      else
      {
        //Serial.printf("[+] Error code: ");
        //Serial.println(respCode);
      }
      http.end();
    }

    /*  Phần 2: Thực hiện nhận chuỗi điều khiển
        Cấu trúc của chuỗi điều khiển có dạng như sau:
        {
          "data": {
            "id": 3,
            "name": "espcong",
            "enabled": "Enabled",
            "connectionState": "Disconnected",
            "deviceToCloudMessages": 131,
            "cloudToDeviceMessages": 0,
            "desired": "{\"hello\": \"whatlad\", \"desired_ph\": \"7\", \"desired_temp\": \"20\", \"desired_humid\": \"85\"}",
            "reported": "{}"
          }
        }
    */
    if (millis() - lastPoll > pollingCycle)
    {
      lastPoll = millis();
      //Serial.println("\n[+] Starting get method to receive desired twin:");
      // Bước 1: Setup http client với hostname
      http.begin(twinurl.c_str());
      http.addHeader("Accept", JSON_MIME);
      // Bước 2: thực hiện phương thức GET để lấy dữ liệu
      int httpResponseCode = http.GET();
      // Bước 3: Lấy dữ liệu từ chuỗi JSON nhận về
      if (httpResponseCode > 0)
      {
        //Serial.print("[+] HTTP Response code: ");
        //Serial.println(httpResponseCode);
        String payload = http.getString();
        //Serial.println("[+] Payload: " + payload);
        // Phân tích dữ liệu trả về
        deserializeJson(doc, payload);
        // Lấy dữ liệu desired
        String desired = doc["data"]["desired"];
        //Serial.println("[+] Desired string: " + desired);
        doc.clear();
        // Biến dữ liệu desired thành object!
        deserializeJson(doc, desired);
        // Thay doi o day: Dieu khien dua vao du lieu nhan duoc
        if (!doc.isNull())
        {
          MODE_AUTOMATE = doc["autoMode"];
          desiredTemper = doc["setTemper"];
          if (!MODE_AUTOMATE)
          {
            heatEN = doc["heater"];
            chillEN = doc["chiller"];
          }
        }
      }
      else
      {
        //Serial.print("[+] Error code: ");
        //Serial.println(httpResponseCode);
      }
      http.end();
    }
  }

  /*
  if (millis() - printTimePoint > printTimer)
  {
    printTimePoint = millis();
    if (tempCount < 180)
    {
      TemperArr[tempCount] = temperatureValue;
      phArr[tempCount] = phValue;
      tdsArr[tempCount] = tdsValue;
      tempCount++;
      Serial.println("----------");
      for (int i = 0; i < tempCount; i++)
      {
        Serial.print(TemperArr[i]);
        Serial.print(",");
      }
      Serial.println("----------");
      for (int i = 0; i < tempCount; i++)
      {
        Serial.print(phArr[i]);
        Serial.print(",");
      }
      Serial.println("----------");
      for (int i = 0; i < tempCount; i++)
      {
        Serial.print(tdsArr[i]);
        Serial.print(",");
      }
      Serial.println("----------");
    }
    else
    {
      tempCount = 0;
    }
  }
  */
}

void wifiConnect(char *ssid, char *pass)
{
  WiFi.mode(WIFI_STA);
  attempt = 0;
  for (;;)
  {
    attempt++;
    WiFi.begin(ssid, pass);
    uint8_t status = WiFi.waitForConnectResult();
    if (status == WL_CONNECTED)
    {
      Serial.println("WiFi connected");
      ssid_ap = ssid;
      pass_ap = pass;
      break;
    }
    if (attempt == 10)
    {
      Serial.println("Reconnecting...");
      WiFi.begin(&ssid_ap[0], &pass_ap[0]);
      break;
    }
    delay(100);
  }
}

void writeString(const char *toStore, int startAddr)
{
  int i = 0;
  for (; i < LENGTH(toStore); i++)
  {
    EEPROM.write(startAddr + i, toStore[i]);
  }
  EEPROM.write(startAddr + i, '\0');
  EEPROM.commit();
}

String readStringFromFlash(int startAddr)
{
  char in[MAX_LENGTH];
  char curIn;
  int i = 0;
  curIn = EEPROM.read(startAddr);
  for (; i < MAX_LENGTH; i++)
  {
    curIn = EEPROM.read(startAddr + i);
    in[i] = curIn;
  }
  return String(in);
}

int getMedianNum(int bArray[])
{
  int iFilterLen = SCOUNT;
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
  {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else
  {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void takeSample()
{
  // Read temperature
  tempSensor.requestTemperatures();
  temperatureValue = tempSensor.getTempCByIndex(0);

  tdsBuffer[ADC_Index] = analogRead(tdsPin);
  pHBuffer[ADC_Index] = analogRead(pHPin);
  ADC_Index++;
  if (ADC_Index == SCOUNT)
  {
    ADC_Index = 0;
  }
}

void updateValue()
{
  int copyIndex;
  int bufferCopy[SCOUNT]; // Create a copy of the sample then pass to filter fuction

  for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
  {
    bufferCopy[copyIndex] = tdsBuffer[copyIndex];
  }
  // read the analog value more stable by the median filtering algorithm, and convert to voltage value
  float averageVoltage = getMedianNum(bufferCopy) * (float)VREF / 4096.0;
  // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  float compensationCoefficient = 1.0 + 0.02 * (temperatureValue - 25.0);
  // temperature compensation
  float compensationVoltage = averageVoltage / compensationCoefficient;
  // convert voltage value to tds value
  tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

  // update pH
  for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
  {
    bufferCopy[copyIndex] = pHBuffer[copyIndex];
  }
  float a = -0.0106;
  float b = 25.3;
  phValue = a * getMedianNum(bufferCopy) + b;
}