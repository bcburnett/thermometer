#include <Arduino.h>
#include "bcbaws.h"
#include "bcbsdcard.h"
#include "bcbbmx.h"
#include "time.h"
#include "WiFiCred1.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <BMx280I2C.h>
#include "State.h"

#define ARDUINO_RUNNING_CORE 1

// internal rtc variables
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -18000;
const int daylightOffset_sec = 3600;

bool wifiavail = false;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// ST_CP pin 12
const int PROGMEM latchPin = 2;
// SH_CP pin 11
const int PROGMEM clockPin = 4;
// DS pin 14
const int PROGMEM dataPin = 15;
const unsigned long PROGMEM blink[] {0, 1, 3, 7, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191, 16383, 32767, 65535, 131071, 65535, 32767, 16383, 8191, 4095, 2047, 1023, 511, 255, 127, 63, 31, 15, 7, 3, 1, 0};
unsigned char bytes[4];
int count = 1;
// localtime from internal rtc
String printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return ("");
  }
  return (asctime(&timeinfo));
}

String printLocalHour() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return ("");
  }
  return (String(asctime(&timeinfo)).substring(11, 16) + " ");
}

// define functions
void UpdateClients(void *pvParameters); // maintains the websocket display
void UpdateDatabase(void *pvParameters); // maintains the websocket display
void MeasureData(void *pvParameters); // maintains the websocket display
void initWiFi();
void initTime();

void setup() {
  // start the serial interface
  Serial.begin(115200);
  delay(500);
  Wire.begin();
  initBmx280();

  initWiFi();
  initTime();
  initWebServer();
  initWebSocket();
  u8g2.begin();
  initSDCard();
  checkForIndex();
  state.reload = true;
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  xTaskCreatePinnedToCore(UpdateClients, "updateClients" // A name just for humans
                          ,
                          4096 // This stack size can be checked & adjusted by
                          // reading the Stack Highwater
                          ,
                          NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1)
                          // being the highest, and 0 being the lowest.
                          ,
                          NULL, ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(UpdateDatabase, "updateDatabase" // A name just for humans
                          ,
                          4096 // This stack size can be checked & adjusted by
                          // reading the Stack Highwater
                          ,
                          NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1)
                          // being the highest, and 0 being the lowest.
                          ,
                          NULL, ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(MeasureData, "measureData" // A name just for humans
                          ,
                          4096 // This stack size can be checked & adjusted by
                          // reading the Stack Highwater
                          ,
                          NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1)
                          // being the highest, and 0 being the lowest.
                          ,
                          NULL, ARDUINO_RUNNING_CORE);

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
    SPIFFS.end();
    // Disable client connections
    ws.enable(false);

    // Advertise connected clients what's going on
    ws.textAll("OTA Update Started");

    // Close them
    ws.closeAll();
    state.ota = true;


    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

void loop() {
  ArduinoOTA.handle(); // check if an update is available
  if (WiFi.status() != WL_CONNECTED) // check if wifi is still connected, if not, reconnect
    initWiFi();
  vTaskDelay(60);
}

void UpdateClients(void *pvParameters) { // handle websocket and oled displays
  (void)pvParameters;
  for (;;) {
    if (!state.ota) {
      notifyInitialClients(getJson(true)); // send state to the client as a json string
    }
    vTaskDelay(30000);
  }
}

void UpdateDatabase(void *pvParameters) { // handle websocket and oled displays
  (void)pvParameters;
  for (;;) {
    //if(!state.ota) {updateDB();} // send state to the mysql database
    u8g2.clearBuffer();              // clear the internal memory
    u8g2.setFont(u8g2_font_bauhaus2015_tn ); // choose a suitable font

    u8g2.drawStr(
      0, 33,
      printLocalHour().c_str()); // write something to the internal memory

    u8g2.setFont(u8g2_font_osb21_tf); // choose a suitable font
    

    u8g2.drawStr(
      10, 63,
      (String(state.temp)+ " F").c_str()); // write something to the internal memory

 

    u8g2.sendBuffer();              // transfer internal memory to the display
count ++;
    int n = std::lround(state.temp);
    Serial.println(n);
    
    unsigned long b = blink[ledValue(n)];
    Serial.println(b);
    bytes[0] = (b & 0xFF);
    bytes[1] = (b >> 8) & 0xFF;
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bytes[1]);
    shiftOut(dataPin, clockPin, LSBFIRST, bytes[0]);
    digitalWrite(latchPin, HIGH);
    delay(100);
    vTaskDelay(15000);
  }
}

int ledValue(int temp){
if(temp > 90) return 15;
if(temp > 85) return 14;
if(temp > 80) return 13;
if(temp > 75) return 12;
if(temp > 65) return 11;
if(temp > 55) return 10;
if(temp > 45) return 9;
if(temp > 35) return 8;
if(temp > 25) return 7;
if(temp > 15) return 6;
if(temp > 5) return 5;
if(temp > 0) return 4;
if(temp > -5) return 3;
if(temp > -20) return 2;
if(temp > -30) return 1;
return 0;
}

void MeasureData(void *pvParameters) { // handle websocket and oled displays
  (void)pvParameters;
  for (;;) {
    if (!state.ota) {
      doSensorMeasurement(); // send state to the mysql database
    }
    vTaskDelay(30000);
  }
}

void initWiFi() {
  Serial.println("connecting to wifi");
  // connect to wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("connecting to wifi failed");
    delay(500);
    initWiFi();
  }
  Serial.println("wifi connected");
  wifiavail = true;
}

void initTime() {
  // set the clock from ntp server
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
}
