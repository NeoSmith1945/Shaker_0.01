// For installation, the following libraries need to be installed:
// * Websockets by Markus Sattler (can be tricky to find -> search for "Arduino Websockets"
// * ArduinoJson by Benoit Blanchon
//
// Written by neo (last update: 29.03.2022)

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "tools.h"
#include "Servo_PCA9685.h"

#include "credentials.h"

// const char* ssid      = "bla-bla"; 
// const char* password  = "blablabla";

#define I2C_SDA 21
#define I2C_SCL 22
 
bool bLowVoltage = false;

// Set web server port number to 80
WebServer server(80);
// Initialization of webserver and websocket
WebSocketsServer webSocket = WebSocketsServer(81);    // the websocket uses port 81 (standard port for websockets

cServo_PCA9685 pca9685 = cServo_PCA9685(0x40, Wire);
  
// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 

unsigned long  updateTime = 0;

// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

bool canBeStarted = false;
bool canBeStopped = false;

TaskHandle_t      Task_HW = NULL;
TaskHandle_t      Task_CurrentMon = NULL;
TaskHandle_t      Task_VoltageMon = NULL;

SemaphoreHandle_t let_me_process;

float Amps1_Max = 0 ;
float Amps2_Max = 0 ;

// The JSON library uses static memory, so this will need to be allocated:


Config config;                         // <- global configuration object

int clientCounter = 0;
String StrIndexHtml;
File index_html;

void setup() {
  File file;
  boolean usingSPIFFS_html = true;

  Serial.begin(115200);

  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
  } else {
    loadConfiguration("/config.json", config);
    if(String(config.instance) == "CONTROLLER") 
      index_html = SPIFFS.open("/index2.html");
    else
      index_html = SPIFFS.open("/index.html");
    if(!index_html){
      Serial.println("Failed to index.html file!");
      usingSPIFFS_html = false;
    } else
      StrIndexHtml = index_html.readString();
  }
  
  Serial.println("PCA9685 Servo Init with OTA: red");
  BlinkRGB_LED(PIN_RED, 2, 2000);
  Serial.println("PCA9685 Servo Init with OTA: green");
  BlinkRGB_LED(PIN_GREEN, 2, 2000);
  Serial.println("PCA9685 Servo Init with OTA: blue");
  BlinkRGB_LED(PIN_BLUE, 2, 2000);
  Wire.begin(I2C_SDA, I2C_SCL);
  // Print to monitor
  Serial.println("PCA9685 Servo Init with OTA: I2C OK");
  // Initialize PCA9685
  pca9685.begin();
  // Set PWM Frequency to 50Hz
  pca9685.setPWMFreq(FREQUENCY);  

  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);// using GPIO - SVN  ---> connected Volt 1
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);// using GPIO - SVP  ---> connected Volt 2
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);// using GPIO - IO34 ---> connected Strom 1
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);// using GPIO - IO33 ---> connected Strom 2
  
 // adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);// using GPIO - IO35  
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
 // ... Give ESP 10 seconds to connect to station.

  unsigned long startTime = millis();
  Serial.print("Waiting for wireless connection ");
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    analogWrite(PIN_BLUE,  847);
    delay(250);
    Serial.print(".");
    analogWrite(PIN_BLUE,  0);
    delay(250);
  }
  
  Serial.println();
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    BlinkRGB_LED(PIN_RED, 3, 2000);
    ESP.restart();
  }

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // OTA
  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"1234");
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);
  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");
  // No authentication by default
  // ArduinoOTA.setPassword("admin");
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
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
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("ArduinoOTA begin OK");    // print a message out in the serial port
     
  let_me_process = xSemaphoreCreateMutex();

  Serial.println("server begin OK");    // print a message out in the serial port
  xTaskCreate( VoltageMonTask, "Voltage_Monitor_Task", 50000, NULL, 1, &Task_VoltageMon);

  Serial.println("setup OK");    // print a message out in the serial port

  BlinkRGB_LED(PIN_GREEN, 3, 1000);

  pca9685.setCurPosMajor(config.CurPos0) ;
  pca9685.setCurPosSupport(config.CurPos1);
  pca9685.setMajorServMin(config.MajorServMin); 
  pca9685.setMajorServMax(config.MajorServMax); 
  pca9685.setSupportServMin(config.SupportServMin);
  pca9685.setSupportServMax(config.SupportServMax);

  BlinkRGB_LED(PIN_GREEN, 3, 1000);
 
  if (usingSPIFFS_html)  server.on("/", [](){ server.send(200,"text/html", StrIndexHtml);});
  else server.on("/", [](){ server.send(400,"text/html", String("Not found index.html"));});

  server.begin();                                     // start web server
  webSocket.begin();                                  // start websocket
  webSocket.onEvent(webSocketEvent);                  // define a callback function -> what does the ESP32 need to do when an event from the websocket is received? -> run function "webSocketEvent()"
}

void loop()
{
  unsigned long startTime = millis();
  ArduinoOTA.handle();
  
  if (bLowVoltage == false) {
    analogWrite(PIN_RED, 0);
    analogWrite(PIN_RED, 841);
  }
  server.handleClient();                              // Needed for the webserver to handle all clients
  webSocket.loop();                                   // Update function for the webSockets 
  
  if (bLowVoltage == false) analogWrite(PIN_BLUE, 841);

  if (clientCounter > 0) {
    if (( startTime - updateTime > 5000) || (updateTime == 0 )) {
      if (bLowVoltage == false) analogWrite(PIN_BLUE, 0);
      updateTime = startTime;
      update_ui();
    }  
  }
}