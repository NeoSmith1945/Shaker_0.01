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

#include "WebServer.h"
#include "WebSocketsServer.h"
#include <ArduinoJson.h>                             

#include "SPIFFS.h"
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
StaticJsonDocument<200> doc_rx;
StaticJsonDocument<500> doc_tx;


Config config;                         // <- global configuration object

int clientCounter = 0;
String StrIndexHtml;
File index_html;

// Loads the configuration from a file
void loadConfiguration(const char *filename, Config &config) {
  uint8_t i;
  // Open file for reading
  File file = SPIFFS.open(filename);

  StaticJsonDocument<512> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  // Copy values from the JsonDocument to the Config
  for (i=0;i<3;i++) {
    config.maxMajor[i] = doc["maxMajor"][i] | 150;
    config.minMajor[i] = doc["minMajor"][i] | 95;
    config.maxSupport[i] = doc["maxSupport"][i] | 140;
    config.minSupport[i] = doc["minSupport"][i] | 95;
  }
  strlcpy(config.instance, doc["instance"] | "controller",  // <- source
              sizeof(config.instance));         // <- destination's capacity
  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();

  config.shakeMode = 0;
  config.initMajorMin = 100;
  config.initMajorMax = 180;
  config.initSupportMin = 110;
  config.initSupportMax = 163;
// 16 servo objects can be created on the ESP32
  config.CurPos0 = 120 ;
  config.CurPos1 = 120 ;
}

// Simple function to send information to the web clients
void sendJson(String l_type, String l_value) {
  if (clientCounter>0) {
    String jsonString = "";                           // create a JSON string for sending data to the client
    JsonObject object = doc_tx.to<JsonObject>();      // create a JSON Object
    object["type"] = l_type;                          // write data into the JSON object -> I used "type" to identify if LED_selected or LED_intensity is sent and "value" for the actual value
    object["value"] = l_value;
    serializeJson(doc_tx, jsonString);                // convert JSON object to string
    webSocket.broadcastTXT(jsonString);               // send JSON string to all clients
  }
}

void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length) {      // the parameters of this callback function are always the same -> num: id of the client who send the event, type: type of message, payload: actual data sent and length: length of payload
  String StateMsg;
  
  switch (type) {                                     // switch on the type of information sent
    case WStype_DISCONNECTED:                         // if a client is disconnected, then type == WStype_DISCONNECTED
      Serial.println("Client " + String(num) + " disconnected");
      clientCounter --;
      break;
    case WStype_CONNECTED:                            // if a client is connected, then type == WStype_CONNECTED
      Serial.println("Client " + String(num) + " connected");
      clientCounter ++;
      sendJson("SERVO_intensity", String(config.waiting));
      sendJson("MOD_selected", String(config.shakeMode));
      break;
    case WStype_FRAGMENT_TEXT_START:                                 // if a client has sent data, then type == WStype_TEXT
      Serial.println("Client " + String(num) + " send fragment text");
      break;   
    case WStype_TEXT:                                 // if a client has sent data, then type == WStype_TEXT
      // try to decipher the JSON string received
      DeserializationError error = deserializeJson(doc_rx, payload);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }
      else {
        // JSON string was received correctly, so information can be retrieved:
        const char* l_type = doc_rx["type"];
        const int l_value = doc_rx["value"];
        Serial.println("Type: " + String(l_type));
        Serial.println("Value: " + String(l_value));

        // if LED_intensity value is received -> update and write to LED
        if(String(l_type) == "SERVO_intensity") {
          xSemaphoreTake( let_me_process, portMAX_DELAY ); 
          config.waiting = int(l_value);
          sendJson("SERVO_intensity", String(l_value));
          pca9685.setWaiting(config.waiting);
          canBeStarted = true;
          xSemaphoreGive( let_me_process );
        }
        // else if LED_select is changed -> switch on LED and switch off the rest
        if(String(l_type) == "MOD_selected") {
          config.shakeMode = int(l_value);
          sendJson("MOD_selected", String(l_value));
          config.MajorServMin = config.minMajor[config.shakeMode]; 
          config.MajorServMax = config.maxMajor[config.shakeMode];
          config.SupportServMin = config.minSupport[config.shakeMode];
          config.SupportServMax = config.maxSupport[config.shakeMode];
          StateMsg = "Mode:"+String(config.shakeMode) +"- Range:"+String(config.MajorServMax)+"-"+String(config.MajorServMin)+";"+String(config.SupportServMax)+"-"+String(config.SupportServMin);
          sendJson("SERVO_CURENT_STATUS", StateMsg);
        }
        if(String(l_type) == "SERVO_POS_ONE_MIN") {
          xSemaphoreTake( let_me_process, portMAX_DELAY ); 
          config.MajorServMin = int(l_value);
          Serial.println(config.MajorServMin);
          sendJson("SERVO_POS_ONE_MIN", String(l_value));
          pca9685.setMajorServMin(config.MajorServMin); 
          xTaskCreate( current_monitor_task, "Current_Monitor_Task", 50000, NULL, 1, &Task_CurrentMon);
          pca9685.MoveMajorServoToNewPos(config.MajorServMin);            
          if (Task_CurrentMon != NULL) vTaskDelete(Task_CurrentMon);
          xSemaphoreGive( let_me_process );
        }
        if(String(l_type) == "SERVO_POS_ONE_MAX") {
          xSemaphoreTake( let_me_process, portMAX_DELAY );
          config.MajorServMax = int(l_value);
          Serial.println(config.initMajorMax);
          sendJson("SERVO_POS_ONE_MAX", String(l_value));          
          pca9685.setMajorServMax(config.MajorServMax); 
          xTaskCreate( current_monitor_task, "Current_Monitor_Task", 50000, NULL, 1, &Task_CurrentMon);
          pca9685.MoveMajorServoToNewPos(config.MajorServMax);
          if (Task_CurrentMon !=  NULL) vTaskDelete(Task_CurrentMon);
          xSemaphoreGive( let_me_process );
        }
        if(String(l_type) == "SERVO_POS_TWO_MIN") {
          xSemaphoreTake( let_me_process, portMAX_DELAY ); 
          config.SupportServMin = int(l_value);
          sendJson("SERVO_POS_TWO_MIN", String(l_value));
          Serial.println(config.SupportServMin);
          pca9685.setSupportServMin(config.SupportServMin);
          //Rotate the servo
          xTaskCreate( current_monitor_task, "Current_Monitor_Task", 50000, NULL, 1, &Task_CurrentMon);
          pca9685.MoveSupportServoToNewPos(config.SupportServMin);
          vTaskDelete(Task_CurrentMon);
          xSemaphoreGive( let_me_process );
        }
        if(String(l_type) == "SERVO_POS_TWO_MAX") {
          xSemaphoreTake( let_me_process, portMAX_DELAY );
          config.SupportServMax = int(l_value);
          sendJson("SERVO_POS_TWO_MAX", String(l_value));
          Serial.println(config.SupportServMax);
          pca9685.setSupportServMax(config.SupportServMax);
          //Rotate the servo
          xTaskCreate( current_monitor_task, "Current_Monitor_Task", 50000, NULL, 1, &Task_CurrentMon);
          pca9685.MoveSupportServoToNewPos(config.SupportServMax);
          vTaskDelete(Task_CurrentMon);
          xSemaphoreGive( let_me_process );
        }
        if(String(l_type) == "SERVO_SHAKE_AMOUNT") {
          xSemaphoreTake( let_me_process, portMAX_DELAY ); 
          config.count = int(l_value);
          sendJson("SERVO_SHAKE_AMOUNT", String(l_value));
          Serial.println(config.count);
          canBeStarted = true;
          pca9685.setCount(config.count); 
          xSemaphoreGive( let_me_process );
        }
        if(String(l_type) == "SERVO_SHAKE_DURATION") {
          xSemaphoreTake( let_me_process, portMAX_DELAY ); 
          config.shakeDurationMillisec = int(l_value)*1000*60;
          sendJson("SERVO_SHAKE_DURATION", String(l_value));
          Serial.println(config.shakeDurationMillisec);
          xSemaphoreGive( let_me_process );
        }
        if(String(l_type) == "SERVO_SHAKE_SPEED") {
          xSemaphoreTake( let_me_process, portMAX_DELAY ); 
          config.waiting = int(l_value);
          sendJson("SERVO_SHAKE_SPEED", String(l_value));          
          pca9685.setWaiting(config.waiting);
          canBeStarted = true;
          xSemaphoreGive( let_me_process );
        }
        if(String(l_type) == "SERVO_STOP") {
          xSemaphoreTake( let_me_process, portMAX_DELAY ); 
          sendJson("SERVO_STOP", String(l_value));           
          Serial.print("Stop Submit fired"); 
          if (canBeStopped) {
            canBeStarted = true; 
            canBeStopped = false;
            if ( Task_CurrentMon !=  NULL) vTaskDelete(Task_CurrentMon);
            if ( Task_HW !=  NULL) vTaskDelete(Task_HW);
          }  
          xSemaphoreGive( let_me_process );
        }

        if(String(l_type) == "SERVO_START") {
          xSemaphoreTake( let_me_process, portMAX_DELAY ); 
          sendJson("SERVO_START", String(l_value));          

          Serial.print("Start Submit fired:"); 
          
          pca9685.setMajorServMin(config.MajorServMin); 
          pca9685.setMajorServMax(config.MajorServMax); 
          pca9685.setSupportServMin(config.SupportServMin);
          pca9685.setSupportServMax(config.SupportServMax);
          
          pca9685.setCount(config.count); 
          pca9685.setWaiting(config.waiting);
    
          if ((bLowVoltage == false) && (canBeStopped == false)) {
            Serial.println("shaking task is started......"); 
            if (config.count > 0)
              xTaskCreatePinnedToCore( shakeTask, "shakeTaskPerCount", 10000, NULL, 1, &Task_HW, 1);
            else 
              xTaskCreatePinnedToCore( shakeTaskPerTimer, "shakeTaskPerTimer", 10000, NULL, 1, &Task_HW, 1);

            canBeStopped = true;
            canBeStarted = false;
          } else {
            Serial.println("shaking task is already running"); 
          }          
          xSemaphoreGive( let_me_process ); 
        }
      }
      break;
  }
}

void update_ui(){
  char tmp_buffer[10];
  
  snprintf(tmp_buffer, 8, "%2.4f", fReadBatteryChannel_0());
  sendJson("SERVO_POWER_VOLTAGE_CELL", String(tmp_buffer));  
  snprintf(tmp_buffer, 8, "%2.4f", fReadBatteryChannel_3());
  sendJson("SERVO_POWER_VOLTAGE_ALL", String(tmp_buffer));
  snprintf(tmp_buffer, 8, "%2.2f", Amps1_Max);
  sendJson("SERVO_ONE_MAX_CURRENT", String(tmp_buffer));
  snprintf(tmp_buffer, 8, "%2.2f", Amps2_Max);
  sendJson("SERVO_TWO_MAX_CURRENT", String(tmp_buffer));
  snprintf(tmp_buffer, 8, "%3d", pca9685.getCurPosMajor());
  sendJson("SERVO_ONE_CURRENT_POS", String(tmp_buffer));
  snprintf(tmp_buffer, 8, "%3d", pca9685.getCurPosSupport());  
  sendJson("SERVO_TWO_CURRENT_POS", String(tmp_buffer));
}

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

unsigned long  updateTime = 0;

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