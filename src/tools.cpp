#include "tools.h"


extern Config config;


extern const char* ssid   ;
extern const char* password ;

extern bool bLowVoltage ;
extern bool canBeStarted ;
extern bool canBeStopped ;

extern WebServer server;

// Initialization of webserver and websocket
extern WebSocketsServer webSocket ;    // the websocket uses port 81 (standard port for websockets

extern cServo_PCA9685 pca9685 ;

extern int clientCounter ;

extern TaskHandle_t      Task_HW ;
extern TaskHandle_t      Task_CurrentMon ;
extern TaskHandle_t      Task_VoltageMon ;
extern SemaphoreHandle_t let_me_process;
extern float Amps1_Max, Amps2_Max  ;


const float r1 = 30000.0f; // R1 in ohm, 50K
const float r2 =  7500.0f; // R2 in ohm, 10k potentiometer
const long interval = 1000;  // interval at which to blink (milliseconds)
int ledState = LOW;  // ledState used to set the LED

String StateMsg;

float fReadBatteryChannel_3( )
{
  float adcValue = 0.0f;
  float Vbatt = 0.0f;
  float vRefScale = (3.3f / 4096.0f) * ((r1 + r2) / r2);
    
  adc1_get_raw(ADC1_CHANNEL_0); //read and discard
  adcValue = float( adc1_get_raw(ADC1_CHANNEL_0) ); //take a raw ADC reading
  Vbatt = (adcValue * vRefScale * 1.12f) - 0.25f;
  return  Vbatt;
}

float fReadBatteryChannel_0( )
{
  float adcValue = 0.0f;
  float Vbatt = 0.0f;

  float vRefScale = (3.3f / 4096.0f) * ((r1 + r2) / r2);
    
  adc1_get_raw(ADC1_CHANNEL_3); //read and discard
  adcValue = float( adc1_get_raw(ADC1_CHANNEL_3) ); //take a raw ADC reading
  Vbatt = (adcValue * vRefScale * 1.12f) - 0.50f ;
  return  Vbatt; 
}

void BlinkRGB_LED (int PIN_LED,int times, int duration){
    for (int ii=1; ii<times; ii++ ) {
      analogWrite(PIN_LED,   841); delay(duration);
      analogWrite(PIN_LED,   0); delay(duration);
    }
}

void shakeTask( void *param ) { 

  if ( fReadBatteryChannel_3() > 6.0f) {
    if ( pca9685.getMajorServMin() <= pca9685.getSupportServMin() )
      pca9685.setMinV(pca9685.getMajorServMin());
    else
      pca9685.setMinV(pca9685.getSupportServMin());

    if (pca9685.getMajorServMax() >= pca9685.getSupportServMax())
      pca9685.setMaxV(pca9685.getMajorServMax());
    else
      pca9685.setMaxV(pca9685.getSupportServMax());
  
    xTaskCreate( current_monitor_task, "Current_Monitor_Task", 50000, NULL, 2, &Task_CurrentMon);

    pca9685.MoveMajorServoToStart();
    pca9685.MoveSupportServoToStart();
  }
  canBeStopped = true;
  int count = config.count;
  while (count > 0)  {
    count--;
    int waiting = config.waiting;
    if ( fReadBatteryChannel_3() > 6.0f) {
      pca9685.setWaiting(waiting);
      pca9685.goAhead();
      pca9685.setWaiting(waiting);
      pca9685.goBack();
      StateMsg = "shakeTask current count = "+String(count);
      sendJson("SERVO_CURENT_STATUS", StateMsg);
      sendJson("SERVO_SHAKE_AMOUNT", String(count));
      sendJson("DISABLE_START_BTN", String("true"));
      sendJson("ENABLE_STOP_BTN", String("true"));
    } else {
        StateMsg = "shake: LiPo Acu is getting below 6V, servo is off";
        Serial.println(StateMsg);
        sendJson("SERVO_CURENT_STATUS", StateMsg);
        vTaskDelay ( waiting / portTICK_PERIOD_MS);
        config.count = 0;
    }
  }    
  canBeStopped = false;
  StateMsg = "shakeTask terminated: count = "+String(config.count);
  sendJson("SERVO_CURENT_STATUS", StateMsg);
  sendJson("DISABLE_STOP_BTN", String("true"));
  sendJson("ENABLE_START_BTN", String("true"));
  if ( Task_CurrentMon != NULL) vTaskDelete(Task_CurrentMon);
  vTaskDelete(NULL);
}

void shakeTaskPerTimer( void *param ) {
  unsigned long startTime = millis();
 // Previous time
  unsigned long currentTime = startTime; 

  StateMsg = "shakeTask started: duration (millisec/min) = "+String(config.shakeDurationMillisec)+"/"+String(config.shakeDurationMillisec/60000);
  Serial.println(StateMsg);

  if ( fReadBatteryChannel_3() > 6.0f) {
    if ( pca9685.getMajorServMin() <= pca9685.getSupportServMin() ) 
      pca9685.setMinV(pca9685.getMajorServMin());
    else
      pca9685.setMinV(pca9685.getSupportServMin());

    if (pca9685.getMajorServMax() >= pca9685.getSupportServMax())
      pca9685.setMaxV(pca9685.getMajorServMax());
    else
      pca9685.setMaxV(pca9685.getSupportServMax());
  
    xTaskCreate( current_monitor_task, "Current_Monitor_Task", 50000, NULL, 1, &Task_CurrentMon);
    pca9685.MoveMajorServoToStart();
    pca9685.MoveSupportServoToStart();
  }
 
  canBeStopped = true;

  while ( (currentTime - startTime) < config.shakeDurationMillisec )  {
    
    int waiting = config.waiting;
  
    currentTime = millis();
    StateMsg = "shakeTask soll millisec:"+String(config.shakeDurationMillisec)+"; is millisec/sec:"+String(currentTime - startTime)+"/"+String((currentTime - startTime)/1000);
    Serial.println(StateMsg);
    String valueString6 = String(int((config.shakeDurationMillisec - (currentTime - startTime))/60000));   

    sendJson("SERVO_CURENT_STATUS", StateMsg);
    
    if ( fReadBatteryChannel_3() > 6.0f) {
      pca9685.setWaiting(waiting);
      pca9685.goAhead();
      pca9685.setWaiting(waiting);
      pca9685.goBack();
      sendJson("SERVO_SHAKE_DURATION", valueString6);
      sendJson("SERVO_CURENT_STATUS", StateMsg);
      sendJson("DISABLE_START_BTN", String("true"));
      sendJson("ENABLE_STOP_BTN", String("true"));
    } else {
      StateMsg = "LiPo Acu is getting below 6V, servo is off";
      sendJson("SERVO_CURENT_STATUS", StateMsg);
      Serial.println(StateMsg);      
      vTaskDelay ( (waiting*100) / portTICK_PERIOD_MS);
    }     
  }
  
  canBeStopped = false;
  StateMsg = "shakeTask terminated after (millisec/sec) = "+String(currentTime - startTime)+"/"+String((currentTime - startTime)/1000);
  Serial.println(StateMsg);
  sendJson("SERVO_CURENT_STATUS", StateMsg);
  sendJson("DISABLE_STOP_BTN", String("true"));
  sendJson("ENABLE_START_BTN", String("true"));
  if ( Task_CurrentMon != NULL ) vTaskDelete(Task_CurrentMon);
  vTaskDelete(NULL);
}

void current_monitor_task(void *param)
{ float Voltage1, Voltage2  ; // Gets you mV
  float Amps1 , Amps2  ;
  float ACSValue1 = 0.0, Samples1 = 0.0, AvgACS1 = 0.0,  BaseVol = 2.08f ;
  float ACSValue2 = 0.0, Samples2 = 0.0, AvgACS2 = 0.0 ;
  float vRefScale = (3.3f / 4096.0f) ; 

  Voltage1 = 0; 
  Voltage2 = 0 ; // Gets you mV
  Amps1 = 0; 
  Amps2 = 0 ;
  
  while (true)  {
    ACSValue1 = 0.0;
    Samples1 = 0.0; 
    AvgACS1 = 0.0;
    ACSValue2 = 0.0;
    Samples2 = 0.0; 
    AvgACS2 = 0.0;
  
    for (int x = 0; x < 5; x++) { //This would take 500 Samples
      adc1_get_raw(ADC1_CHANNEL_6); //read and discard using GPIO - IO34
      ACSValue1 = float( adc1_get_raw(ADC1_CHANNEL_6) ); //take a raw ADC reading
      Samples1 = Samples1 + ACSValue1;

      adc1_get_raw(ADC1_CHANNEL_5); //read and discard using GPIO - IO33
      ACSValue2 = float( adc1_get_raw(ADC1_CHANNEL_5) ); //take a raw ADC reading
      Samples2 = Samples2 + ACSValue2;
      vTaskDelay ( 300 / portTICK_PERIOD_MS);
    }  
    
    AvgACS1 = Samples1/5;
    Voltage1 = AvgACS1 * vRefScale - BaseVol ; // Gets you mV
    
    Amps1 = ( Voltage1 / 1.32 );

    if (Amps1 >= Amps1_Max) Amps1_Max = Amps1;
     
    AvgACS2 = Samples2/5;
    Voltage2 = AvgACS2 * vRefScale - BaseVol ; // Gets you mV
    Amps2 = ( Voltage2 / 1.32 );

    if (Amps2 >= Amps2_Max) Amps2_Max = Amps2;

    if ( Amps2_Max > 2.5 ) {
      StateMsg = "current monitor: Max current(3) more than 2.5 A, servo will be off";
      sendJson("SERVO_CURENT_STATUS", StateMsg);
      canBeStopped = false;
      xSemaphoreGive(  let_me_process ); 
      if ( Task_HW != NULL ) vTaskDelete(Task_HW);
      vTaskDelete(NULL);
    }

    if ( Amps1_Max > 2.5 ) {
      StateMsg = "current monitor: Max current(1) more than 2.5 A, servo will be off";
      sendJson("SERVO_CURENT_STATUS", StateMsg);
      canBeStopped = false;
      xSemaphoreGive(  let_me_process ); 
      if ( Task_HW != NULL ) vTaskDelete(Task_HW);
      vTaskDelete(NULL);      
    }
    vTaskDelay ( 50 / portTICK_PERIOD_MS);
  } 
}


void VoltageMonTask( void *param ) {

  for(;;) {
     if ( fReadBatteryChannel_3() < 6.5f) break; 
     vTaskDelay ( 7000 / portTICK_PERIOD_MS);
  }
          
  for(;;) {
    bLowVoltage = true;
    if ( Task_CurrentMon != NULL) vTaskDelete(Task_CurrentMon);
    if ( Task_HW !=  NULL) vTaskDelete(Task_HW);
    BlinkRGB_LED(PIN_RED, 5, 1000);
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

// Loads the configuration from a file
void loadConfiguration(const char *filename, Config &config) {
  uint8_t i;
  // Open file for reading
  File file = SPIFFS.open(filename);

  if(!file){
      Serial.println("− failed to open file config");
  }

  StaticJsonDocument<1024> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error){
    Serial.println(F("Failed to deserialize Json from file, using default configuration"));
    config.OTA_Switch = true; 
  } else{
    if (doc["OTA"] == "enabled") 
      config.OTA_Switch = true;
    else 
      config.OTA_Switch = false;
  }
  // Close the file (Curiously, File's destructor doesn't close the file)
  
  file.close();

  // Copy values from the JsonDocument to the Config
  for (i=0;i<3;i++) {
    config.maxMajor[i] = doc["maxMajor"][i] | 150;
    config.minMajor[i] = doc["minMajor"][i] | 95;
    config.maxSupport[i] = doc["maxSup"][i] | 160;
    config.minSupport[i] = doc["minSup"][i] | 120;
  }

  strlcpy(config.instance, doc["instance"] | "CONTROLLER",  // <- source
              sizeof(config.instance));         // <- destination's capacity
  strlcpy(config.NetWorkType, doc["NetWork"] | "public",  // <- source
              sizeof(config.NetWorkType));         // <- destination's capacity
  strlcpy(config.Privat_ssid, doc["Privat_SSID"] | "ShutlerNet",  // <- source
              sizeof(config.Privat_ssid));         // <- destination's capacity
  strlcpy(config.Privat_pass, doc["Privat_PASS"] | "12345678",  // <- source
              sizeof(config.Privat_pass));         // <- destination's capacity
  
  config.shakeMode = 0;
  config.initMajorMin = 100;
  config.initMajorMax = 180;
  config.initSupportMin = 110;
  config.initSupportMax = 163;
// 16 servo objects can be created on the ESP32
  config.CurPos0 =  doc["MajorServoPos"] | 140;
  config.CurPos1 = doc["SupServoPos"] | 130;

  
}

// Simple function to send information to the web clients
void sendJson(String l_type, String l_value) {
  StaticJsonDocument<500> doc_tx;

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
  StaticJsonDocument<200> doc_rx;
  StaticJsonDocument<500> doc_tx;

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
