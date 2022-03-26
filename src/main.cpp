#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "SPIFFS.h"
#include "tools.h"
#include "Servo_PCA9685.h"


#define I2C_SDA 21
#define I2C_SCL 22
 
bool bLowVoltage = false;

// Set web server port number to 80
WiFiServer server(80);

const char* ssid     = "myNetworkNL";
const char* password = "31075979163814654332";

int MajorServMin, MajorServMax, SupportServMin, SupportServMax, count, waiting;

cServo_PCA9685 pca9685 = cServo_PCA9685(0x40, Wire);
  
// Variables for Servo Motor positions (expand as required)


int initMajorMin = 100;
int initMajorMax = 180;
int initSupportMin = 110;
int initSupportMax = 163;

// 16 servo objects can be created on the ESP32
 

int CurPos0 = 120 ;
int CurPos1 = 120 ;

// Variable to store the HTTP request
String header;

// Decode HTTP GET value
String valueString0 = String(initMajorMin);
String valueString1 = String(initMajorMax);
String valueString2 = String(initSupportMin);
String valueString3 = String(initSupportMax);
String valueString4 = String(0);
String valueString5 = String(20);
String valueString6 = String(0);

int pos01 = 0;
int pos02 = 0;
int pos11 = 0;
int pos12 = 0;
int pos21 = 0;
int pos22 = 0;
int pos31 = 0;
int pos32 = 0;
int pos41 = 0;
int pos42 = 0;
int pos51 = 0;
int pos52 = 0;
int pos61 = 0;
int pos62 = 0;

String valueVoltageCell00 ;
String valueVoltageTotal ;
String valueCur_1 ;
String valueCur_2 ;

// Current time
unsigned long currentTime = millis();

// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

long shakeDurationMillisec = 0;

char full_volt_string[10];
char cell_volt_string[10];
char cur_1_string[14];
char cur_2_string[14];

bool canBeStarted = false;
bool canBeStopped = false;

TaskHandle_t      Task_HW = NULL;
TaskHandle_t      Task_CurrentMon = NULL;
TaskHandle_t      Task_VoltageMon = NULL;

SemaphoreHandle_t let_me_process;

String StateMsg;

float Voltage1, Voltage2  ; // Gets you mV
float Amps1 , Amps2  ;
float Amps1_Max = 0 , Amps2_Max=0  ;


const int led = 2; // ESP32 Pin to which onboard LED is connected


uint8_t i;

bool ConnectionEstablished; // Flag for successfully handled connection

TaskHandle_t  CoreTaskHnd ; 

void setup() {
  File file;
  boolean usingSPIFFS;

  Serial.begin(115200);

  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    usingSPIFFS = false;
  } else {
    File file = SPIFFS.open("/config.json");
    if(!file){
      Serial.println("Fai+led to open file!");
      usingSPIFFS = false;
    }    
  }
  
  if (usingSPIFFS) {
    Serial.println("Content of file:");
    while(file.available()){
      Serial.write(file.read());
    }
    file.close();
  }

//  pinMode(PIN_RED,   OUTPUT);
//  pinMode(PIN_GREEN, OUTPUT);
//  pinMode(PIN_BLUE,  OUTPUT);

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
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000)
  {
    analogWrite(PIN_BLUE,  847);
    delay(250);
    Serial.print(".");
    analogWrite(PIN_BLUE,  0);
    delay(250);
  }
  
  Serial.println();
  
  while (WiFi.status() != WL_CONNECTED)
  {
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
 
  snprintf(full_volt_string, 8, "%2.2f", fReadBatteryChannel_3());
  snprintf(cell_volt_string, 8, "%2.2f", fReadBatteryChannel_0());

  valueVoltageCell00 = String( cell_volt_string ) ;
  valueVoltageTotal  = String( full_volt_string ) ;

  snprintf(cur_1_string, 12, "%2.2f/(%3d)", Amps1_Max,CurPos0);
  snprintf(cur_2_string, 12, "%2.2f/(%3d)", Amps2_Max,CurPos1);
  valueCur_1 = String( cur_1_string ) ;
  valueCur_2 = String( cur_2_string ) ;
    
  let_me_process = xSemaphoreCreateMutex();

  Serial.println("server begin OK");    // print a message out in the serial port
 
  server.begin();

  xTaskCreate( VoltageMonTask, "Voltage_Monitor_Task", 50000, NULL, 1, &Task_VoltageMon);

  Serial.println("setup OK");    // print a message out in the serial port

  BlinkRGB_LED(PIN_GREEN, 3, 1000);

  pca9685.setCurPosMajor(CurPos0) ;
  pca9685.setCurPosSuport(CurPos1);
  
  pca9685.setMajorServMin(MajorServMin); 
  pca9685.setMajorServMax(MajorServMax); 
  pca9685.setSupportServMin(SupportServMin);
  pca9685.setSupportServMax(SupportServMax);

  BlinkRGB_LED(PIN_GREEN, 3, 1000);
  
}

void loop()
{
  ArduinoOTA.handle();
 // Serial.println("ArduinoOTA handle OK");    // print a message out in the serial port
  if (bLowVoltage == false) {
    analogWrite(PIN_RED, 0);
    analogWrite(PIN_RED, 841);
  }
  
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    if (bLowVoltage == false) {
      analogWrite(PIN_RED, 0);
      analogWrite(PIN_BLUE, 841);
    }

    if (currentTime - previousTime > timeoutTime / 10) {
       Serial.printf( "\nVbatt(SVP/0) %f \n", fReadBatteryChannel_0());
       Serial.printf( "Vbatt(SVN/3) %f \n", fReadBatteryChannel_3());
    }

    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
                
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");       
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><meta http-equiv=\"refresh\" content=\"20\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>body { text-align: center; font-family: \"Trebuchet MS\", Arial; margin-left:auto; margin-right:auto;}");
            client.println(".slider { width: 300px; }</style>");
            client.println("<script src=\"https://ajax.googleapis.com/ajax/libs/jquery/3.3.1/jquery.min.js\"></script>");
                     
            // Web Page
            if (bLowVoltage == false) client.println("</head><body><h1>Let's shake the baby carriage</h1>");
            else client.println("</head><body><h1>Let's replace the battery first before shake the carriage</h1>");
            
            client.println("<p>Positions Servo Major (min,max): (<span id=\"servoPosOneMinS\"></span>,<span id=\"servoPosOneMaxS\"></span>)</p>");          
            client.println("<input type=\"range\" min=\"90\" max=\"180\" class=\"slider\" id=\"servoSliderOneMin\" onchange=\"servoOneMin(this.value)\" value=\""+valueString0+"\"/>");            
            client.println("<script>var slider0 = document.getElementById(\"servoSliderOneMin\");");
            client.println("var servoP0 = document.getElementById(\"servoPosOneMinS\"); servoP0.innerHTML = slider0.value;");
            client.println("slider0.oninput = function() { slider0.value = this.value; servoP0.innerHTML = this.value; }");      
            client.println("$.ajaxSetup({timeout:1000}); function servoOneMin(pos0) { ");
            client.println("$.get(\"/?value0=\" + pos0 + \"&\"); {Connection: close};}</script>");
            
            client.println("<input type=\"range\" min=\"90\" max=\"180\" class=\"slider\" id=\"servoSliderOneMax\" onchange=\"servoOneMax(this.value)\" value=\""+valueString1+"\"/>");            
            client.println("<script>var slider1 = document.getElementById(\"servoSliderOneMax\");");         
            client.println("var servoP1 = document.getElementById(\"servoPosOneMaxS\"); servoP1.innerHTML = slider1.value;");
            client.println("slider1.oninput = function() { slider1.value = this.value; servoP1.innerHTML = this.value; }");           
            client.println("$.ajaxSetup({timeout:1000}); function servoOneMax(pos1) { ");
            client.println("$.get(\"/?value1=\" + pos1 + \"&\"); {Connection: close};}</script>");
                  
            client.println("<p>Positions Servo Support (min,max):(<span id=\"servoPosTwoMinS\"></span>,<span id=\"servoPosTwoMaxS\"></span>)</p>");                     
            client.println("<input type=\"range\" min=\"90\" max=\"180\" class=\"slider\" id=\"servoSliderTwoMin\" onchange=\"servoTwoMin(this.value)\" value=\""+valueString2+"\"/>");            
            client.println("<script>var slider2 = document.getElementById(\"servoSliderTwoMin\");");
            client.println("var servoP2 = document.getElementById(\"servoPosTwoMinS\"); servoP2.innerHTML = slider2.value;");
            client.println("slider2.oninput = function() { slider2.value = this.value; servoP2.innerHTML = this.value; }");
            client.println("$.ajaxSetup({timeout:1000}); function servoTwoMin(pos2) { ");
            client.println("$.get(\"/?value2=\" + pos2 + \"&\"); {Connection: close};}</script>");

            client.println("<input type=\"range\" min=\"90\" max=\"180\" class=\"slider\" id=\"servoSliderTwoMax\" onchange=\"servoTwoMax(this.value)\" value=\""+valueString3+"\"/>");            
            client.println("<script>var slider3 = document.getElementById(\"servoSliderTwoMax\");"); 
            client.println("var servoP3 = document.getElementById(\"servoPosTwoMaxS\"); servoP3.innerHTML = slider3.value;");
            client.println("slider3.oninput = function() { slider3.value = this.value; servoP3.innerHTML = this.value; }");
            client.println("$.ajaxSetup({timeout:1000}); function servoTwoMax(pos3) { ");
            client.println("$.get(\"/?value3=\" + pos3 + \"&\"); {Connection: close};}</script>");

            client.println("<p>Duration (amount): <span id=\"durationAm\"></span></p>");          
            
            client.println("<input type=\"range\" min=\"0\" max=\"100\" class=\"slider\" id=\"shakeAmount\" onchange=\"servoAmounts(this.value)\" value=\""+valueString4+"\"/>");            
            client.println("<script>var slider4 = document.getElementById(\"shakeAmount\");");        
            client.println("var servoP4 = document.getElementById(\"durationAm\"); servoP4.innerHTML = slider4.value;");
            client.println("slider4.oninput = function() { slider4.value = this.value; servoP4.innerHTML = this.value; }");            
            client.println("$.ajaxSetup({timeout:1000}); function servoAmounts(pos4) { ");
            client.println("$.get(\"/?value4=\" + pos4 + \"&\"); {Connection: close};}</script>");

            client.println("<p>Or Duration (Minutes): <span id=\"durationM\"></span></p>");          
            client.println("<input type=\"range\" min=\"0\" max=\"45\" class=\"slider\" id=\"shakeDurationM\" onchange=\"servoDuration(this.value)\" value=\""+valueString6+"\"/>");            
            client.println("<script>var slider6 = document.getElementById(\"shakeDurationM\");");
            
            client.println("var servoP6 = document.getElementById(\"durationM\"); servoP6.innerHTML = slider6.value;");
            client.println("slider6.oninput = function() { slider6.value = this.value; servoP6.innerHTML = this.value; }");
            
            client.println("$.ajaxSetup({timeout:1000}); function servoDuration(pos6) { ");
            client.println("$.get(\"/?value6=\" + pos6 + \"&\"); {Connection: close};}</script>");


            client.println("<p>Speed (sec per step): <span id=\"speedS\"></span></p>");          

            client.println("<input type=\"range\" min=\"20\" max=\"45\" class=\"slider\" id=\"shakeSpeed\" onchange=\"servoSpeed(this.value)\" value=\""+valueString5+"\"/>");            
            client.println("<script>var slider5 = document.getElementById(\"shakeSpeed\");");
            
            client.println("var servoP5 = document.getElementById(\"speedS\"); servoP5.innerHTML = slider5.value;");
            client.println("slider5.oninput = function() { slider5.value = this.value; servoP5.innerHTML = this.value; }");
            
            client.println("$.ajaxSetup({timeout:1000}); function servoSpeed(pos5) { ");
            client.println("$.get(\"/?value5=\" + pos5 + \"&\"); {Connection: close};}</script>");

            client.println("<p>Power Supplay Cell/Total: <span id=\"servoPowerSup\"></span>");          
            client.println("<label for=\"cell00\">"+valueVoltageCell00+"V/</label><label for=\"cell00\">"+valueVoltageTotal+"V</label></p>");

            client.println("<p>Mux Current Sensor 1/2: <span id=\"servoCurrentConsum\"></span>");          
            client.println("<label for=\"currentl\">"+valueCur_1+"A/</label><label for=\"current2\">"+valueCur_2+"A</label></p>");

     //       client.println("<p>Status: <span id=\"StatusMsg\"></span><label for=\"statusMsg\">"+StateMsg+"</label></p>");

            client.println("<p><span id=\"Buttons\"></span></p>");          

            if (( canBeStarted == true )&&( bLowVoltage == false ))  {
              client.println("<input type=\"submit\" class=\"button\" id=\"btnStart\" onclick=\"startClick(this.value)\" value=\"START\"/>");            
              client.println("<script>var btnStar = document.getElementById(\"btnStart\");");
              client.println("$.ajaxSetup({timeout:1000}); function startClick(Submit) { ");
              client.println("$.post(\"/?valueStart=\" + Submit + \"&\"); {Connection: close};}</script>");
            }

            if ( canBeStopped == true )  {
               client.println("<input type=\"submit\" class=\"button\" id=\"btnStop\" onclick=\"stopClick(this.value)\" value=\"STOP\" />");            
               client.println("<script>var btnStop = document.getElementById(\"btnStop\");");
               client.println("$.ajaxSetup({timeout:1000}); function stopClick(Submit) { ");
               client.println("$.post(\"/?valueStop=\" + Submit + \"&\"); {Connection: close};}</script>");
            }

            
            client.println("</body></html>");     

            
            if(header.indexOf("POST /?valueStart=")>=0) {
              Serial.print("Start Submit fired:"); 
              Serial.println(valueString0); 
              Serial.println(valueString1); 
              Serial.println(valueString2); 
              Serial.println(valueString3); 
              Serial.println(valueString4); 
              Serial.println(valueString5);
              
              MajorServMin = valueString0.toInt();
              MajorServMax = valueString1.toInt();

              SupportServMin = valueString2.toInt();
              SupportServMax = valueString3.toInt();

              xSemaphoreTake( let_me_process, portMAX_DELAY );

              pca9685.setMajorServMin(MajorServMin); 
              pca9685.setMajorServMax(MajorServMax); 
              pca9685.setSupportServMin(SupportServMin);
              pca9685.setSupportServMax(SupportServMax);
              
              count = valueString4.toInt();
              waiting = valueString5.toInt();
              shakeDurationMillisec = valueString6.toInt()*1000*60;
              valueString4 = String(count);

              pca9685.setCount(count); 
              pca9685.setWaiting(waiting);
    
              if ((bLowVoltage == false) && (canBeStopped == false)) {
                Serial.println("shaking task is started......"); 
                if (count > 0)
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

            if(header.indexOf("POST /?valueStop=")>=0) {
              Serial.print("Stop Submit fired:"); 
              Serial.println(valueString0); 
              Serial.println(valueString1); 
              Serial.println(valueString2); 
              Serial.println(valueString3);
              if (canBeStopped) {
                canBeStarted = true; 
                canBeStopped = false;
                xSemaphoreGive(  let_me_process ); 
                if ( Task_CurrentMon !=  NULL) vTaskDelete(Task_CurrentMon);
                if ( Task_HW !=  NULL) vTaskDelete(Task_HW);
              }  
            }

           //GET /?value=180& HTTP/1.1
 
            if(header.indexOf("GET /?value0=")>=0) {
              pos01 = header.indexOf('=');
              pos02 = header.indexOf('&');
              valueString0 = header.substring(pos01+1, pos02);
              //Rotate the servo
              xSemaphoreTake( let_me_process, portMAX_DELAY ); 
                
              int new_pos = valueString0.toInt();

              pca9685.setMajorServMin(new_pos); 

              Serial.println(new_pos);
         
              xTaskCreate( current_monitor_task, "Current_Monitor_Task", 50000, NULL, 1, &Task_CurrentMon);
              
              pca9685.MoveMajorServoToNewPos(new_pos);
                            
              if (Task_CurrentMon != NULL) vTaskDelete(Task_CurrentMon);
        
              xSemaphoreGive( let_me_process );
              
              Serial.println(valueString0); 
            }         

            if(header.indexOf("GET /?value1=")>=0) {
              pos11 = header.indexOf('=');
              pos12 = header.indexOf('&');
              valueString1 = header.substring(pos11+1, pos12);
              //Rotate the servo
              xSemaphoreTake( let_me_process, portMAX_DELAY );
  
              int new_pos = valueString1.toInt();

              Serial.println(new_pos);
              pca9685.setMajorServMax(new_pos); 

              xTaskCreate( current_monitor_task, "Current_Monitor_Task", 50000, NULL, 1, &Task_CurrentMon);
              
              pca9685.MoveMajorServoToNewPos(new_pos);
             
              if (Task_CurrentMon !=  NULL) vTaskDelete(Task_CurrentMon);

              xSemaphoreGive( let_me_process );
            }         

            //GET /?value=180& HTTP/1.1
              
            if(header.indexOf("GET /?value2=")>=0) {
              Serial.print("Slider Two fire (min):"); 
              pos21 = header.indexOf('=');
              pos22 = header.indexOf('&');
              valueString2 = header.substring(pos21+1, pos22);

              int new_pos = valueString2.toInt();
              Serial.println(new_pos);
              pca9685.setSupportServMin(new_pos);

              //Rotate the servo
              xSemaphoreTake( let_me_process, portMAX_DELAY );
              
              xTaskCreate( current_monitor_task, "Current_Monitor_Task", 50000, NULL, 1, &Task_CurrentMon);
              pca9685.MoveSupportServoToNewPos(new_pos);
          
              vTaskDelete(Task_CurrentMon);

              xSemaphoreGive( let_me_process );
              Serial.println(valueString2); 
            }         

            if(header.indexOf("GET /?value3=")>=0) {
              Serial.print("Slider Two fire (max):"); 
              pos31 = header.indexOf('=');
              pos32 = header.indexOf('&');
              valueString3 = header.substring(pos31+1, pos32);
              int new_pos = valueString3.toInt();
              Serial.println(new_pos);
              pca9685.setSupportServMax(new_pos);
   
              //Rotate the servo
              xSemaphoreTake( let_me_process, portMAX_DELAY );
       
              xTaskCreate( current_monitor_task, "Current_Monitor_Task", 50000, NULL, 1, &Task_CurrentMon);
              pca9685.MoveSupportServoToNewPos(new_pos);
              vTaskDelete(Task_CurrentMon);

              xSemaphoreGive( let_me_process );
              Serial.println(valueString3); 
            }         

            if(header.indexOf("GET /?value4=")>=0) {
              pos41 = header.indexOf('=');
              pos42 = header.indexOf('&');
              valueString4 = header.substring(pos41+1, pos42);

              xSemaphoreTake( let_me_process, portMAX_DELAY ); 

              canBeStarted = true;

              count = valueString4.toInt();
              pca9685.setCount(count); 
              
              valueString4 = String(count);
              xSemaphoreGive( let_me_process );
            }         

            if(header.indexOf("GET /?value5=")>=0) {
              pos51 = header.indexOf('=');
              pos52 = header.indexOf('&');
              valueString5 = header.substring(pos51+1, pos52);

              xSemaphoreTake( let_me_process, portMAX_DELAY ); 
            
              waiting = valueString5.toInt();
              valueString5 = String(waiting);

              pca9685.setWaiting(waiting);

              canBeStarted = true;
              xSemaphoreGive( let_me_process );
            }        

            if(header.indexOf("GET /?value6=")>=0) {
              pos61 = header.indexOf('=');
              pos62 = header.indexOf('&');
              
              xSemaphoreTake( let_me_process, portMAX_DELAY ); 

              valueString6 = header.substring(pos61+1, pos62);
              Serial.println(valueString6);
              
              xSemaphoreGive( let_me_process );
               
            }          

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
      
    if (bLowVoltage == false) analogWrite(PIN_BLUE, 0);

    Serial.println("Client disconnected.");
    Serial.println("");

    snprintf(full_volt_string, 8, "%2.4f", fReadBatteryChannel_3());
    snprintf(cell_volt_string, 8, "%2.4f", fReadBatteryChannel_0());

    snprintf(cur_1_string, 12, "%2.2f/(%3d)", Amps1_Max,CurPos0);
    snprintf(cur_2_string, 12, "%2.2f/(%3d)", Amps2_Max,CurPos1);
    valueCur_1 = String( cur_1_string ) ;
    valueCur_2 = String( cur_2_string ) ;
 

    valueVoltageCell00 = String( cell_volt_string ) ;
    valueVoltageTotal  = String( full_volt_string ) ;
 
    StateMsg = "State: Cell 0(SVP/0):"+valueVoltageCell00 +"; Vbatt(SVN/3):"+valueVoltageTotal +"; Cur(1):"+valueCur_1+"A; Cur(2):"+valueCur_2+" A; count:"+String(count);

  }
}