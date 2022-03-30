#include <driver/adc.h>
#include <analogWriter.h>
#include "WebServer.h"
#include "WebSocketsServer.h"
#include <ArduinoJson.h>                             

#include "SPIFFS.h"

// Include Adafruit PCA9685 Servo Library
#include "Servo_PCA9685.h"

#define PIN_RED    27 // GIOP25
#define PIN_GREEN  25 // GIOP27
#define PIN_BLUE   32 // GIOP32

struct Config {
  char instance[64];
  int  maxMajor[3];
  int  minMajor[3];
  int  maxSupport[3];
  int  minSupport[3];
  int  shakeMode;
  int initMajorMin;
  int initMajorMax;
  int initSupportMin ;
  int initSupportMax ;
  int CurPos0 ;
  int CurPos1  ;
  int MajorServMin;
  int MajorServMax;
  int SupportServMin; 
  int SupportServMax; 
  int count;
  int waiting;
  long shakeDurationMillisec ;
};

float fReadBatteryChannel_3( );
float fReadBatteryChannel_0( );
void blinkLED() ;
void BlinkRGB_LED (int PIN_LED, int times, int duration);
void current_monitor_task(void *param);
void shakeTaskPerTimer( void *param ) ;
void shakeTask( void *param ) ;
void VoltageMonTask( void *param );
void update_ui();
void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length);
void sendJson(String l_type, String l_value);
void loadConfiguration(const char *filename, Config &config);
