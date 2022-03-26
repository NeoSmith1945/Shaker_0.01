#include <driver/adc.h>
#include <analogWriter.h>

// Include Adafruit PCA9685 Servo Library
#include "Servo_PCA9685.h"

#define PIN_RED    27 // GIOP25
#define PIN_GREEN  25 // GIOP27
#define PIN_BLUE   32 // GIOP32


extern const char* ssid   ;
extern const char* password ;

extern int count, waiting;


extern bool bLowVoltage ;

extern cServo_PCA9685 pca9685 ;

// 16 servo objects can be created on the ESP32
 

extern long shakeDurationMillisec;

extern char full_volt_string[10];
extern char cell_volt_string[10];
extern char cur_1_string[14];
extern char cur_2_string[14];

extern bool canBeStarted ;
extern bool canBeStopped ;

extern TaskHandle_t      Task_HW ;
extern TaskHandle_t      Task_CurrentMon ;
extern TaskHandle_t      Task_VoltageMon ;

extern SemaphoreHandle_t let_me_process;

extern String StateMsg;
extern float Voltage1;
extern float Voltage2  ; // Gets you mV
extern float Amps1 , Amps2  ;
extern float Amps1_Max, Amps2_Max  ;

extern String valueString4;
extern String valueString5;
extern String valueString6;

extern String valueVoltageCell00 ;
extern String valueVoltageTotal ;
extern String valueCur_1 ;
extern  String valueCur_2 ;

void blinkLED() ;

float fReadBatteryChannel_3( );
float fReadBatteryChannel_0( );

void BlinkRGB_LED (int PIN_LED, int times, int duration);

void current_monitor_task(void *param);

void shakeTaskPerTimer( void *param ) ;

void shakeTask( void *param ) ;

void VoltageMonTask( void *param );
