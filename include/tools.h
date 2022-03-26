#include <driver/adc.h>
#include <analogWriter.h>

// Include Adafruit PCA9685 Servo Library
#include "Servo_PCA9685.h"

#define PIN_RED    27 // GIOP25
#define PIN_GREEN  25 // GIOP27
#define PIN_BLUE   32 // GIOP32


extern const char* ssid   ;
extern const char* password ;

extern int MajorServMin, MajorServMax, SupportServMin, SupportServMax, count, waiting;

// Creat object to represent PCA9685 at default I2C address

// Define maximum and minimum number of "ticks" for the servo motors
// Range from 0 to 4095
// This determines the pulse width

#define FREQUENCY 250
#define SERVOMIN  500  // Minimum value
#define SERVOMAX  2500  // Maximum value
 
// Define servo motor connections (expand as required)
#define SER0  0   //Servo Motor 0 on connector 0
#define SER1  1  //Servo Motor 1 on connector 12


extern bool bLowVoltage ;

extern cServo_PCA9685 pca9685 ;
  
// Variables for Servo Motor positions (expand as required)

extern int initMajorMin ;
extern int initMajorMax ;

extern int initSupportMin ;
extern int initSupportMax ;

// 16 servo objects can be created on the ESP32
 
extern int pos ;    // variable to store the servo position

extern int CurPos0  ;
extern int CurPos1  ;

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
