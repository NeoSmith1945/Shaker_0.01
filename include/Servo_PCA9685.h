#ifndef _Servo_PCA9685_H
#define _Servo_PCA9685_H

#include "Adafruit_PWMServoDriver.h"

#define FREQUENCY 250
#define SERVOMIN  500  // Minimum value
#define SERVOMAX  2500  // Maximum value
 
// Define servo motor connections (expand as required)
#define SER0  0   //Servo Motor 0 on connector 0
#define SER1  1  //Servo Motor 1 on connector 12

/*!
 *  @brief  Class that stores state and functions for interacting with PCA9685
 * PWM chip
 */
class cServo_PCA9685 {
public:
  cServo_PCA9685();
  cServo_PCA9685(const uint8_t addr);
  cServo_PCA9685(const uint8_t addr, TwoWire &i2c);
  void begin(uint8_t prescale = 0);
  void reset();
  void sleep();
  void wakeup();
  void setPWMFreq(float freq);
  uint8_t getPWM(uint8_t num);
  void setPWM(uint8_t num, uint16_t on, uint16_t off);
  void setExtClk(uint8_t prescale) ;
  void setPin(uint8_t num, uint16_t val, bool invert = false);
  uint8_t readPrescale(void);
  void writeMicroseconds(uint8_t num, uint16_t Microseconds);
  void MoveMajorServoToStart();
  void MoveMajorServoToNewPos(int newPos);
  void MoveSupportServoToStart();
  void MoveSupportServoToNewPos(int newPos);
  void setMajorServMin(int min); 
  void setMajorServMax(int max); 
  void setSupportServMin(int min);
  void setSupportServMax(int max);
  void setCount(int count); 
  void setWaiting(int waiting);
  void setCurPosMajor(int pos)  ;
  void setCurPosSuport(int pos)  ;
  void goAhead();
  void goBack();
  void setMinV(int min); 
  void setMaxV(int max); 
  int getMajorServMin(); 
  int getMajorServMax(); 
  int getSupportServMin();
  int getSupportServMax();
  int getCount(); 
  int getWaiting();
  int getCurPosMajor()  ;
  int getCurPosSuport()  ;

private:
  Adafruit_PWMServoDriver pca9685 ;

  int MajorServMin; 
  int MajorServMax; 
  int SupportServMin;
  int SupportServMax;

  int count; 
  int waiting;
  int MinV; 
  int MaxV; 
  
// Variables for Servo Motor positions (expand as required)
  int pwm;
  int CurPosMajor  ;
  int CurPosSuport  ;

  int angleToPulse(int ang);

};

#endif
