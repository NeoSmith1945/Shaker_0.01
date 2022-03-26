 /*!  @file servoPCA9685.cpp
 *
 *  @mainpage Adafruit 16-channel PWM & Servo driver
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit PWM & Servo driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These displays use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "Adafruit_PWMServoDriver.h"
#include "Servo_PCA9685.h"
#include <Wire.h>

//#define ENABLE_DEBUG_OUTPUT

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
 * TwoWire interface
 */
cServo_PCA9685::cServo_PCA9685() {
    pca9685 = Adafruit_PWMServoDriver();
    MinV = 0;
    MaxV = 0;
}

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
 * TwoWire interface
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 */
cServo_PCA9685::cServo_PCA9685(const uint8_t addr) {
    pca9685 = Adafruit_PWMServoDriver(addr);
}

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
 * TwoWire interface
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 *  @param  i2c  A reference to a 'TwoWire' object that we'll use to communicate
 *  with
 */
cServo_PCA9685::cServo_PCA9685(const uint8_t addr, TwoWire &i2c) {
    pca9685 = Adafruit_PWMServoDriver(addr, i2c);
}

/*!
 *  @brief  Setups the I2C interface and hardware
 *  @param  prescale
 *          Sets External Clock (Optional)
 */
void cServo_PCA9685::begin(uint8_t prescale) {
    pca9685.begin(prescale);
}

/*!
 *  @brief  Sends a reset command to the PCA9685 chip over I2C
 */
void cServo_PCA9685::reset() {
    pca9685.reset();
}

/*!
 *  @brief  Puts board into sleep mode
 */
void cServo_PCA9685::sleep() {
    pca9685.sleep();
 }

/*!
 *  @brief  Wakes board from sleep
 */
void cServo_PCA9685::wakeup() {
    pca9685.wakeup();
}

/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *  @param  prescale
 *          Configures the prescale value to be used by the external clock
 */
void cServo_PCA9685::setExtClk(uint8_t prescale) {
    pca9685.setExtClk(prescale);
}

void cServo_PCA9685::setPWMFreq(float freq){
    pca9685.setPWMFreq(freq);
}

uint8_t cServo_PCA9685::getPWM(uint8_t num){
    return pca9685.getPWM(num);
}

void cServo_PCA9685::setPWM(uint8_t num, uint16_t on, uint16_t off){
    pca9685.setPWM(num, on, off);
}

void cServo_PCA9685::setPin(uint8_t num, uint16_t val, bool invert){
    pca9685.setPin(num,  val, invert );
}

uint8_t cServo_PCA9685::readPrescale(void){
    return pca9685.readPrescale();
}

void cServo_PCA9685::writeMicroseconds(uint8_t num, uint16_t Microseconds){
    pca9685.writeMicroseconds(num, Microseconds);
}

void cServo_PCA9685::setMajorServMin(int min){
    MajorServMin = min; 
}
void cServo_PCA9685::setMajorServMax(int max){
    MajorServMax = max; 
} 
void cServo_PCA9685::setSupportServMin(int min){
    SupportServMin = min;
} 

void cServo_PCA9685::setSupportServMax(int max){
    SupportServMax = max;
} 

void cServo_PCA9685::setMinV(int min){
    MinV = min;
} 

void cServo_PCA9685::setMaxV(int max){
    MaxV = max;
} 

void cServo_PCA9685::setCount(int c){
  count = c; 
} 

void cServo_PCA9685::setWaiting(int w){
    waiting = w ;
} 

void cServo_PCA9685::setCurPosMajor(int pos)  {
    CurPosMajor = pos ;
    pwm = map(CurPosMajor, 0, 180, SERVOMIN, SERVOMAX);
    pca9685.setPWM(SER0, 0, pwm);
} 

void cServo_PCA9685::setCurPosSupport(int pos)  {
    CurPosSupport = pos ;
    pwm = map(CurPosSupport, 0, 180, SERVOMIN, SERVOMAX);
    pca9685.setPWM(SER1, 0, pwm);
} 

int cServo_PCA9685::angleToPulse(int ang){
   int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
//   Serial.print("Angle: ");Serial.print(ang);
//   Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}

void cServo_PCA9685::goAhead(){
  for (int pos = MinV; pos <= MaxV; pos++ ) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      pwm = angleToPulse(pos);
      if ((pos >= MajorServMin) && (pos <= MajorServMax)) {         
           pca9685.setPWM(SER0, 0, pwm);
           CurPosMajor = pos;
       }
      // in steps of 1 degree
      if ((pos >= SupportServMin) && (pos <= SupportServMax)) {
          pca9685.setPWM(SER1, 0, pwm);
          CurPosSupport = pos;
      }
      vTaskDelay ( waiting / portTICK_PERIOD_MS);
   }
}

void cServo_PCA9685::goBack(){
  for (int pos = MaxV; pos >= MinV; pos-- ) { // goes from 180 degrees to 0 degrees
      pwm = angleToPulse(pos);
      if ((pos >= MajorServMin) && (pos <= MajorServMax)) {                  
          pca9685.setPWM(SER0, 0, pwm);
          CurPosMajor = pos;
      }
      if ((pos >= SupportServMin) && (pos <= SupportServMax )) { 
          pca9685.setPWM(SER1, 0, pwm);
          CurPosSupport = pos;
      }
      vTaskDelay ( waiting / portTICK_PERIOD_MS);  
  }
}

void cServo_PCA9685::MoveMajorServoToStart(){
   MoveMajorServoToNewPos(MajorServMin);
}

void cServo_PCA9685::MoveMajorServoToNewPos(int newPos){
  int step = 1;
  int startPos = CurPosMajor;
  if (CurPosMajor == newPos) return;
  if (CurPosMajor > newPos) {
    for (int pos = startPos; pos >= newPos; pos -= step) { // goes from 180 degrees to 0 degrees
        pwm = angleToPulse(pos);
        pca9685.setPWM(SER0, 0, pwm);
        CurPosMajor = pos;
        vTaskDelay ( waiting / portTICK_PERIOD_MS);
    };
  } else {
      for (int pos = startPos; pos <= newPos; pos += step) { // goes from 180 degrees to 0 degrees
        pwm = angleToPulse(pos);
        pca9685.setPWM(SER0, 0, pwm);
        CurPosMajor = pos;
        vTaskDelay ( waiting / portTICK_PERIOD_MS);
      };
  };
}


void cServo_PCA9685::MoveSupportServoToStart(){
   MoveMajorServoToNewPos(MajorServMin);
}

void cServo_PCA9685::MoveSupportServoToNewPos(int newPos){
   int startPos = CurPosSupport;
   if (CurPosSupport == newPos) return;
   if (CurPosSupport < newPos) {
     for (int pos = startPos; pos <= newPos; pos ++) { // goes from 180 degrees to 0 degrees
       pwm = angleToPulse(pos);
       pca9685.setPWM(SER1, 0, pwm);
       CurPosSupport = pos;
       vTaskDelay ( waiting / portTICK_PERIOD_MS);
     };
  } else  {  // CurPos1 > newPos
    for (int pos = startPos; pos >= newPos; pos --) { // goes from 180 degrees to 0 degrees
      pwm = angleToPulse(pos);
      pca9685.setPWM(SER1, 0, pwm);
      CurPosSupport = pos;
      vTaskDelay ( waiting / portTICK_PERIOD_MS);
   };
 }
}
int cServo_PCA9685::getMajorServMin(){
    return MajorServMin; 
}
int cServo_PCA9685::getMajorServMax(){
    return MajorServMax ; 
} 

int cServo_PCA9685::getSupportServMin(){
    return SupportServMin ;
} 

int cServo_PCA9685::getSupportServMax(){
    return SupportServMax ;
} 

int cServo_PCA9685::getCurPosMajor(){
    return CurPosMajor ;
} 

int cServo_PCA9685::getCurPosSupport(){
    return CurPosSupport ;
} 
