#include "tools.h"

const float r1 = 30000.0f; // R1 in ohm, 50K
const float r2 =  7500.0f; // R2 in ohm, 10k potentiometer
const long interval = 1000;  // interval at which to blink (milliseconds)
int ledState = LOW;  // ledState used to set the LED

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
  Serial.printf(" \nshake: Major Servo Config %d %d",MajorServMin, MajorServMax);
  Serial.printf(" \nshake: Support Servo Config %d %d",SupportServMin, SupportServMax);

  StateMsg = "State: shakeTask started: count = "+String(count);

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
 
  while (count > 0)  {
    count--;
    valueString4 = String(count);
    if ( fReadBatteryChannel_3() > 6.0f) {
      StateMsg = "State: go ahead";
      pca9685.setWaiting(waiting);
      
      pca9685.goAhead();
      pca9685.setWaiting(waiting);

      StateMsg = "State: go back";
 
      pca9685.goBack();
      
      StateMsg = "State: loop complete";
      
    } else {
        Serial.println("shake:LiPo Acu is getting below 6V, servo is off");
        vTaskDelay ( waiting / portTICK_PERIOD_MS);
        count = 0;
    }
  }    
  
  canBeStopped = false;

  StateMsg = "State: shakeTask terminated: count = "+String(count);
 
  if ( Task_CurrentMon != NULL) vTaskDelete(Task_CurrentMon);

  StateMsg = "State: shakeTask task CurrentMon deleted";
  
  vTaskDelete(NULL);

}

void shakeTaskPerTimer( void *param ) {
 
  unsigned long startTime = millis();
 // Previous time
  unsigned long currentTime = startTime; 

  StateMsg = "State: shakeTask started: duration (millisec/min) = "+String(shakeDurationMillisec)+"/"+String(shakeDurationMillisec/60000);
 
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

  while ( (currentTime - startTime) < shakeDurationMillisec )  {
    currentTime = millis();
    pca9685.setWaiting(waiting);
    StateMsg = "State: shakeTask duration (soll millisec)="+String(shakeDurationMillisec)+"(is millisec/sec)= "+String(currentTime - startTime)+"/"+String((currentTime - startTime)/1000);
    Serial.println(StateMsg);
    valueString6 = String(int((shakeDurationMillisec - (currentTime - startTime))/60000));   
 
    if ( fReadBatteryChannel_3() > 6.0f) {
      StateMsg = "State: go ahead";
      pca9685.goAhead();
      StateMsg = "State: go back";
      pca9685.setWaiting(waiting);
      pca9685.goBack();
      StateMsg = "State: loop complete";
    } else {
      StateMsg = "State: LiPo Acu is getting below 6V, servo is off";
      Serial.println(StateMsg);      
      vTaskDelay ( (waiting*100) / portTICK_PERIOD_MS);
    }     
  }
  
  canBeStopped = false;
  StateMsg = "State: shakeTask terminated after (millisec/sec) = "+String(currentTime - startTime)+"/"+String((currentTime - startTime)/1000);
  Serial.println(StateMsg);

  if ( Task_CurrentMon != NULL ) vTaskDelete(Task_CurrentMon);
    
  vTaskDelete(NULL);
}

void current_monitor_task(void *param)
{
  float ACSValue1 = 0.0, Samples1 = 0.0, AvgACS1 = 0.0,  BaseVol = 2.08f ;
  float ACSValue2 = 0.0, Samples2 = 0.0, AvgACS2 = 0.0 ;

  //Change BaseVol as per your reading in the first step.                            

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
      StateMsg = "current_monitor_task: Max current(3) more than 2.5 A, servo will be off";
      canBeStopped = false;
      xSemaphoreGive(  let_me_process ); 
      if ( Task_HW != NULL ) vTaskDelete(Task_HW);
      vTaskDelete(NULL);
    }

    if ( Amps1_Max > 2.5 ) {
      StateMsg = "current_monitor: Max current(1) more than 2.5 A, servo will be off";
      canBeStopped = false;
      xSemaphoreGive(  let_me_process ); 
      if ( Task_HW != NULL ) vTaskDelete(Task_HW);
      vTaskDelete(NULL);      
    }
    snprintf(cur_1_string, 12, "%2.2f/(%3d)", Amps1_Max,CurPos0);
    snprintf(cur_2_string, 12, "%2.2f/(%3d)", Amps2_Max,CurPos1);
    valueCur_1 = String( cur_1_string ) ;
    valueCur_2 = String( cur_2_string ) ;
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
