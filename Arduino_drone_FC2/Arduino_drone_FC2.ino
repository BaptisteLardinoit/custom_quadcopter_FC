#include <Servo.h>
#include "Wire.h"
Servo motor1;Servo motor2;Servo motor3;Servo motor4;  
float throttle1, throttle2, throttle3, throttle4;


float T1,T2,T3,T4,T5;
float InputThrottle,InputRoll,InputPitch,InputYaw, InputArm;

float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

float loopTime = 0.004;                                                           //duration of loop to calculate the derivative

float PID_roll; float PID_pitch; float PID_yaw;
float PIDreturn[] = {0, 0, 0};

float PRateRoll = 0.6; float IRateRoll = 0.0/*3.5*/; float DRateRoll = 0.03;               //PID constants
float PRatePitch = 0.6; float IRatePitch = 0.0/*3.5*/; float DRatePitch = 0.03;
float PRateYaw = 2; float IRateYaw = 0.0/*12*/; float DRateYaw = 0.0; 

////////////////////////////////////////////////////MPU///////////////////////////////////////////
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
/////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  Serial.println("Start programme...");

  motor1.attach(12);motor2.attach(11);motor3.attach(10);motor4.attach(9);

  pinMode(14, INPUT);pinMode(15, INPUT);pinMode(16, INPUT);pinMode(17, INPUT);  //Radio input pin's

  attachInterrupt(digitalPinToInterrupt(17), Throttle, CHANGE);  //RC input
  attachInterrupt(digitalPinToInterrupt(16), Roll, CHANGE);
  attachInterrupt(digitalPinToInterrupt(15), Pitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(14), Yaw, CHANGE);
  attachInterrupt(digitalPinToInterrupt(8), Arm, CHANGE);

  delay(250);
  motor1.write(0);motor2.write(0);motor3.write(0);motor4.write(0);
  while(InputThrottle < -5 || InputThrottle > 10){ //Checking to see throttle is at min.
    Serial.print("Put the throttle down to arm               ");
    Serial.println(InputThrottle);
    digitalWrite(13,LOW);
    delay(1);
  }
  /////////////////////////////////////MPU///////////////////////////////////////////////////////

  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);   //Start gyro power mode
  Wire.write(0x6B);
  Wire.write(0x00);

  Wire.endTransmission();

  for(RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber++){
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    if(RateCalibrationNumber%50==0){
      digitalWrite(13, !digitalRead(13));
      Serial.println("Calibration don't move the drone");
    }
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
///////////////////////////////////////////////////////////////////////////////////////////////////
 
}



void loop() {
  while(InputArm > 900 && InputArm < 1100){ //Arming switch
    Serial.println("Disarmed");
    motor1.write(35);motor2.write(35);motor3.write(35);motor4.write(35);
    while(InputThrottle < -5 || InputThrottle > 10){ //Throttle has to be down again.
    Serial.println("Disarmed");
    delay(1);
  }
  delay(1);
  }

  PrevErrorRateRoll = ErrorRateRoll;    //Previous error for calculating the derivative
  PrevErrorRatePitch = ErrorRatePitch;
  PrevErrorRateYaw = ErrorRateYaw;

  gyro_signals();
  //Timer = micros();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;

  DesiredRateRoll = 0.15*(InputRoll-1500);
  DesiredRatePitch = 0.15*(InputPitch-1500);
  DesiredRateYaw = 0.15*(InputYaw-1500);

  ErrorRateRoll = DesiredRateRoll+RateRoll;
  ErrorRatePitch = DesiredRatePitch+RatePitch;
  ErrorRateYaw = DesiredRateYaw+RateYaw;

  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll, loopTime);
  PID_roll = PIDreturn[0]; 
  PrevItermRateRoll = PIDreturn[1]; 
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch, loopTime);
  PID_pitch = PIDreturn[0];
  PrevItermRatePitch = PIDreturn[1];
  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw, loopTime);
  PID_yaw = PIDreturn[0];
  PrevItermRateYaw = PIDreturn[1];
  
  if (InputThrottle > 162) InputThrottle = 162; //162 = 0.9*180

  throttle1 = InputThrottle + PID_roll + PID_pitch + PID_yaw;
  throttle2 = InputThrottle - PID_roll + PID_pitch - PID_yaw;
  throttle3 = InputThrottle - PID_roll - PID_pitch + PID_yaw;
  throttle4 = InputThrottle + PID_roll - PID_pitch - PID_yaw;
/////////////////////////////////////////////////////////////Range limiting////////////////////////////////////
  if(throttle1 > 180) throttle1 = 179;
  if(throttle2 > 180) throttle2 = 179;
  if(throttle3 > 180) throttle3 = 179;
  if(throttle4 > 180) throttle4 = 179;
  int idle = 16; int cutoff = 10;
  if(throttle1 < idle && throttle1 > cutoff) throttle1 = idle;
  if(throttle2 < idle && throttle2 > cutoff) throttle2 = idle;
  if(throttle3 < idle && throttle3 > cutoff) throttle3 = idle;
  if(throttle4 < idle && throttle4 > cutoff) throttle4 = idle;
  if(throttle1 < cutoff) throttle1 = 0;
  if(throttle2 < cutoff) throttle2 = 0;
  if(throttle3 < cutoff) throttle3 = 0;
  if(throttle4 < cutoff) throttle4 = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  motor1.write(throttle1+35);
  motor2.write(throttle2+35);
  motor3.write(throttle3+35);
  motor4.write(throttle4+35);

  Serial.println(throttle1);
}

  /*Serial.print("Roll rate [°/s]=");
  Serial.print(RateRoll);
  Serial.print("Pitch rate [°/s]=");
  Serial.print(RatePitch);
  Serial.print("Yaw rate [°/s]=");
  Serial.println(RateYaw);
  delay(50);*/