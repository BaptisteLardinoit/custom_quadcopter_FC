
void gyro_signals(void){
  Wire.beginTransmission(0x68);       //Start I2C communication with the gyro
  Wire.write(0x1A);                   //Low pass filter activated
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);       //
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);       //Access register storing gyro measurements
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8| Wire.read();  //Read the gyro measurement around the X axis
  int16_t GyroY=Wire.read()<<8| Wire.read();
  int16_t GyroZ=Wire.read()<<8| Wire.read();
  RateRoll=(float)GyroX/65.5;   //Convert the measurement units to °/s
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
}