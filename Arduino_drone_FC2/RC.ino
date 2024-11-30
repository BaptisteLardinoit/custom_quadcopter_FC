void Throttle(){
  if(digitalRead(17)==HIGH){
    T1=micros();
  }
  if(digitalRead(17)==LOW){
    InputThrottle = micros()-T1;
    InputThrottle = map(InputThrottle, 992, 2016, 0, 180);
  }
}

void Roll(){
  if(digitalRead(16)==HIGH){
    T2=micros();
  }
  if(digitalRead(16)==LOW){
    InputRoll = micros()-T2;
  }
}
void Pitch(){
  if(digitalRead(15)==HIGH){
    T3=micros();
  }
  if(digitalRead(15)==LOW){
    InputPitch = micros()-T3;
  }
}
void Yaw(){
  if(digitalRead(14)==HIGH){
    T4=micros();
  }
  if(digitalRead(14)==LOW){
    InputYaw = micros()-T4;
  }
}
void Arm(){
  if(digitalRead(8)==HIGH){
    T5=micros();
  }
  if(digitalRead(8)==LOW){
    InputArm = micros()-T5;
  }
}
