void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm, float loopTime){
  float Pterm = P*Error;
  float Dterm = D*((Error-PrevError)/loopTime);
  float Iterm = PrevIterm+I*Error*loopTime;
  if(Iterm>36) Iterm = 400;        //36=0.2*180 
  else if(Iterm<-36) Iterm = -400;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput>36) PIDOutput=36;
  else if (PIDOutput <-36) PIDOutput=-36;
  PIDreturn[0]=PIDOutput;
  PIDreturn[1]=Iterm;
}