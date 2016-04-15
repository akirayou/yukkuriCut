#include <TimerThree.h>
//extern  float heaterA=0;
//extern  float monA;

void wireCont(){
  static unsigned long out=0;
  const  long ss=2;
  int ad=analogRead(PIN_HEATER_SENS);//consume 100us
  //500mV=0A 133mV/A
  //1 ADbit= 0.03671287593A  500mV=102.4
  float A=(ad-102.4)*0.03671287593;
  A=monA*0.9+A*0.1;
  monA=A;
  
  if(A<heaterA)out+=1;
  if(heaterA<A)out-=1;
  out+=round((heaterA-A)*5);
  
  if(out<0)out=0;
  if(255*ss<out)out=255*ss;
  if(heaterA<0.001)out=0;
  analogWrite(PIN_HEATER,out/ss);
}
void wireInit(){
  pinMode(PIN_HEATER,OUTPUT);
  analogWrite(PIN_HEATER,0);
  Timer3.initialize(10*1000);//us
  Timer3.attachInterrupt(wireCont);
}

