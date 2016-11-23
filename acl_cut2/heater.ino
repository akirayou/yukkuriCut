#include <TimerThree.h>
//extern  float heaterA=0;
//extern  float monA;
//extern  int  monOut;
void wireCont(){
  static unsigned long out=0;
  const  long ss=4;
  int ad=analogRead(PIN_HEATER_SENS);//consume 100us
  //500mV=0A 133mV/A
  //1 ADbit= 0.03671287593A  500mV=102.4
  float rawA=(ad-102.4)*0.03671287593;
  monA=monA*0.9+rawA*0.1;
  // monA=monOut/255*V/R
  // R/V = monOut/monA / 255
  // heaterA must be adjusted as related to 1/V and R(=nichrome wire length)
  
  if(monA<heaterA)out+=1;
  if(heaterA<monA )out -=1;
  if(0.2+heaterA<rawA)out-=ss;
  if(heaterA -0.4 >rawA)out+=ss;
  
  out+=(int)((heaterA-monA)*ss*3);
  
  if(out<0)out=0;
  if(255*ss<out)out=255*ss;
  if(heaterA<0.001)out=0;
  monOut =monOut*0.9 + 0.1*out;
  analogWrite(PIN_HEATER,out/ss);
}
void wireInit(){
#if FASTER_TIMER == 8
  TCCR0B = TCCR0B & 0b11111000 | 2;
#endif
#if FASTER_TIMER == 64
  TCCR0B = TCCR0B & 0b11111000 | 1;
#endif

  pinMode(PIN_HEATER,OUTPUT);
  analogWrite(PIN_HEATER,0);
  Timer3.initialize(10*1000);//us
  Timer3.attachInterrupt(wireCont);
}

