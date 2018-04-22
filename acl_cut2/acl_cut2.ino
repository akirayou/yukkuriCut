//------------------------------------------------------------------------------
// 2 Axis CNC Demo  
// dan@marginallycelver.com 2013-08-30
//------------------------------------------------------------------------------
// Original Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.
// 
// modified by @akira_you Nico-TECH 2016-4
// ---modified for YukkuriCutter
// 

//#define FASTER_TIMER 8
//FAN
#define FASTER_TIMER 64
#include <Servo.h>
Servo fanServo;
int fanLevel=0;

//About Hardware
//heater Driver(PIN_HEATER) is hiActive FET (PWM) with 5V
//heater sensor(PIN_HEATER_SENS) is pololu acs715 with 1uF filter capasitor
//Motor driver is anything is OK. I use pololu  drv8834 
//                                and 200step/rot 5V Stepping motor
//This software only for 32u4 CPU arduino 
//Because this is using Timer3. Please rewrite this for other Arduino
//
//4 Push button is for manual move and cut off heater power
//Please See BUT_* define in config.h
//


#include "config.h"
//heater
void wireInit();
float heaterA=0;
float monA;
float monOut;
//heater --end
float deadVolumeCol(long dx,long dy,long odx,long ody);
//pause for  30min is max total delay in each command
unsigned long Atime;
unsigned long AtimeTar;
void ApauseInit(){
  Atime=micros();
}
void Apause(long ms,long minimum=MIN_STEP_DELAY){
  
  Atime+=ms*FASTER_TIMER;
  minimum*=FASTER_TIMER;
  minimum+= micros();
  if((long)(Atime-minimum)<0){
    while( (long) ( micros()-minimum)<0); 
  }else{
    while( (long) ( micros()-Atime)<0); //wait for time comes
  }
}


void buttonInit(){
  pinMode(BUT_X1, INPUT_PULLUP); 
  pinMode(BUT_Y1, INPUT_PULLUP); 
  pinMode(BUT_X2, INPUT_PULLUP); 
  pinMode(BUT_Y2, INPUT_PULLUP); 
}
void buttonCont(){
    bool run=0;
   if(!digitalRead(BUT_X1)){m1step(1);run=1;}
   if(!digitalRead(BUT_X2)){m1step(-1);run=1;}
   if(!digitalRead(BUT_Y1)){m2step(1);run=1;}
   if(!digitalRead(BUT_Y2)){m2step(-1);run=1;}
   if(run){
    ApauseInit();
    Apause(MIN_STEP_DELAY);
    heaterA=0;
   }
}


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

char buffer[MAX_BUF];  // where we store the message until we get a newline
int sofar;  // how much is in the buffer

float px, py;  // location

// speeds
float fr=0;  // human version
long step_delay;  // machine version

// settings
char mode_abs=1;  // absolute mode?


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------






/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  if(nfr<0)return;//nochange
  if(fr==nfr) return;  // same as last time?  quit now.

  if(nfr>MAX_FEEDRATE || nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE/STEPS_PER_MM );
    Serial.print(F("mm/s and less than "));
    Serial.print(MAX_FEEDRATE/STEPS_PER_MM );
    Serial.println(F("mm/s."));
    return;
  }
  step_delay = 1000000/nfr;
  fr=nfr;
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx,float npy) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
}


/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void line(float newx,float newy,bool doSpeedUp=true) {
  
  long dx=newx-px;
  long dy=newy-py;
  if(dx==0 && dy==0)return;
  float nlen=hypot(dx,dy);
  static long odx,ody;
  //Dead volume correction   (now testing. Is it good or bad?)
  static float speedUpLen=0;
  speedUpLen*=max(0,cos(atan2(odx,ody)-atan2(dx,dy))); //for safety
  //speedUpLen=0;//for more safety
  if(doSpeedUp)speedUpLen+=deadVolumeCol(dx,dy,odx,ody);
  //speedUpLen=0;
  const float speedupRate=1.5;
  speedUpLen/=(speedupRate-1)/speedupRate;
  odx=dx;
  ody=dy;


  int dirx=dx>0?1:-1;
  int diry=dy>0?-1:1;  // because the motors are mounted in opposite directions
  dx=abs(dx);
  dy=abs(dy);
  long i;
  long over=0;


  if(dx>dy) {
    float step_delay2=step_delay * nlen/dx;  //real path length VS dx correction
    long speedUpLenI=round(speedUpLen*dx/nlen);

    for(i=0;i<dx;++i) {
      if( (!digitalRead(BUT_X1)) && (!digitalRead(BUT_X1)))return;//emergency abort
      m1step(dirx);
      over+=dy;
      if(over>=dx) {
        over-=dx;
        m2step(diry);
      }
      Apause(i<speedUpLenI? max(MIN_STEP_DELAY,step_delay2/speedupRate): step_delay2);
    }
  } else {
    float step_delay2=step_delay * nlen/dy;
    long speedUpLenI=round(speedUpLen*dy/nlen);
    for(i=0;i<dy;++i) {
      if( (!digitalRead(BUT_X1)) && (!digitalRead(BUT_X1)))return;//emergency abort
      m2step(diry);
      over+=dx;
      if(over>=dy) {
        over-=dy;
        m1step(dirx);
      }
      Apause(i<speedUpLenI? max(MIN_STEP_DELAY,step_delay2/speedupRate): step_delay2);
    }
  }
  speedUpLen-=nlen;
  if(speedUpLen<0)speedUpLen=0;
  px+=dirx*dx; //bubfix  don't use newXY
  py-=diry*dy;
}


// returns angle of dy/dx as a value from 0...2PI
float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}


// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc


void arc(float cx,float cy,float x,float y,float dir) {
  // get radius
  float dx = px - cx;
  float dy = py - cy;
  float radius=sqrt(dx*dx+dy*dy);
  //exception when radius=0 
  if(radius<1 || radius*0.001> abs(px-x)+abs(py-y) ){
    line(x,y);
    return;
  }
  // find angle of arc (sweep)
  float angle1=atan3(dy,dx);
  float angle2=atan3(y-cy,x-cx);
  float theta=angle2-angle1;
  
  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;
  theta=angle2-angle1;
  
  float nx, ny, angle3, scale;
  float angleStep=PI-acos(1/radius-1); //1step error is enough fine
  if(angleStep<0.01)angleStep=0.01;
  bool first=true;
  if(theta<0){
    for(angle3=angle1;angle3>=theta+angle1+angleStep;angle3-=angleStep){
      nx = cx + cos(angle3) * radius;
      ny = cy + sin(angle3) * radius;
      line(nx,ny);
      first=false;
    }
  }else{
    for(angle3=angle1;angle3<=theta+angle1-angleStep;angle3+=angleStep){
      nx = cx + cos(angle3) * radius;
      ny = cy + sin(angle3) * radius;
      line(nx,ny);
      first=false;
    }
  }
  line(x,y);
}


/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parsenumber(char code,float val) {
  char *ptr=buffer;
  while( *ptr && ptr<buffer+sofar) {
    if(*ptr==code) {
      return atof(ptr+1);
    }
    ptr=strchr(ptr,' ');
    if(ptr==0)break;
    ptr++;
  }
  return val;
} 


/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(const char *code,float val) {
  Serial.print(code);
  Serial.println(val);
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",px/STEPS_PER_MM);
  output("Y",py/STEPS_PER_MM);
  output("F",fr/STEPS_PER_MM);
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * display helpful information
 */
void help() {
  Serial.print(F("GcodeCNCDemo2AxisV1 "));
  Serial.println(VERSION);
  Serial.print("Heatwire using timer No.(in Arduino.h)[must not to be 3-5 on 32u4] ");
  Serial.println(digitalPinToTimer(PIN_HEATER));
  Serial.println(F("Commands:"));
  Serial.println(F("G00 [X()] [Y(mm)] [F(mm/sec)]; - line"));
  Serial.println(F("G01 [X(mm)] [Y(mm)] [F(feedrate)]; - line"));
  Serial.println(F("G02 [X(mm)] [Y(mm)] [I(mm)] [J(mm)] [F(mm/sec)]; - clockwise arc"));
  Serial.println(F("G03 [X(mm)] [Y(mm)] [I(mm)] [J(mm)] [F(mm/sec)]; - counter-clockwise arc"));
  Serial.println(F("G04 P[mili seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(mm)] [Y(mm)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("M104 S(heater A); - set heaater current (A) over 99 means always ON"));
  Serial.println(F("All commands must end with a newline."));
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  ApauseInit();
  float mpx,mpy;
  int cmd = parsenumber('G',-1);
  switch(cmd) {
  case  0:
  case  1: { // line
    feedrate(STEPS_PER_MM * parsenumber('F',-1));
    mpx=px/STEPS_PER_MM;
    mpy=py/STEPS_PER_MM;
    line( STEPS_PER_MM * parsenumber('X',(mode_abs?mpx:0)) + (mode_abs?0:px),
          STEPS_PER_MM * parsenumber('Y',(mode_abs?mpy:0)) + (mode_abs?0:py) );
    break;
    }
  case 2:
  case 3: {  // arc
      feedrate(STEPS_PER_MM *parsenumber('F',-1));
      mpx=px/STEPS_PER_MM;
      mpy=py/STEPS_PER_MM;
      arc(
          STEPS_PER_MM * parsenumber('I',0) +px,
          STEPS_PER_MM * parsenumber('J',0) +py,
          STEPS_PER_MM * parsenumber('X',(mode_abs?mpx:0)) + (mode_abs?0:px),
          STEPS_PER_MM * parsenumber('Y',(mode_abs?mpy:0)) + (mode_abs?0:py),
          (cmd==2) ? -1 : 1);
      break;
    }
  case  4:  Apause(parsenumber('P',0)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( STEPS_PER_MM *parsenumber('X',0),
              STEPS_PER_MM *parsenumber('Y',0) );
    break;
  default:  break;
  }

  cmd = parsenumber('M',-1);
  switch(cmd) {
  case 18:  // disable motors
    disable();
    break;
  case 39:
    Serial.println(F("HATSUNE-MIKU is not implimented Yet."));
    Serial.print(F("  heaterA "));
    Serial.println(monA);
    Serial.print(F("  heater out "));
    Serial.println(monOut);
    Serial.print(F(" Est P/L= "));
    Serial.println(heaterA* monA / monOut * 255*4  );
    Serial.print(F(" fanLevel "));
    Serial.println(fanLevel  );
    break;
  case 100:  help();  break;
  case 114:  where();  break;
  case 104:
    heaterA=parsenumber('S',0);
    break;
  case 105:
    {
      float tmp=parsenumber('S',0);
      if(tmp<0)tmp=0;
      if(1000<tmp)tmp=1000;
      fanLevel=tmp;
      fanServo.writeMicroseconds(1000+fanLevel);    

    }
    break;
  default:  break;
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  fanServo.attach(FAN_SERVO_PIN);
  fanServo.writeMicroseconds(1000);
  wireInit();
  buttonInit();
  // put your setup code here, to run once:



  Serial.begin(BAUD);  // open coms
  
  setup_controller();  
  position(0,0);  // set staring position
  feedrate((MAX_FEEDRATE + MIN_FEEDRATE)/2);  // set default speed

  help();  // say hello
  ready();
}


/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
  // listen for serial commands
  buttonCont();
  
  while(Serial.available() > 0) {  // if something is available
    char c=Serial.read();  // get it
    Serial.print(c);  // repeat it back so I know you got the message
    if(sofar<MAX_BUF-1) buffer[sofar++]=c;  // store it
    if((c=='\n') ) {
      // entire message received
      buffer[sofar]=0;  // end the buffer so string functions work right
      Serial.print(F("\r\n"));  // echo a return character for humans
      processCommand();  // do something with the command
      ready();
    }
  }
}


/**
* This file is part of GcodeCNCDemo.
*
* GcodeCNCDemo is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GcodeCNCDemo is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Foobar. If not, see <http://www.gnu.org/licenses/>.
*/
