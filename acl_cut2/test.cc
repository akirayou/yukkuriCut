#include <iostream>
#include <stdlib.h>
#include <string.h>
using namespace std;
#include <cmath>
#define IN_STUB  
#define max(x,y) fmax(x,y)
#define PI 3.1415926535
void wireInit(){}
#define F(x) (x)
class SerialT{
 public:
  void begin(int a){}
  template <class T>
  void print(T in){cerr << in;}
  template <class T>
  void println(T in){cerr << in<<endl;}
  int available(){return !cin.eof();}
  int read(){return cin.get(); }
  
};
SerialT Serial;
int digitalRead(int pin){return 1;}
void digitalWrite(int pin,int val){}
int analogRead(int pin){return 0;}
int analogWrite(int pin,int val){return 0;}
#define INPUT_PULLUP 0
#define INPUT 0
#define OUTPUT 0
#define HIGH 1
#define LOW 0
int pinMode(int a,int b){}




int x=0;
int y=0;
unsigned long mtime=0;
unsigned long micros(){
mtime+=10;
return mtime;
}
void m1step(int a){
x+=a;
 cout << x<< "\t"<<y<<"\t"<<mtime<<endl;
}
void m2step(int a){
y+=a;
 cout << x<< "\t"<<y<<"\t"<<mtime<<endl;
}
void disable(){}
void setup_controller(){}

#include "deadvolume.ino"
#include "acl_cut2.ino"

int main(){
setup();
while(!cin.eof()){
loop();
}
}

