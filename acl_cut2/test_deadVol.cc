#include<iostream>
#include<cmath>
using namespace std;
#include "deadvolume.ino"
int main(){
    double r=1000;
    
    for(double dtheta=-1*M_PI ;dtheta<=1.0*M_PI;dtheta+=0.1*M_PI){
      cerr<< dtheta<<endl;
      for(float h=0;h<M_PI*2;h+=0.01){
        //r=(rand()%1000)+300;
        r=30000;
        long dy=r*sin(h);
        long dx=r*cos(h);
        //r=(rand()%1000)+300;
        r=700;
        long ody=r*sin(h+dtheta);
        long odx=r*cos(h+dtheta);
        cout <<dtheta << " " << deadVolumeCol(dx,dy,odx,ody)<<endl;
        
      }
    }

    
}
