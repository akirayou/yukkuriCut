#include<iostream>
#include<cmath>
using namespace std;
#include "deadvolume.ino"
int main(){
    double r=1000;
    double dtheta=0.5*M_PI;
    
    for(float h=0;h<M_PI*2;h+=0.01){
        //r=(rand()%1000)+300;
        r=30000;
        long dy=r*sin(h);
        long dx=r*cos(h);
        //r=(rand()%1000)+300;
        r=700;
        long ody=r*sin(h+dtheta);
        long odx=r*cos(h+dtheta);
        cout <<deadVolumeCol(dx,dy,odx,ody)<<endl;
        
    }
    
    
}