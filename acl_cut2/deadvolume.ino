float deadVolumeCol(long dx,long dy,long odx,long ody){
  //cutting wire width little bit lager than real width.Because it includes vapor layer
  //Unit is step. for example 1000step/mm and 0.2mm wire real width is 200.
  const float wireWidth=200; 
  float nlen=hypot(dx,dy);

  float olen=hypot(odx,ody);
  float halfTheta=atan2(odx,ody)-atan2(dx,dy);//          0.5 * acos(( ((float)dx)*odx+((float)dy)*ody )   /nlen/olen);
  if(halfTheta<0)halfTheta+=2*M_PI;
  halfTheta=fmod(halfTheta,2*M_PI);
  
  if(M_PI<halfTheta)halfTheta=2*M_PI-halfTheta;
  halfTheta/=2;
  
  if(olen<1 || nlen<1)halfTheta=0;
 
  float speedUpLen=(tan(halfTheta)-halfTheta) * wireWidth/4;
  if(wireWidth*10<speedUpLen)speedUpLen= wireWidth*10;
  return speedUpLen;
}

