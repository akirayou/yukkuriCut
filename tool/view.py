# -*- coding: utf-8 -*-
"""
Created on Fri Dec  2 14:19:30 2016

@author: akira
"""

import matplotlib.pyplot as plt
import matplotlib.patches as pat
import math

fig=plt.figure("")
plt.clf()
ax = plt.axes(xlim = (-30,30), ylim = (-30,30))
#ax = matplotlib.pylab.gca()
ax.set_aspect(1)

minX=9E+10
minY=9E+10
maxX=-9E+10
maxY=-9E+10

def mm(x,y):
    global minX,minY,maxX,maxY
    minX=min(x,minX)
    maxX=max(x,maxX)
    minY=min(y,minY)
    maxY=max(y,maxY)
    

def g03(fx,fy,tx,ty,i,j):
    mm(fx,fy)
    mm(tx,ty)
    cx=fx+i
    cy=fy+j
    th1=math.atan2(fy-cy,fx-cx)*180/math.pi
    th2=math.atan2(ty-cy,tx-cx)*180/math.pi
    l=math.hypot(i,j)*2
    
    ax.add_patch(pat.Arc((cx,cy),l,l,0, th1,th2))
    
def g02(fx,fy,tx,ty,i,j):
    mm(fx,fy)
    mm(tx,ty)

    g03(tx,ty,fx,fy,i-tx+fx,j-ty+fy)
  
def g01(fx,fy,x,y):
    mm(fx,fy)
    mm(x,y)

    ax.add_patch(pat.Polygon([[fx,fy],[x,y]],closed=False))


ox=0
oy=0
nowX=0
nowY=0
nowI=0
nowJ=0

import sys
for l in sys.stdin:
    if(len(l)<3):continue
    c=l.split()

    if(not c[0][0] is  "G"):continue
    cmd=int(c[0][1:])
    x=nowX
    y=nowY
    i=nowI
    j=nowJ
    
    for arg in c[1:]:
        if(arg[0] is "X"):x=float(arg[1:])
        if(arg[0] is "Y"):y=float(arg[1:])
        if(arg[0] is "I"):i=float(arg[1:])
        if(arg[0] is "J"):j=float(arg[1:])
    if(cmd==92):
        ox=nowX=x
        oy=nowY=y
    if(cmd==1 or cmd==0):
       g01(nowX-ox,nowY-oy,x-ox,y-oy)
    if(cmd==2):        
       g02(nowX-ox,nowY-oy,x-ox,y-oy,i,j)
    if(cmd==3):        
       g03(nowX-ox,nowY-oy,x-ox,y-oy,i,j)
       
        
        
    nowX=x
    nowY=y
    nowI=i
    nowJ=j
plt.xlim([minX-5,maxX+5])
plt.ylim([minY-5,maxY+5])
ax.set_aspect(1)
ax.invert_xaxis()
plt.pause(600)
