# -*- coding: utf-8 -*-
"""
Created on Sat Apr 23 06:26:38 2016

@author: akira
"""

import sys
import numpy as np
xsig= 1
ysig= -1

def revCmd(c):
    if(c["G"]==2):c["G"]=3
    if(c["G"]==3):c["G"]=2
    c["X"],c["LX"]=c["LX"],c["X"]
    c["Y"],c["LY"]=c["LY"],c["Y"]
    if(c["G"]==2 or c["G"]==3 ):
        c["I"]-= c["LX"]-c["X"]
        c["J"]-= c["LY"]-c["Y"]
    return c
def cmdToStr(c):
    cmdOrder=("G","X","Y","I","J")
    ret=""
    for i in cmdOrder:
        if i in c:
            ret += i+str(c[i])+" "
    return ret


cmd=[]
xlog=[]
ylog=[]

lastX=None
lastY=None
for line in sys.stdin:
    if(len(line)<2 or "G" != line[0]): continue
    c = line.split()
    c=list(filter(lambda c:c[0] in ("G","X","Y","I","J") , c))
    if(len(c)<2):continue    
    now=dict([ [d[0],float(d[1:])] for d in c  ])
    now['G']=int(now['G'])
    now['LX']=lastX
    now['LY']=lastY
    if(1<= now['G'] <= 3):
        cmd.append(now)
    if('X' in now):
        lastX=now['X']
    if('Y' in now):
        lastY=now['Y']



    
#find 0,0
nx=np.array( [ c['X'] for c in cmd])
ny=np.array( [ c['Y'] for c in cmd])
lx=np.array( [ c['LX'] for c in cmd])
ly=np.array( [ c['LY'] for c in cmd])
min_x=min(np.min(nx*xsig),np.min(lx*xsig))*xsig
min_y=min(np.min(ny*ysig),np.min(ly*ysig))*ysig
#move to 0,0 (panel setting position normalizing)
if True:
    nx-=min_x
    lx-=min_x
    ny-=min_y
    ly-=min_y
    for c in cmd:
        c['X']-=min_x
        c['LX']-=min_x
        c['Y']-=min_y
        c['LY']-=min_y
        
    min_x=0
    min_y=0



#set startPoint
i=np.hypot(lx,ly).argmin();
lastX=lx[i]
lastY=ly[i]

sortedCmd=[]
while 0<len(cmd):
    nx=np.array( [ c['X'] for c in cmd])
    ny=np.array( [ c['Y'] for c in cmd])
    nlen=np.hypot(nx-lastX,ny-lastY)
    nidx=nlen.argmin()
    nmin=nlen[nidx]

    lx=np.array( [ c['LX'] for c in cmd])
    ly=np.array( [ c['LY'] for c in cmd])
    llen=np.hypot(lx-lastX,ly-lastY)
    lidx=llen.argmin()
    lmin=llen[lidx]

    if(lmin<=nmin):#normal mode
        if(0.01<lmin):print("#Error? skip detected,Not One-Stroke sketch:",lmin)
        sortedCmd.append(cmd[lidx])
        del cmd[lidx]
    else: #reverse mode
        if(0.01<nmin):print("#Error? skip detected,Not One-Stroke sketch",nmin)
        sortedCmd.append(revCmd(cmd[nidx]))
        del cmd[nidx]

    lastX=sortedCmd[-1]["X"]
    lastY=sortedCmd[-1]["Y"]

#cutStart="X"+str(min_x-3*xsig)+" Y"+str(sortedCmd[0]['LY'])
#cutStart="X"+str(sortedCmd[0]['LX'])+" Y"+str(min_y-3*ysig)
cutStart="X"+str(min_x)+" Y"+str(min_y-3*ysig)


print("G90")
print("G01 F4.4")
print("M104 S0.1")
for i in range(10): print("G04 P500\nM39")
print("G92 "+cutStart)
print("G01 X"+str(sortedCmd[0]['LX'])+" Y"+str(sortedCmd[0]['LY'])) 
print("\n".join([cmdToStr(c) for c in sortedCmd]))
print("G01 "+cutStart)
print("M104 S0")
