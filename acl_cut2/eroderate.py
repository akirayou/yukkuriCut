import numpy as np
import math
import itertools
import sys
speedup=1
skip=10
wireRadius=150//speedup
wireWidth=wireRadius*2+1
work=np.zeros((wireWidth,wireWidth),dtype=np.int8)

pos=np.array( list(itertools.product(range(-wireRadius,wireRadius+1),repeat=2))).reshape(wireWidth,wireWidth,2)


f=sys.stdin #open('o')

log=np.loadtxt(f,dtype=np.float32)[::skip,:]
log[:,0]/=speedup
log[:,1]/=speedup

def fill(dx,dy,fillVal):
    global work
    cond= np.hypot(pos[:,:,1]-dx,pos[:,:,0]-dy)<wireRadius
    work[cond]=fillVal
def nofErode(log):
    global work
    x,y=log[-1,:2]
    work.fill(0)
    fill(0,0,1)
    tlog=log[:-1,:2]
    tlog= tlog[(np.abs(tlog[:,0]-x)<=wireRadius )* (np.abs(tlog[:,1]-y)<=wireRadius) ,:]
    
    if True:    
        for tx,ty in tlog:
            if(math.hypot(tx-x,ty-y)<=wireRadius):        
                fill(tx-x,ty-y,0)
    else:
        for twx,twy in itertools.product(range(wireWidth),repeat=2):
            wx=twx+x-wireRadius
            wy=twy+y-wireRadius
            
            if(  np.any(np.hypot(tlog[:,0]-wx,tlog[:,1]-wy)  < wireRadius )):
                work[twx,twy]=0
            
    return work.sum()

oldTime=0
for i in range(2,len(log)):
    dtime=log[i,2]-log[i-1,2];
    el=nofErode(log[:i])
    print(log[i,0]*speedup,log[i,1]*speedup,el/dtime,el)
    
    
c=np.abs(log[:,1])<=wireRadius

    
