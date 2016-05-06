import sys
#memo:
# On tuning, you have to run little bit long path
# 

heaterA=(float)(sys.argv[1]);
startFeed=(float)(sys.argv[2]);
step=(float)(sys.argv[3]);
nofStep=(int)(sys.argv[4])

print("""
G91
M104 S"""+str(heaterA)+"""
G04 P500
M39
G04 P500
M39
G04 P500
M39
G04 P500
M39
G04 P500
M39
G04 P500
M39
G04 P500
M39
""" )
for s in range(nofStep):
    f=round(step*s+startFeed,3)
    print( heaterA,f, file=sys.stderr) 
    
    print("""
G00 X3 F2.4 
G01  F"""+str(f)+"""
G00 Y-20
G00 X3 
G00 Y20
""")

print("""
M104 S0
M18
""")