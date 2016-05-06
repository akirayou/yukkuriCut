import serial
import sys

import glob
dev=glob.glob('/dev/cu.usb*')[0]
print("Device:",dev)
#1stline: echo back of command
#2~nline: message
#last line: ">" only
class Cmd:
    def __init__(self,dev):
        self.ser = serial.Serial(dev, timeout=0.01)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.nowCmdNo=0
        self.results=[]
        self.lines=[]
        self.readBuf=""
        self.lastCmd=""
        
    def initResults(self):
        self.nowCmdNo=0
        self.results=[]
        self.lines=[]
        self.readBuf=""
    def retCmd(self):
        return [ r[0] for r in self.results]
    def getResults(self):
        self.readBuf+=self.ser.read(10000).decode().replace('>','>\n')
        
        newLines=self.readBuf.split('\n')
        self.readBuf=newLines[-1]
        newLines=newLines[:-1]
        self.lines.extend( [l.rstrip() for l in newLines])    
        if(">" in self.lines):
            l=self.lines.index(">")
            if(l>0 and len(self.lines[0])>2 and (self.lines[0][0]=="G" or self.lines[0][0]=="M")):
                self.results.append(self.lines[:l])
            self.lines=self.lines[l+1:]
            
    def command(self,cmd):
        if(len(cmd)<2): return None
        if(cmd[0]!='M' and cmd[0]!='G'): return None
        print("send:",cmd)
        cmd+="\n"
        self.ser.write(cmd.encode("utf-8"))
        self.ser.flush()    
        cmd=cmd.rstrip("\n")
        self.nowCmdNo+=1
        lastCmd=cmd

cmd=Cmd(dev)
cmd.command("M114\n\n\n\n\n\n\n\n\n\n\n\n\n")
cmd.command("M39")
while not "M39" in  cmd.retCmd():
    cmd.getResults()
print("inited")
cmd.initResults()

printedNo=0
for line in sys.stdin:
    resLen=len(cmd.results)
    while(cmd.nowCmdNo-resLen>4):
        cmd.getResults()
        resLen=len(cmd.results)
        if(printedNo!=resLen):
            res='\n'.join([ ''.join(lines) for lines in cmd.results[printedNo:]])
            print(res)
            printedNo=resLen
    
    cmd.command(line)
    
while(cmd.nowCmdNo!=resLen):
    cmd.getResults()
    resLen=len(cmd.results)
    if(printedNo!=resLen):
        res='\n'.join([ ''.join(lines) for lines in cmd.results[printedNo:]])
        print(res)
        printedNo=resLen

print("total # of command",cmd.nowCmdNo)