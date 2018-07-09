import re
import socket
import threading
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import time
import pickle
from tkinter import *
import serial
import math
import binascii
import struct

#from pncPlot import update_plot_data

# Connection Parameters
feedbackhost = '0.0.0.0'
feedbackport = 514
controlhost = '129.1.15.5'
controlhost = '127.0.0.1'
controlport = 5007
bokehHost = '127.0.0.1'
bokehPort = 6666

# Control parameters
filename = r'C:\Users\Roby\VirtualBox VMs\File Transfers\Position Sampling\Files to Send\xyzabposn_tcp_25_4.ngc'
blocklen = 25

#connectionSevered=0

# Machine Parameters
mach_offsets = np.array([2e-5, 32e-5, 22e-5, 1e-5, 359.976]);
prog_offsets = np.array([0, 0, 0, -90, 0]);

# Data Collection
commandData = np.array([0,0,0,0,0])
feedbackData = np.array([0,0,0,0,0])
encoderData = np.array([1000000])

queueData = np.array([0])
threadData = np.array([400])
timeData = np.array([0])
sampledTimeData = np.array([0])

feedbackCounter = 0
dataDict = dict()

runningTime = 0
axis_posns = []

#FIXME
num_samples = 40

#Threads
bokehIntf = 0
serialIntf = 0
control = 0
feedback = 0

def process_data_points(data_string):
    global runningTime, feedbackCounter, timeData, queueData, commandData, feedbackData, sampledTimeData, threadData

    if '0: tc' in data_string:
        m = re.search('tcqLen(.+?)T(.+?)dC(.+?):(.+)', data_string)
        if m:
            feedbackSampleCount = data_string.count('&')
            tcqLength = float(m.group(1))
            threadPeriod = float(m.group(2))
            servoDt = float(m.group(3))
            if (servoDt > feedbackSampleCount * 20):
                return
            data_points = m.group(4).strip()
            data_points = re.sub('S[0-9]*:', '', data_points)
            
            #Each time sample delimited with &
            samples = data_points.split('&')
            coords = [sample.split('|') for sample in samples]
            coords = np.asarray([[float(coord) for coord in coords[index][:-1]] for index in range(len(coords[:-1]))])
            
            commanded = coords[:,0:5]
            feedback = coords[:,5:]
            
            times = np.linspace(runningTime+servoDt/feedbackSampleCount,runningTime+servoDt,feedbackSampleCount)

            #Store parsed data
            sampledTimeData = np.append(sampledTimeData, float(m.group(3)))
            timeData = np.append(timeData, times)
            queueData = np.vstack((queueData, tcqLength))
            threadData = np.vstack((threadData, servoDt))
            
            if commandData is None:
                commandData = commanded
            else:
                commandData = np.vstack((commandData, commanded))
            if feedbackData is None:
                feedbackData = feedback
            else:
                feedbackData = np.vstack((feedbackData, feedback))

            #Update global time counter
            runningTime += servoDt
            feedbackCounter += 1
    #onUpdatePlots()


#def onUpdatePlots(fnum, plotData, dim):
#numUpdatePlots = 0
class bokehInterface(threading.Thread):
    #def __init__(self):
    #    super(bokehInterface, self).__init__()
    def __init__(self, socket):
        super(bokehInterface, self).__init__()
        #self.tcqPlotData = []
        #self.needUpdate = 0
        self.socket = socket
        #self.data = ""
        print('bokehInterface started')

    def init():
        pass
    
    def write(self,data):
        #self.conn.send(msg)

        #self.socket.sendto(pickle.dumps(np.array([1,2,3,4,5])),(bokehHost,bokehPort))
        #print('sending data to bokeh')
        self.socket.sendto(pickle.dumps(data),(bokehHost,bokehPort))
        #self.socket.sendto((data+'\r').encode('utf-8'),(bokehHost,bokehPort))
    
    #def onUpdatePlots():
    #    global runningTime, feedbackCounter, timeData, queueData, commandData, feedbackData, sampledTimeData

    #global numUpdatePlots
    #if len(plotData) >= 2:
    # print('writing to Bokeh server')

    #    print(queueData.flatten())
    #    return sampledTimeData, queueData.flatten()
    
##
##    if numUpdatePlots == 10:
##        numUpdatePlots = 0
##        update_plot_data((sampledTimeData, queueData.flatten()))
##        return sampledTimeData, queueData.flatten()
##    else:
##        numUpdatePlots += 1
        

class fb_server(threading.Thread):
    def __init__(self, conn):
        super(fb_server, self).__init__()
        #self.tcqPlotData = []
        self.needUpdate = 0
        self.conn = conn
        self.data = ""
        print('Feedback thread started')

    def run(self):
        while True:
            (bytes_received, rec_address) = self.conn.recvfrom(65536)
            string_received = bytes_received.decode("utf-8")
            self.data = self.data + string_received
            if self.data.endswith(u"&"):
                #print('Received complete string', string_received)
                process_data_points(self.data)
                self.data = ""
                self.needUpdate = 1
                #print('feedback needs update')
                dataDict['TCQ'] = queueData
                dataDict['Thread'] = threadData
                dataDict['Encoder'] = encoderData
                #bokehIntf.write([queueData threadData])
                bokehIntf.write(dataDict)

    def initPlots(self):
        print('initializing plots')

    def send_msg(self,msg):
        self.conn.send(msg)

    def close(self):
        self.conn.close()

class control_client(threading.Thread):
    def __init__(self, conn):
        super(control_client, self).__init__()
        self.axes = ['X','Y','Z','A','B']
        self.conn = conn
        self.data = ""    

    def run(self):
        while True:
            bytes_received = self.conn.recv(4096)
            string_received = ''
            self.data = self.data + string_received
            if self.data.endswith(u"\n"):
                process_data_points(self.data)
                self.data = ""

    def convertBin(num):
        return ''.join(bin(ord(c)).replace('0b', '').rjust(8, '0') for c in struct.pack('!f', num))

    def writeLineUTF(self,data):
        self.conn.send((data+'\r').encode('utf-8'))
        #data = data + '\r'
        #self.conn.send(binascii.hexlify(data))

    def writeLineBin(self,data):
        #self.conn.send((data+'\r').encode('utf-8'))
        
        data = data + '\r'
        self.conn.send(binascii.hexlify(bytes(data)))

    def write(self,data):
        self.conn.send(data.encode('utf-8'))

    def login(self):
        self.conn.send('hello EMC roby 1\rset enable EMCTOO\rset machine on\rset mode auto\r'.encode('utf-8'))

    def commandPoints(self,points,lines,blocklen):
        self.conn.send('dm '.encode('utf-8'))
        self.conn.send(convertBin(lines))
        self.conn.send(convertBin(blocklen))
        for point in points:
            self.conn.send(convertBin(point))
        self.conn.send('\r'.encode('utf-8'))

    def formatPoints(self, filename, blocklen):
        pts = np.loadtxt(filename)/25.4;

        # Pad with zeros to make array length correct
        pts = np.pad(pts, [(0,(blocklen - pts.shape[0]) % blocklen),(0,0)], 'edge');
        coords = [np.reshape(pts[:,index],(-1,blocklen)) for index in np.arange(0,3)]
        coords.append(90 * np.ones_like(coords[0]))
        coords.append(np.zeros_like(coords[0]))
        return np.asarray(coords)

    def writePoints(self,filename,blocklen,ic):
        #points = self.formatPoints(filename,blocklen)
        self.writeLine('hello EMC roby 1')
        self.writeLine('set enable EMCTOO')
        self.writeLine('set machine on')
        #self.writeLine('set echo off')
        self.writeLine('set mode mdi')

        #Set initial position
        # ic[0:3] = points[0:3,0,0]       
        icLine = ' '.join(axis + str(icc) for axis, icc in zip(self.axes, ic))
        #icLine = 'X0Y0Z0A45B0'
        self.writeLine('set mdi G0'+icLine)
        time.sleep(2)
        self.writeLine('set mode auto')
        time.sleep(1)

        file = open(filename)
        for line in file:
            #time.sleep(0.015)
            time.sleep(0.02)
            self.writeLine(line)

        #points_str = np.char.mod('%.4f', points)
        #for ndx in range(points.shape[1]):
            #axisLine = '&'
            #points_list = map(str,points[0,0,:])
            #axis_lines = ['|'.join(points_str[ndx_axis,ndx,:].tolist()) for ndx_axis in range(points.shape[0])]
            #axis_line = '&'.join(axis_lines)
            #self.writeLine('dm '+axis_line)
            #time.sleep(0.02)
            #print(axis_line)
        #self.conn.send('hello EMC

    def close(self):
        self.conn.close()
        #sys.exit()

serialLock = threading.Lock()
class serialInterface(threading.Thread):
    def __init__(self):
        super(serialInterface, self).__init__()
        #self.serial_port = serial.Serial('COM9', 115200)
        self.serialPort = serial.Serial('COM9', 250000)
        print('serialInterface started')
    
    def read(self):
        line = self.serialPort.readline()
        return line

##    def write(self, data):
##        line = self.serial_port.readline()
##        return line

    def reset(self):
        global encoderData, serialLock
        serialLock.acquire()
        self.serialPort.close()
        self.serialPort.open()
        encoderData = np.array([0])
        queueData = np.array([0])
        serialLock.release()

    def setEncoderCount(self,count):
        #self.serial_port.write('R'.encode('utf-8'))
        #Calculate number of characters in set command
        numBytes = math.floor(math.log(count,10))+1
        commandStr = 'S' + str(numBytes) + str(count)
        self.serialPort.write(commandStr.encode('utf-8'))

    def requestEncoderCount(self):
        self.serialPort.write('G'.encode('utf-8'))

    def run(self):
        global encoderData, serialLock
        self.setEncoderCount(1000)
        while True:
            serialLock.acquire()
            self.requestEncoderCount()
            line = self.read().decode("utf-8").strip()
            #print(line)
            axisCounts = line.split(' ')
            #print(axisCounts)
            if line and str.isnumeric(axisCounts[0]):
                #print(axisCounts[0], axisCounts[1])
                #print('got numeric');
                #encoderData = np.vstack((encoderData,int(line)))
                encoderData = np.vstack((encoderData,axisCounts[0]))
            serialLock.release()
            time.sleep(0.04)

def commInit():
    global control, feedback, bokehIntf, serialIntf
    # Run sockets
    try:
        #establish listener for rsyslog
        socket_fb = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        socket_fb.bind((feedbackhost,feedbackport))
    except socket.error:
        print ('Failed to create feedback socket')
        sys.exit()

    try:
        #establish control connection
        socket_ctrl = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        socket_ctrl.connect((controlhost,controlport))
    except socket.error:
        print ('Failed to create control socket')
        sys.exit()
        
    try:
        #establish writer to bokeh
        socket_bokeh = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #socket_bokeh.bind((bokehHost,bokehPort))
    except socket.error:
        print ('Failed to create Bokeh socket')
        sys.exit()
        
    print ('[+] Listening for feedback data on port', feedbackport)
    print ('[+] Connection to emcrsh established at address', controlhost,'on port', controlport)
    print ('[+] Bokeh interface socket opened successfully')

    bokehIntf = bokehInterface(socket_bokeh)
    bokehIntf.start()

    feedback = fb_server(socket_fb)
    feedback.start()

    control = control_client(socket_ctrl)
    control.start()

    ##serialIntf = serialInterface()
    ##serialIntf.start()

    #Read program data
    #filename = 'E:\SculptPrint\PocketNC\Position Sampling\Diva Head\servo_samples'
    #filename = 'C:\Users\Roby\VirtualBox VMs\File Transfers\Position Sampling\Files to Send\xyzabposn_tcp_25_4.ngc'
    

    #Run
    #control.writeLine('hello EMC roby 1\rset enable EMCTOO')
    ##control.writePoints(filename,blocklen,[0,0,0,45,0])

##def main():
##    if __name__ == '__main__':
##        #pass
##        if feedback.needUpdate:
##            #updatePlots(feedback.tcqFH,queueData,[1])
##            print('feedback needs update')
##            #bokehIntf.write(queueData)
##            feedback.needUpdate = 0
##
##commInit()
##
##while True:
##    main()
