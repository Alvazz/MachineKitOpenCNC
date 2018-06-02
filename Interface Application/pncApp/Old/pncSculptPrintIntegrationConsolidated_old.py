import socket
import sys

import threading
import numpy as np
import struct
import time
import csv

import re

import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import time
import pickle
from tkinter import *
import serial
import math
import binascii
import struct
import csv

# Default connection parameters
def_feedback_listen_ip = '0.0.0.0'
def_feedback_listen_port = 514
def_control_client_ip = '129.1.15.5'
def_control_client_port = 5007

# Initialize control communication with PocketNC using TCP and feedback read
# communication with UDP.
def appInit(feedback_listen_ip = def_feedback_listen_ip,
             feedback_listen_port = def_feedback_listen_port,
             control_client_ip = def_control_client_ip,
             control_client_port = def_control_client_port):
    global data_source, machine_controller

    machine = MachineModel()

    try:
        feedback_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        feedback_socket.bind((feedback_listen_ip, feedback_listen_port))
    except socket.error:
        print ('Failed to bind to feedback socket to listen on')
        sys.exit()

    try:
        control_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        control_socket.connect((control_client_ip, control_client_port))
    except socket.error:
        print ('Failed to connect to client ip for giving it control')
        sys.exit()      

    print ('[+] Listening for feedback data on port', feedback_listen_port)
    print ('[+] Connection to control client (emcrsh) established at address',
                control_client_ip,'on port', control_client_port)

    feedback_listener = MachineFeedbackListener(feedback_socket, data_source)
    feedback_listener.start()

    machine_controller = MachineController(control_socket, machine, data_source)
    machine_controller.start()

# Global variables
data_source = DataStore()
machine_controller = []


##############

# Connection Parameters
feedbackhost = '0.0.0.0'
feedbackport = 514
controlhost = '129.1.15.5'
#controlhost = '127.0.0.1'
controlport = 5007
bokehHost = '127.0.0.1'
bokehPort = 6666

# Control parameters
filename = r'C:\Users\Roby\VirtualBox VMs\File Transfers\Position Sampling\Files to Send\xyzabposn_tcp_25_4.ngc'
Xpts = r'E:\SculptPrint\PocketNC\Position Sampling\Xpts_opt.csv'
Ypts = r'E:\SculptPrint\PocketNC\Position Sampling\Ypts_opt.csv'
Zpts = r'E:\SculptPrint\PocketNC\Position Sampling\Zpts_opt.csv'
Apts = r'E:\SculptPrint\PocketNC\Position Sampling\Apts_opt.csv'
Bpts = r'E:\SculptPrint\PocketNC\Position Sampling\Bpts_opt.csv'
blocklen = 25

#connectionSevered=0

# Machine Parameters
mach_offsets = np.array([2e-5, 32e-5, 22e-5, 1e-5, 359.976]);
prog_offsets = np.array([0, 0, 0, -90, 0]);

# Data Collection
commandData = np.array([0,0,0,0,0])
feedbackData = np.array([0,0,0,0,0])
encoderData = np.array([1000000])

rshQueueData = np.array([0])
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

class bokehInterface(threading.Thread):
    def __init__(self, socket):
        super(bokehInterface, self).__init__()
        self.socket = socket
        print('bokehInterface started')

    def init():
        pass
    
    def write(self,data):
        self.socket.sendto(pickle.dumps(data),(bokehHost,bokehPort))

#Server Thread to Hold Data from RSH and rsyslog
class dataManager(threading.Thread):
    def __init__(self, conn):
        super(dataManager, self).__init__()
        #self.tcqPlotData = []
        self.needUpdate = 0
        self.conn = conn
        self.data = ""
        print('Data Management thread started')

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
            #print(string_received)
            if self.data.endswith(u"*|"):
                #print('Received complete string', string_received)
                process_data_points(self.data)
                self.data = ""
                self.needUpdate = 1
                #print('feedback needs update')
                dataDict['TCQ'] = queueData
                dataDict['Thread'] = threadData
                dataDict['Encoder'] = encoderData
                #bokehIntf.write([queueData threadData])
                #print(queueData[-1])
                bokehIntf.write(dataDict)

    def initPlots(self):
        print('initializing plots')

    def send_msg(self,msg):
        self.conn.send(msg)

    def close(self):
        self.conn.close()

class control_client(threading.Thread):
    ### Load Data Storage ###
    rshQueueData = np.array([0])
    rshFeedbackStrings = ['PROGRAM_STATUS', 'bL']
    
    def __init__(self, conn):
        super(control_client, self).__init__()
        self.axes = ['X','Y','Z','A','B']
        self.conn = conn
        self.data = ""

        #State variables
        self.modes = ['manual', 'mdi', 'auto']
        self.mode = 'manual'
        self.binaryMode = 0
        self.loggingMode = 0
        self.status = 'idle'
        
    def run(self):
        while True:
            #print('running control feedback')
            bytes_received = self.conn.recv(4096)
            #print(bytes_received)
            string_received = bytes_received.decode("utf-8")
            if 'bL=' in string_received and '\n' in string_received:
                #print(string_received)
                self.data = self.data + string_received
                if self.data.endswith(u"\n"):
                    #process_data_points(self.data)
                    self.processRSHFeedback(self.data)
                    self.data = ""

    ### Data Handling ###
    def convertFloat2Bin(self, num):
        return struct.pack('!f', num)

    def convertInt2Bin(self, num):
        return struct.pack('!i',num)
    
    def importPoints(self, file, polyLines, blockLength):
        points = np.array(list(csv.reader(open(file, "rt"), delimiter=","))).astype("float")
        padPoints = np.lib.pad(points,((0,blockLength-(np.size(points,0)%blockLength)),(0,0)),'constant',constant_values=points[-1])
        shapePoints = padPoints.reshape((-1,blockLength),order='C')
        return np.pad(shapePoints,((0,polyLines-(np.size(shapePoints,0)%polyLines)),(0,0)),'constant',constant_values=shapePoints[-1,-1])

    def processRSHFeedback(self, fbString):
        m = re.search('bL=(.+)', fbString)
        rshBufLen = int(m.group(1))
        #print(rshBufLen)
        #self.rshQueueData = np.vstack((self.rshQueueData, rshBufLen))
        self.rshQueueData = np.append(self.rshQueueData, rshBufLen)
        #print(self.rshQueueData)

    def formatBinaryLine(self,axisCoords,polyLines,blockLength,positionInFile):
        return #position in file#
        
    ### Writing Functions ###
    def writeLineUTF(self,data):
        self.conn.send((data+'\r').encode('utf-8'))

##    def writeLineBin(self,data):
##        data = data + '\r'
##        self.conn.send(binascii.hexlify(bytes(data)))

    def write(self,data):
        self.conn.send(data.encode('utf-8'))


    ### API for Common RSH Commands ###
    def login(self):
        self.writeLineUTF('hello EMC robie 1')
        self.writeLineUTF('set enable EMCTOO')
        self.setAutoMode()
        self.setDrivePower(1)
        self.setEcho(0)
        self.setMachineUnits('mm')
        #self.writeLineUTF('set machine on')
        #self.writeLineUTF('set mode auto')
        #self.writeLineUTF('

    def getProgramStatus(self):
        self.writeLineUTF('get program_status')
        return

    def setBinaryMode(self, flag):
        if flag:
            print('setting binary mode on')
            self.writeLineUTF('set comm_mode binary')
            self.binaryMode = 1
        else:
            print('setting binary mode off')
            self.conn.send(struct.pack('!f',-np.inf))
            self.binaryMode = 0
        time.sleep(0.1)

    def setAutoMode(self):
        self.writeLineUTF('set mode auto')
        time.sleep(0.02)
        self.mode = 'auto'

    def setMDIMode(self):
        self.writeLineUTF('set mode mdi')
        time.sleep(0.02)
        self.mode = 'mdi'

    def setManualMode(self):
        self.writeLineUTF('set mode manual')
        time.sleep(0.02)
        self.mode = 'manual'

    def setMDILine(self,line):
        self.writeLineUTF('set mdi ' + line)
        time.sleep(0.02)

    def setDrivePower(self,flag):
        sendstr = 'set machine '
        if flag:
            sendstr += 'on'
        else:
            sendstr += 'off'
        self.writeLineUTF(sendstr)

    def setLogging(self, flag):
        if (flag != self.loggingMode):
            #Need to toggle logging mode
            if self.binaryMode:
                print('disabling binary')
                self.setBinaryMode(0)
                self.setMDIMode()
                self.setMDILine('G68')
                self.setAutoMode()
                self.setBinaryMode(1)
                self.loggingMode = not(self.loggingMode)
            else:
                self.setMDIMode()
                self.setMDILine('G68')
                self.setAutoMode()
                self.loggingMode = not(self.loggingMode)
        print('logging mode ' + str(int(self.loggingMode)))

    def setMachineUnits(self,units):
        sendstr = 'set units '
        if units == 'mm':
            sendstr += 'mm'
        elif units == 'inch':
            sendstr += 'inches'
        self.writeLineUTF(sendstr)

        self.setMDIMode()
        self.setMDILine('G21')

    def setEcho(self,flag):
        sendstr = 'set echo '
        if flag:
            sendstr += 'on'
        else:
            sendstr += 'off'
        control.writeLineUTF(sendstr)

    def setHome(self, fX, fY, fZ, fA, fB):
        self.saveState()
        self.setManualMode()
        sendstr = 'set home '
        if fX:
            sendstr = sendstr + '0 '
        if fY:
            sendstr = sendstr + '1 '
        if fZ:
            sendstr = sendstr + '2 '
        if fA:
            sendstr = sendstr + '3 '
        if fB:
            sendstr = sendstr + '4 '
        self.writeLineUTF(sendstr)

    ### Control Functions ###
    def resetPosition(self,X,Y,Z,A,B):
        self.writeLineUTF('set mode mdi')
        time.sleep(0.01)
        self.writeLineUTF('set mdi g0x'+str(X)+'y'+str(Y)+'z'+str(Z)+'a'+str(A)+'b'+str(B))
        time.sleep(1)

    def commandPoints(self,polyLines,blockLength,commandsToSend):
        X = self.importPoints(Xpts,polyLines,blockLength)
        Y = self.importPoints(Ypts,polyLines,blockLength)
        Z = self.importPoints(Zpts,polyLines,blockLength)
        A = self.importPoints(Apts,polyLines,blockLength)
        B = self.importPoints(Bpts,polyLines,blockLength)
        axisCoords = np.stack((X,Y,Z,A,B),axis=2)

        axisCoords = np.stack((X,Y,Z,30+np.zeros_like(A),np.zeros_like(B)),axis=2)
        

        self.resetPosition(axisCoords[0][0][0],axisCoords[0][0][1],axisCoords[0][0][2],axisCoords[0][0][3],axisCoords[0][0][4])
        time.sleep(3)
        
        self.setAutoMode()
        self.setBinaryMode(1)

        if commandsToSend == -1:
            print("sending all")
            commandsToSend = int(axisCoords.shape[0]/polyLines)
            print(commandsToSend)
        
        for command in range(0,commandsToSend):
            frame = bytearray()
            frame.extend(self.convertInt2Bin(polyLines))
            frame.extend(self.convertInt2Bin(blockLength))
            for polyLine in range(0,polyLines):
                print(command)
                for axis in range(0,axisCoords.shape[2]):
                    if axis <= 2:
                        scale = 1/25.4
                    else:
                        scale = 1
                    for point in range(0,axisCoords.shape[1]):
                        frame.extend(self.convertFloat2Bin(axisCoords[(command*polyLines)+polyLine,point,axis]*scale))
                        
            frame.extend(struct.pack('!f',np.inf))
            self.conn.send(frame)

            #if queueData[-1] < 1000:
            Kp = 1
            #sleepTime = 0.1 + 1/(Kp*(1000-queueData[-1]))
            sleepTime = (blockLength*polyLines)/1000 - (Kp*((1000-self.rshQueueData[-1])))/1000
            #sleepTime = self./1000 - Kp/((1000-self.rshQueueData[-1]))
            #print(sleepTime, queueData[-1])
            print(sleepTime, self.rshQueueData[-1])
            if sleepTime > 0:
                time.sleep(sleepTime)
            #time.sleep(0.01)

        time.sleep(2)
        self.setBinaryMode(0)
        #self.resetPosition()

    def runNetworkPID(Kp,Ki,Kd,rshBufLen, setPoint):
        return #sleeptime

    ### State Machine ###
    def saveState(self):
        #save machine state
        return #saved state structure

    def restoreState(self, stateStruct):
        #Restore state to prev state after op
        return

    def formatPoints(self, filename, blocklen):
        pts = np.loadtxt(filename)/25.4;
        # Pad with zeros to make array length correct
        pts = np.pad(pts, [(0,(blocklen - pts.shape[0]) % blocklen),(0,0)], 'edge');
        coords = [np.reshape(pts[:,index],(-1,blocklen)) for index in np.arange(0,3)]
        coords.append(90 * np.ones_like(coords[0]))
        coords.append(np.zeros_like(coords[0]))
        return np.asarray(coords)

##    def writePoints(self,filename,blocklen,ic):
##        #points = self.formatPoints(filename,blocklen)
##        self.writeLine('hello EMC roby 1')
##        self.writeLine('set enable EMCTOO')
##        self.writeLine('set machine on')
##        #self.writeLine('set echo off')
##        self.writeLine('set mode mdi')
##
##        #Set initial position
##        # ic[0:3] = points[0:3,0,0]       
##        icLine = ' '.join(axis + str(icc) for axis, icc in zip(self.axes, ic))
##        #icLine = 'X0Y0Z0A45B0'
##        self.writeLine('set mdi G0'+icLine)
##        time.sleep(2)
##        self.writeLine('set mode auto')
##        time.sleep(1)
##
##        file = open(filename)
##        for line in file:
##            #time.sleep(0.015)
##            time.sleep(0.02)
##            self.writeLine(line)



    def close(self):
        self.conn.close()
        #sys.exit()

serialLock = threading.Lock()
class serialInterface(threading.Thread):
    def __init__(self):
        super(serialInterface, self).__init__()
        #self.serial_port = serial.Serial('COM9', 115200)
        self.serialPort = serial.Serial('COM12', 250000)
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

    #serialIntf = serialInterface()
    #serialIntf.start()

    #Read program data
    #filename = 'E:\SculptPrint\PocketNC\Position Sampling\Diva Head\servo_samples'
    #filename = 'C:\Users\Roby\VirtualBox VMs\File Transfers\Position Sampling\Files to Send\xyzabposn_tcp_25_4.ngc'
    
######### SCULPTPRINT INTEGRATION CODE #########

def start():
    commInit()
    return True

def read():
    #return feedbackData[-1]
    return [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]]

# Returns true if monitoring is currently happening.
def isMonitoring():
    #global feed_thread
    return feedback.is_alive()
    #return True

# Called to stop monitoring the machine.
# Will execute when the stop button is pressed in the Monitor Machine feature.
def stop():
    #global feed_thread
    if feedback.is_alive():
        print('Feed thread is still alive')
    else:
        print('Feed thread is not alive')
    #feedback.deactivate()
    #feedback.join()
    #feedback.close()
    print('Buffer file was closed.\n')
    return True;
































##################################################
# Store feedback data from other modules
# A Machine Feedback record is of the following form:
#    a) A time stamp T of when the record is generated.
#    b) A set of n commanded joint space positions before T
#    c) A set of n stepgen feedback positions before T
#    d) n time values approximating when each stepgen feedback point was generated
#    e) A snapshot of the tcq length at T
class DataStore():
    def __init__(self):
        #Timers for each data source
        self.machine_running_time = 0
        self.encoder_running_time = 0

        #Counters for number of records
        self.machine_feedback_num_records = 0
        self.encoder_feedback_num_records = 0


        ### DATA STORES ###
        #Timers -- time_delta - 1 for each record, times_interpolated 
        self.machine_time_delta = np.empty(1,dtype=float)
        self.machine_times_interpolated = np.empty(1,dtype=float)

        #Buffer fill level
        self.machine_tc_queue_length = np.empty(1,dtype=int)
        self.highres_tc_queue_length = np.empty(1,dtype=int)

        #Positions from stepgen and encoders
        self.commanded_joint_positions = np.empty([1,5],dtype=float)

        self.stepgen_feedback_positions = np.empty([1,5],dtype=float)
        self.encoder_feedback_positions = np.empty([1,5],dtype=float)

        #Thread counter
        self.rt_thread_num_executions_delta = np.empty(1,dtype=int)


    def appendMachineFeedbackRecords(self, records):
        for record in records:
            self.commanded_joint_positions = np.vstack((self.commanded_joint_positions, record['commanded_joint_positions']))
            self.stepgen_feedback_positions = np.vstack((self.stepgen_feedback_positions, record['stepgen_feedback_positions']))

            self.machine_time_delta = np.append(self.machine_time_delta, record['machine_time_delta'])
            self.machine_times_interpolated = np.append(self.machine_times_interpolated, record['machine_times_interpolated'])
            self.machine_tc_queue_length = np.append(self.machine_tc_queue_length, record['machine_tcq_length'])
            self.rt_thread_num_executions_delta = np.append(self.rt_thread_num_executions_delta, record['rt_thread_num_executions_delta'])
            
            self.machine_feedback_num_records += 1
            self.machine_running_time += record['machine_time_delta']

    def appendMachineControlRecords(self, records):
        for record in records:
            self.highres_tc_queue_length = np.append(self.highres_tc_queue_length, record['highres_tcq_length'])

    def appendEncoderFeedbackRecords(self, records):
        for record in records:
            pass

class MachineModel():
    def __init__(self):
        #State variables
        self.modes = ['MANUAL', 'MDI', 'AUTO']
        self.statuses = ['IDLE', 'RUNNING', 'PAUSED']
        self.rsh_feedback_strings = ['bL=', 'PROGRAM_STATUS']

        self.axes = ['X','Y','Z','A','B']
        self.mode = self.modes[0]
        self.status = self.statuses[0]
        self.binaryMode = 0
        self.loggingMode = 0
        

    ### State Machine ###
    def saveState(self):
        #save machine state
        return #saved state structure

    def restoreState(self, stateStruct):
        #Restore state to prev state after op
        return


class MachineFeedbackListener(threading.Thread):
    def __init__(self, conn, data_store):
        super(MachineFeedbackListener, self).__init__()
        self.conn = conn
        self.data_store = data_store
        self.received_data_string = ""
        print('Feedback thread started')

    def run(self):
        while True:
            (bytes_received, rec_address) = self.conn.recvfrom(65536)
            string_received = bytes_received.decode("utf-8")
            self.received_data_string += string_received

            # A complete record of machine data has been received
            if self.received_data_string.endswith(u"*|"):
                machine_feedback_records = self.processMachineDataString(self.received_data_string)
                self.data_store.appendMachineFeedbackRecords(machine_feedback_records)
                self.received_data_string = ""

    def processMachineDataString(self, machine_data_string):
        machine_feedback_records = []
        if '0: tc' in machine_data_string:
            parsed_string = re.search('tcqLen(.+?)T(.+?)dC(.+?):(.+)', machine_data_string)
            if parsed_string:
                record = dict()

                feedback_num_points = machine_data_string.count('&')
                tcq_length = float(parsed_string.group(1)) 
                delta_thread_cycles = float(parsed_string.group(2))
                delta_machine_clock = float(parsed_string.group(3))

                data_points = parsed_string.group(4).strip()
                data_points = re.sub('S[0-9]*:', '', data_points)
                
                #Each time sample delimited with &
                samples = data_points.split('&')
                coords = [sample.split('|') for sample in samples]
                coords = np.asarray([[float(coord) for coord in coords[index][:-1]] for index in range(len(coords[:-1]))])
                
                commanded_joint_positions = coords[:,0:5]
                stepgen_feedback_positions = coords[:,5:]

                #Store parsed data
                record['machine_time_delta'] = delta_machine_clock
                record['machine_times_interpolated'] = np.linspace(delta_machine_clock/feedback_num_points,
                                                                   delta_machine_clock,feedback_num_points)
                record['machine_tcq_length'] = tcq_length              
                record['commanded_joint_positions'] = commanded_joint_positions
                record['stepgen_feedback_positions'] = stepgen_feedback_positions
                record['rt_thread_num_executions_delta'] = delta_thread_cycles

                machine_feedback_records.append(record)
        return machine_feedback_records

    def close(self):
        self.conn.close()


Xpts = r'E:\SculptPrint\PocketNC\Position Sampling\Xpts_opt.csv'
Ypts = r'E:\SculptPrint\PocketNC\Position Sampling\Ypts_opt.csv'
Zpts = r'E:\SculptPrint\PocketNC\Position Sampling\Zpts_opt.csv'
Apts = r'E:\SculptPrint\PocketNC\Position Sampling\Apts_opt.csv'
Bpts = r'E:\SculptPrint\PocketNC\Position Sampling\Bpts_opt.csv'

class MachineController(threading.Thread):   
    def __init__(self, conn, machine, data_store):
        super(MachineController, self).__init__()
        
        self.conn = conn
        self.received_data_string = ""
        
        self.data_store = data_store
        self.rsh_buffer_length = 0
        self.machine = machine
        
    def run(self):
        while True:
            #Process feedback from EMCRSH
            string_received = self.conn.recv(4096).decode("utf-8")
            complete_data_string = ''
            if '\n' in string_received:
                
                split_received_string = string_received.split('\n')
                complete_data_string = self.received_data_string + split_received_string[0]
                self.received_data_string = ''.join(split_received_string[1:])
                print(complete_data_string)
            else:
                self.received_data_string += string_received
                
            if any(s in complete_data_string for s in self.machine.rsh_feedback_strings):
                if self.machine.rsh_feedback_strings[0] in string_received:
                    #Buffer Length
                    self.processRSHFeedback(complete_data_string)
                elif self.machine.rsh_feedback_strings[1] in string_received:
                    #Program Status
                    self.machine.status = complete_data_string.split(' ')[1].strip()


    ### Data Handling ###
    def convertFloat2Bin(self, num):
        return struct.pack('!f', num)

    def convertInt2Bin(self, num):
        return struct.pack('!i',num)
    
    def importPoints(self, file, polyLines, blockLength):
        points = np.array(list(csv.reader(open(file, "rt"), delimiter=","))).astype("float")
        padPoints = np.lib.pad(points,((0,blockLength-(np.size(points,0)%blockLength)),(0,0)),'constant',constant_values=points[-1])
        shapePoints = padPoints.reshape((-1,blockLength),order='C')
        return np.pad(shapePoints,((0,polyLines-(np.size(shapePoints,0)%polyLines)),(0,0)),'constant',constant_values=shapePoints[-1,-1])

    def processRSHFeedback(self, fbString):
        m = re.search(self.machine.rsh_feedback_strings[0] + '(.+)', fbString)
        self.rsh_buffer_length = int(m.group(1))
        self.data_store.appendMachineControlRecords([dict([('highres_tcq_length', self.rsh_buffer_length)])])

    def formatBinaryLine(self,axisCoords,polyLines,blockLength,positionInFile):
        return #position in file#
        
    ### Writing Functions ###
    def writeLineUTF(self,data):
        self.conn.send((data+'\r').encode('utf-8'))
        time.sleep(0.1)

    def writeLineBin(self,frame):
        frame.extend(struct.pack('!f',np.inf))
        self.conn.send(frame)

    ### API for Common RSH Commands ###
    def login(self):
        self.writeLineUTF('hello EMC robie 1')
        self.writeLineUTF('set enable EMCTOO')
        self.setAutoMode()
        self.setDrivePower(1)
        self.setEcho(0)
        self.setMachineUnits('mm')

    def getProgramStatus(self):
        self.writeLineUTF('get program_status')
        return

    def setBinaryMode(self, flag):
        if flag:
            print('setting binary mode on')
            self.writeLineUTF('set comm_mode binary')
            self.binaryMode = 1
        else:
            print('setting binary mode off')
            self.conn.send(struct.pack('!f',-np.inf))
            self.binaryMode = 0
            time.sleep(0.02)

    def setAutoMode(self):
        self.writeLineUTF('set mode auto')
        self.mode = 'auto'

    def setMDIMode(self):
        self.writeLineUTF('set mode mdi')
        self.mode = 'mdi'

    def setManualMode(self):
        self.writeLineUTF('set mode manual')
        self.mode = 'manual'

    def setMDILine(self,line):
        self.writeLineUTF('set mdi ' + line)

    def setDrivePower(self,flag):
        sendstr = 'set machine '
        if flag:
            sendstr += 'on'
        else:
            sendstr += 'off'
        self.writeLineUTF(sendstr)

    def setLogging(self, flag):
        if (flag != self.loggingMode):
            #Need to toggle logging mode
            if self.binaryMode:
                print('disabling binary')
                self.setBinaryMode(0)
                self.setMDIMode()
                self.setMDILine('G68')
                self.setAutoMode()
                self.setBinaryMode(1)
                self.loggingMode = not(self.loggingMode)
            else:
                self.setMDIMode()
                self.setMDILine('G68')
                self.setAutoMode()
                self.loggingMode = not(self.loggingMode)
        print('logging mode ' + str(int(self.loggingMode)))

    def setMachineUnits(self,units):
        sendstr = 'set units '
        if units == 'mm':
            sendstr += 'mm'
        elif units == 'inch':
            sendstr += 'inches'
        self.writeLineUTF(sendstr)

        self.setMDIMode()
        self.setMDILine('G21')

    def setEcho(self,flag):
        sendstr = 'set echo '
        if flag:
            sendstr += 'on'
        else:
            sendstr += 'off'
        self.writeLineUTF(sendstr)

    def setHome(self, fX, fY, fZ, fA, fB):
        self.saveState()
        self.setManualMode()
        sendstr = 'set home '
        if fX:
            sendstr = sendstr + '0 '
        if fY:
            sendstr = sendstr + '1 '
        if fZ:
            sendstr = sendstr + '2 '
        if fA:
            sendstr = sendstr + '3 '
        if fB:
            sendstr = sendstr + '4 '
        self.writeLineUTF(sendstr)

    ### Control Functions ###
    def commandWaitDone(self, timeout = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            self.getProgramStatus()
            if self.machine.status == 'IDLE':
                return True
        return False

    def resetPosition(self,X,Y,Z,A,B):
        self.writeLineUTF('set mode mdi')
        self.writeLineUTF('set mdi g0x'+str(X)+'y'+str(Y)+'z'+str(Z)+'a'+str(A)+'b'+str(B))

    def commandPoints(self,polylines,block_length,commands_to_send):
        X = self.importPoints(Xpts,polylines,block_length)
        Y = self.importPoints(Ypts,polylines,block_length)
        Z = self.importPoints(Zpts,polylines,block_length)
        A = self.importPoints(Apts,polylines,block_length)
        B = self.importPoints(Bpts,polylines,block_length)
        axisCoords = np.stack((X,Y,Z,A,B),axis=2)

        axisCoords = np.stack((X,Y,Z,30+np.zeros_like(A),np.zeros_like(B)),axis=2)
        print(axisCoords)
        self.resetPosition(axisCoords[0][0][0],axisCoords[0][0][1],axisCoords[0][0][2],axisCoords[0][0][3],axisCoords[0][0][4])
        self.commandWaitDone(10)
        
        self.setAutoMode()
        self.setBinaryMode(1)

        if commands_to_send == -1:
            print("sending all")
            commands_to_send = int(axisCoords.shape[0]/polylines)
            print(commands_to_send)
        
        for command in range(0,commands_to_send):
            frame = bytearray()
            frame.extend(self.convertInt2Bin(polylines))
            frame.extend(self.convertInt2Bin(block_length))
            for polyLine in range(0,polylines):
                print(command)
                for axis in range(0,axisCoords.shape[2]):
                    if axis <= 2:
                        scale = 1.0/25.4
                    else:
                        scale = 1
                    for point in range(0,axisCoords.shape[1]):
                        frame.extend(self.convertFloat2Bin(axisCoords[(command*polylines)+polyLine,point,axis]*scale))
                        print(axisCoords[(command*polylines)+polyLine,point,axis]*scale)
                        
            frame.extend(struct.pack('!f',np.inf))
            self.conn.send(frame)

            sleep_time = self.runNetworkPID(self.rsh_buffer_length,block_length,polylines,1000)
            print(sleep_time, self.rsh_buffer_length)
            time.sleep(sleep_time)

        self.setBinaryMode(0)
        #self.resetPosition()

    def runNetworkPID(self,rsh_buffer_length,block_length,polylines,set_point_buffer_length,Kp=1,Ki=0,Kd=0):
        #sleepTime = (blockLength*polyLines)/1000 - (Kp*((1000-self.rshQueueData[-1])))/1000
        sleep_time = max((block_length*polylines)/1000 - (Kp*((set_point_buffer_length-rsh_buffer_length)))/1000,0)
        return sleep_time

    def formatPoints(self, filename, blocklen):
        pts = np.loadtxt(filename)/25.4;
        # Pad with zeros to make array length correct
        pts = np.pad(pts, [(0,(blocklen - pts.shape[0]) % blocklen),(0,0)], 'edge');
        coords = [np.reshape(pts[:,index],(-1,blocklen)) for index in np.arange(0,3)]
        coords.append(90 * np.ones_like(coords[0]))
        coords.append(np.zeros_like(coords[0]))
        return np.asarray(coords)

    def close(self):
        self.conn.close()
        #sys.exit()


# Default connection parameters
def_feedback_listen_ip = '0.0.0.0'
def_feedback_listen_port = 514
def_control_client_ip = '129.1.15.5'
def_control_client_port = 5007

# Initialize control communication with PocketNC using TCP and feedback read
# communication with UDP.
def appInit(feedback_listen_ip = def_feedback_listen_ip,
             feedback_listen_port = def_feedback_listen_port,
             control_client_ip = def_control_client_ip,
             control_client_port = def_control_client_port):
    global data_source, machine_controller

    machine = MachineModel()

    try:
        feedback_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        feedback_socket.bind((feedback_listen_ip, feedback_listen_port))
        pass
    except socket.error:
        print ('Failed to bind to feedback socket to listen on')
        sys.exit()

    try:
        control_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        control_socket.connect((control_client_ip, control_client_port))
        pass
    except socket.error:
        print ('Failed to connect to client ip for giving it control')
        sys.exit()      

    print ('[+] Listening for feedback data on port', feedback_listen_port)
    print ('[+] Connection to control client (emcrsh) established at address',
                control_client_ip,'on port', control_client_port)

    feedback_listener = MachineFeedbackListener(feedback_socket, data_source)
    feedback_listener.start()

    machine_controller = MachineController(control_socket, machine, data_source)
    machine_controller.start()


######### SCULPTPRINT INTEGRATION CODE #########
data_source = []
def start():
    global data_source
    #commInit()
    # Global variables
    machine_controller = []
    data_source = DataStore()
    appInit()
    return True

def read():
    #return feedbackData[-1]
    return [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]]

# Returns true if monitoring is currently happening.
def isMonitoring():
    #global feed_thread
    #return feedback.is_alive()
    return True

# Called to stop monitoring the machine.
# Will execute when the stop button is pressed in the Monitor Machine feature.
def stop():
    #global feed_thread
    #if feedback.is_alive():
    #    print('Feed thread is still alive')
    #else:
    #    print('Feed thread is not alive')
    #feedback.deactivate()
    #feedback.join()
    #feedback.close()
    #print('Buffer file was closed.\n')
    return True

