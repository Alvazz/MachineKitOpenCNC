import threading
import sys
import numpy as np
import struct
import time
import csv
import re

Xpts = r'E:\SculptPrint\PocketNC\Position Sampling\Xpts_opt.csv'
Ypts = r'E:\SculptPrint\PocketNC\Position Sampling\Ypts_opt.csv'
Zpts = r'E:\SculptPrint\PocketNC\Position Sampling\Zpts_opt.csv'
Apts = r'E:\SculptPrint\PocketNC\Position Sampling\Apts_opt.csv'
Bpts = r'E:\SculptPrint\PocketNC\Position Sampling\Bpts_opt.csv'
points_file = 'E:\SculptPrint\PocketNC\Position Sampling\Diva Head\opt_code'

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
                print(complete_data_string)
                self.received_data_string = ''.join(split_received_string[1:])
            else:
                self.received_data_string += string_received

            if any(s in complete_data_string for s in self.machine.rsh_feedback_strings):
                if self.machine.rsh_feedback_strings[0] in string_received:
                    #Buffer Length
                    self.processRSHFeedback(complete_data_string)
                elif self.machine.rsh_feedback_strings[1] in string_received:
                    #Program Status
                    print('program status')
                    self.machine.status = complete_data_string.split(' ')[1].strip()
                elif self.machine.rsh_feedback_strings[2] in string_received:
                    #Machine Mode
                    print('mode set')
                    self.machine.mode = complete_data_string.split(' ')[1].strip()


    ### Data Handling ###
    def convertFloat2Bin(self, num):
        return struct.pack('!f', num)

    def convertInt2Bin(self, num):
        return struct.pack('!i',num)

    def padAndFormatAxisPoints(self, points, polylines, blocklength):
        #print(points.shape, blocklength, np.size(points, 0), blocklength-(np.size(points,0)%blocklength))
        pad_points = np.lib.pad(points,((0, blocklength-(np.size(points,0)%blocklength)),(0,0)),'constant',constant_values=points[-1])
        shape_points = pad_points.reshape((-1,blocklength),order='C')
        #print(shape_points.shape)
        return np.pad(shape_points,((0,polylines-(np.size(shape_points,0)%polylines)),(0,0)),'constant',constant_values=shape_points[-1,-1])   

    def importAxisPoints(self, file, polylines, blocklength):
        points = np.array(list(csv.reader(open(file, "rt"), delimiter=","))).astype("float")
        return self.padAndFormatAxisPoints(points, polylines, blocklength)
        
    def importAxesPoints(self, file, polylines, blocklength):
        points = np.array(list(csv.reader(open(file, "rt"), delimiter=" "))).astype("float")
        axis_coords = []
        for axis in range(points.shape[1]):
            axis_coords.append(self.padAndFormatAxisPoints(np.asarray([points[:,axis]]).T, polylines, blocklength))
        return np.asarray(axis_coords).transpose(1,2,0)

    def processRSHFeedback(self, fbString):
        m = re.search(self.machine.rsh_feedback_strings[0] + '(\d+)', fbString)
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
        self.setDrivePower(1)
        self.setAutoMode()
        self.setEcho(0)
        self.setMachineUnits('mm')

    def getProgramStatus(self):
        self.writeLineUTF('get program_status')
        return

    def getMachineMode(self):
        self.writeLineUTF('get mode')

    def setBinaryMode(self, flag):
        if flag:
            print('setting binary mode on')
            self.writeLineUTF('set comm_mode binary')
            self.binaryMode = 1
        else:
            print('setting binary mode off')
            self.conn.send(struct.pack('!f',-np.inf))
            self.binaryMode = 0
            time.sleep(0.05)

    def setAutoMode(self):
        self.writeLineUTF('set mode auto')
        #self.machine.mode = 'AUTO'

    def setMDIMode(self):
        self.writeLineUTF('set mode mdi')
        #self.machine.mode = 'MDI'

    def setManualMode(self):
        self.writeLineUTF('set mode manual')
        #self.machine.mode = 'MANUAL'

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

        self.machine.units = 'mm'
        

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
    def MDICommandWaitDone(self, timeout = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            self.getProgramStatus()
            print(self.machine.status)
            if self.machine.status == 'IDLE':
                return True
        return False

    def checkMachineReady(self, timeout = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            self.getProgramStatus()
            self.getMachineMode()
            if self.machine.status == 'IDLE' and self.machine.mode == 'AUTO':
                return True
        return False

    def resetPosition(self,X,Y,Z,A,B):
        self.writeLineUTF('set mode mdi')
        self.writeLineUTF('set mdi g0x'+str(X)+'y'+str(Y)+'z'+str(Z)+'a'+str(A)+'b'+str(B))

    def loadPoints(self, points_file, polylines, blocklength):
        self.commanded_points = self.importAxesPoints(points_file, polylines, blocklength)
        return self.commanded_points

    def commandPoints(self,polylines,blocklength,commands_to_send):
        X = self.importAxisPoints(Xpts,polylines,blocklength)
        Y = self.importAxisPoints(Ypts,polylines,blocklength)
        Z = self.importAxisPoints(Zpts,polylines,blocklength)
        A = self.importAxisPoints(Apts,polylines,blocklength)
        B = self.importAxisPoints(Bpts,polylines,blocklength)
        #(X, Y, Z, A, B) = self.importAxesPoints(points_file)
        #axisCoords = np.stack((X,Y,Z,A,B),axis=2)
        axisCoords = np.stack((X,Y,Z,30+np.zeros_like(A),1+np.zeros_like(B)),axis=2)
        print(axisCoords)
        
        #axisCoords = self.loadPoints(points_file, polylines, blocklength)
        #print(axisCoords)
        #axisCoords[:,:,:3] = -0.5 * np.ones_like(axisCoords[:,:,:3])
        #axisCoords[:,:,3:] = np.zeros_like(axisCoords[:,:,3:])

        if self.machine.units == 'mm':
            scale_translational = 25.4
        else:
            scale_translational = 1.0
        scale_translational = 1.0/25.4
            
        self.resetPosition(axisCoords[0][0][0],
                           axisCoords[0][0][1],
                           axisCoords[0][0][2],
                           axisCoords[0][0][3],axisCoords[0][0][4])
        print(axisCoords[0][0][0]*scale_translational,axisCoords[0][0][1]*scale_translational,axisCoords[0][0][2]*scale_translational,axisCoords[0][0][3],axisCoords[0][0][4])

        self.MDICommandWaitDone(5)
        self.setAutoMode()
        #time.sleep(1)
        #time.sleep(1)
        self.checkMachineReady(2)
        self.setBinaryMode(1)

        if commands_to_send == -1:
            print("sending all")
            commands_to_send = int(axisCoords.shape[0]/polylines)
            print(commands_to_send)
        
        for command in range(0,commands_to_send):
            frame = bytearray()
            frame.extend(self.convertInt2Bin(polylines))
            frame.extend(self.convertInt2Bin(blocklength))
            for polyLine in range(0,polylines):
                #print(command)
                for axis in range(0,axisCoords.shape[2]):
                    if axis <= 2:
                        scale = scale_translational
                    else:
                        scale = 1
                    for point in range(0,axisCoords.shape[1]):
                        frame.extend(self.convertFloat2Bin(axisCoords[(command*polylines)+polyLine,point,axis]*scale))
                        #print(axisCoords[(command*polylines)+polyLine,point,axis]*scale)
                        
            frame.extend(struct.pack('!f',np.inf))
            self.conn.send(frame)

            sleep_time = self.runNetworkPID(self.rsh_buffer_length,blocklength,polylines,1000)
            #print(sleep_time, self.rsh_buffer_length)
            time.sleep(sleep_time)

        self.setBinaryMode(0)
        #self.resetPosition()

    def runNetworkPID(self,current_buffer_length,block_length,polylines,set_point_buffer_length,Kp=.01,Ki=0,Kd=0):
        #sleepTime = (blockLength*polyLines)/1000 - (Kp*((1000-self.rshQueueData[-1])))/1000
        sleep_time = max((block_length*polylines)/1000 - (Kp*((set_point_buffer_length-current_buffer_length)))/1000,0)
        return sleep_time

    def formatPoints(self,filename,blocklen):
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
