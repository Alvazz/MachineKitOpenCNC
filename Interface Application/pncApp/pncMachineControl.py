import threading
import sys
import numpy as np
import struct
import time
import csv
import re
import string
import socket

Xpts = r'E:\SculptPrint\PocketNC\Position Sampling\Xpts_opt.csv'
Ypts = r'E:\SculptPrint\PocketNC\Position Sampling\Ypts_opt.csv'
Zpts = r'E:\SculptPrint\PocketNC\Position Sampling\Zpts_opt.csv'
Apts = r'E:\SculptPrint\PocketNC\Position Sampling\Apts_opt.csv'
Bpts = r'E:\SculptPrint\PocketNC\Position Sampling\Bpts_opt.csv'
#points_file = 'E:\SculptPrint\PocketNC\Position Sampling\Diva Head\opt_code'
points_file = 'E:\SculptPrint\PocketNC\SP Integration\Example\opt_code2.75'
points_file = 'E:\SculptPrint\PocketNC\Standards\opt_code'

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
                print('complete data string ' + complete_data_string + '\n')
                self.received_data_string = ''.join(split_received_string[1:])
            else:
                self.received_data_string += string_received
            #print(self.received_data_string)
            if any(s in complete_data_string for s in self.machine.rsh_feedback_strings):
                #Check for error first
                if self.machine.rsh_feedback_strings[-1] in string_received:
                    #Error in RSH command
                    print('RSH error')
                    self.handleRSHError()
                if self.machine.rsh_feedback_strings[0] in string_received:
                    #Buffer Length
                    self.processRSHFeedback(complete_data_string)
                elif self.machine.rsh_feedback_strings[1] in string_received:
                    #Program Status
                    print('program status')
                    self.machine.status = complete_data_string.split(' ')[1].strip()
                    print('set program status to ' + self.machine.status)
                elif self.machine.rsh_feedback_strings[2] in string_received:
                    #Machine Mode
                    print('mode set')
                    self.machine.mode = complete_data_string.split(' ')[1].strip()
                    print('set machine mode to ' + self.machine.mode)

    def processRSHFeedback(self, fbString):
        m = re.search(self.machine.rsh_feedback_strings[0] + '(\d+)', fbString)
        self.rsh_buffer_length = int(m.group(1))
        self.data_store.appendMachineControlRecords([dict([('highres_tcq_length', self.rsh_buffer_length)])])

    def handleRSHError(self):
        self.machine.rsh_error = 1

    def resetRSHError(self):
        self.machine.rsh_error = 0

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

    def formatBinaryLine(self,axisCoords,polyLines,blockLength,positionInFile):
        return #position in file#
        
    ### Writing Functions ###
    def writeLineUTF(self,data):
        self.conn.send((data+'\r').encode('utf-8'))
        time.sleep(0.25)

    def writeLineBin(self,frame):
        frame.extend(struct.pack('!f',np.inf))
        self.conn.send(frame)

    ### API for Common RSH Commands ###
    def login(self):
        self.writeLineUTF('hello EMC robie 1')
        self.writeLineUTF('set enable EMCTOO')
        self.setEcho(0)
        self.setDrivePower(1)
        #self.setAutoMode()
        #self.setMachineMode('auto')
        self.modeSwitchWait('auto')
        self.setMachineUnits('inch')

    def getProgramStatus(self):
        self.writeLineUTF('get program_status')
        return

    def getMachineMode(self):
        self.writeLineUTF('get mode')
        
    def getDrivePower(self):
        self.writeLineUTF('get machine')

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

   # def setAutoMode(self, timeout = 2):
        #self.writeLineUTF('set mode auto')
        #self.modeSwitchWait('AUTO',timeout)
        #self.machine.mode = 'AUTO'

    #def setMDIMode(self, timeout = 2):
        #self.writeLineUTF('set mode mdi')
        #self.modeSwitchWait('MDI',timeout)
        #self.machine.mode = 'MDI'

    #def setManualMode(self, timeout = 2):
        #self.writeLineUTF('set mode manual')
        #self.modeSwitchWait('MANUAL',timeout)
        #self.machine.mode = 'MANUAL'

    def setMachineMode(self, mode, timeout = 2):
        #self.writeLineUTF('set mode ' + mode)
        self.modeSwitchWait(mode.upper(),timeout)
        
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
            self.modeSwitchWait('mdi')
            self.setMDILine('G21')
        elif units == 'inch':
            sendstr += 'inches'
            self.modeSwitchWait('mdi')
            self.setMDILine('G20')
        #self.writeLineUTF(sendstr)
        self.machine.units = 'mm'
        #FIXME Return state to orig before execution
        

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
            print('machine status is ' + self.machine.status)
            if self.machine.status == 'IDLE':
                return True
        return False

    def modeSwitchWait(self, mode, timeout = 2):
        start_time = time.time()
        mode = mode.upper()
        while (time.time() - start_time) < timeout:
            #self.setMachineMode(mode)
            self.writeLineUTF('set mode ' + mode)
            self.getMachineMode()
            print('in modeSwitchWait, current mode is ' + self.machine.mode)
            print('desired mode is ' + mode + ' and current mode is ' + self.machine.mode)
            if self.machine.mode == mode:
                print('modes matched')
                return True
        return False

    def checkMachineReady(self, timeout = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            print('in check machine ready')
            self.getProgramStatus()
            self.getMachineMode()
            if self.machine.status.upper() == 'IDLE' and self.machine.mode == 'AUTO' and not self.machine.rsh_error:
                print('machine ready')
                return True
            elif self.machine.mode != 'AUTO':
                print('fallthrough: mode was not AUTO')
                self.modeSwitchWait('auto')
        print('machine not ready')
        return False

    def readyMachine(self,timeout = 2):
        #self.setMachineMode('manual', timeout)
        self.modeSwitchWait('manual', timeout)
        #time.sleep(0.1)
        #self.setMachineMode('auto', timeout)
        self.modeSwitchWait('auto', timeout)
        #time.sleep(0.1)

    def resetPosition(self,X,Y,Z,A,B,timeout):
        self.modeSwitchWait('mdi',timeout)
        self.writeLineUTF('set mdi g0x'+str(X)+'y'+str(Y)+'z'+str(Z)+'a'+str(A)+'b'+str(B))
        return self.MDICommandWaitDone(timeout)

    def resetMachine(self, timeout = 2):
        #self.setEstop(0)
        self.setDrivePower(1)
    #def loadPoints(self, points_file, polylines, blocklength):
        #self.commanded_points = self.importAxesPoints(points_file, polylines, blocklength)
        #return self.commanded_points

    def commandPoints(self,polylines,blocklength,commands_to_send):
        X = self.importAxisPoints(Xpts,polylines,blocklength)
        Y = self.importAxisPoints(Ypts,polylines,blocklength)
        Z = self.importAxisPoints(Zpts,polylines,blocklength)
        A = self.importAxisPoints(Apts,polylines,blocklength)
        B = self.importAxisPoints(Bpts,polylines,blocklength)
        axisCoords = np.stack((X,Y,Z,30+np.zeros_like(A),1+np.zeros_like(B)),axis=2)
        
		#a = self.importAxesPoints(points_file,polylines,blocklength)
        #print(a)
        #(X, Y, Z, A, B) = self.importAxesPoints(points_file,polylines,blocklength)
        
        axisCoords = self.importAxesPoints(points_file,polylines,blocklength)
        
        #Store imported axis points in data repository
        self.data_store.imported_axes_points = axisCoords
        #self.data_store.flattened_imported_axes_points = self.data_store.imported_axes_points.reshape(commands_to_send*blocklength,axisCoords.shape[2])
        
        #axisCoords[:,:,2] = axisCoords[:,:,2] + 1
        
        #axisCoords = np.stack((X,Y,Z,A,B),axis=2)
		#axisCoords = np.stack((X,Y,Z,A,B),axis=2)
        #print(axisCoords)
        
        #axisCoords = self.loadPoints(points_file, polylines, blocklength)
        #print(axisCoords)
        #axisCoords[:,:,:3] = -0.5 * np.ones_like(axisCoords[:,:,:3])
        #axisCoords[:,:,3:] = np.zeros_like(axisCoords[:,:,3:])

        if self.machine.units == 'mm':
            scale_translational = 25.4
        else:
            scale_translational = 1.0
        scale_translational = 5.0/25.4
        scale_translational = 1
            
        self.resetMachine()
        retval = self.resetPosition(axisCoords[0][0][0],
                           axisCoords[0][0][1],
                           axisCoords[0][0][2],
                           axisCoords[0][0][3],axisCoords[0][0][4],20)
        if not retval:
            print("start position error, exiting...")
            return
        
        print(axisCoords[0][0][0]*scale_translational,axisCoords[0][0][1]*scale_translational,axisCoords[0][0][2]*scale_translational,axisCoords[0][0][3],axisCoords[0][0][4])

        #self.MDICommandWaitDone(5)
        #self.setManualMode()
        #self.modeSwitchWait('manual')
        #time.sleep(1)
        #self.setAutoMode()
        #self.modeSwitchWait('auto')
        #time.sleep(1)
        #time.sleep(1)
        self.readyMachine()
        if not self.checkMachineReady(2):
            #Ensure machine ready to control
            print('machine wasn''t ready in timeout period, aborting...')
            return
        
        self.setBinaryMode(1)

        if commands_to_send == -1:
            print("sending all 2")
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
        
        #Write out imported points and RSH feedback
        #flattened_points = self.data_store.imported_axes_points.reshape(commands_to_send*blocklength,axisCoords.shape[2])
        #np.savetxt("imported_points.csv",flattened_points,delimiter=",")
        #np.savetxt("buffer_level.csv",self.data_store.highres_tc_queue_length, delimiter=",")
        #self.resetPosition()

    def runNetworkPID(self,current_buffer_length,block_length,polylines,set_point_buffer_length,Kp=.01,Ki=0,Kd=0):
        #sleepTime = (blockLength*polyLines)/1000 - (Kp*((1000-self.rshQueueData[-1])))/1000
        sleep_time = max((block_length*polylines)/1000 - (Kp*((set_point_buffer_length-current_buffer_length)))/1000,0)
        return sleep_time

    def formatPoints(self,filename,blocklen):
        print('formatting points')
        pts = np.loadtxt(filename)/25.4;
        # Pad with zeros to make array length correct
        pts = np.pad(pts, [(0,(blocklen - pts.shape[0]) % blocklen),(0,0)], 'edge');
        coords = [np.reshape(pts[:,index],(-1,blocklen)) for index in np.arange(0,3)]
        coords.append(90 * np.ones_like(coords[0]))
        coords.append(np.zeros_like(coords[0]))
        return np.asarray(coords)

    def close(self):
        self.conn.shutdown(socket.SHUT_RDWR)
        self.conn.close()
        #sys.exit()
