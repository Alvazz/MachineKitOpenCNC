import threading
import sys
import numpy as np
import struct
import time
import csv
import re
import string
import socket, select
import math
import matplotlib.pyplot as plt

Xpts = r'E:\SculptPrint\PocketNC\Position Sampling\Xpts_opt.csv'
Ypts = r'E:\SculptPrint\PocketNC\Position Sampling\Ypts_opt.csv'
Zpts = r'E:\SculptPrint\PocketNC\Position Sampling\Zpts_opt.csv'
Apts = r'E:\SculptPrint\PocketNC\Position Sampling\Apts_opt.csv'
Bpts = r'E:\SculptPrint\PocketNC\Position Sampling\Bpts_opt.csv'
#points_file = 'E:\SculptPrint\PocketNC\Position Sampling\Diva Head\opt_code'
points_file = 'E:\SculptPrint\PocketNC\SP Integration\Example\opt_code2.75'
points_file = 'E:\SculptPrint\PocketNC\Standards\opt_code'
points_file = 'E:\\SculptPrint\\PocketNC\\Position Sampling\\5axTrapezoid'
points_file = 'E:\\SculptPrint\\PocketNC\\Position Sampling\\opt_code_patchup'


class MotionController(threading.Thread):
    def __init__(self, machine_controller):
        super(MotionController, self).__init__()
        self.machine_controller = machine_controller

    def run(self):
        self.machine_controller.commandPoints(1, 25, -1)


class MachineController(threading.Thread):
    def __init__(self, conn, machine, data_store):
        super(MachineController, self).__init__()
        print('in machine controller init')
        self.conn = conn
        self._running = True

        self.received_data_string = ""
        self.data_store = data_store
        self.rsh_buffer_length = 0
        self.machine = machine
        
    def run(self):
        while self._running:
            # Process feedback from EMCRSH
            data_available = select.select([self.conn], [], [], 0.5)
            if data_available[0]:
                string_received = self.conn.recv(4096).decode("utf-8")
                complete_data_string = ''
                if '\n' in string_received:
                    split_received_string = string_received.split('\n')
                    complete_data_string = self.received_data_string + split_received_string[0]
                    print('complete data string ' + complete_data_string + '\n')
                    self.received_data_string = ''.join(split_received_string[1:])
                else:
                    self.received_data_string += string_received
                # print(self.received_data_string)
                if any(s in complete_data_string for s in self.machine.rsh_feedback_strings):
                    # Check for error first
                    if self.machine.rsh_feedback_strings[-1] in string_received:
                        # Error in RSH command
                        print('RSH error')
                        self.handleRSHError()
                    if self.machine.rsh_feedback_strings[0] in string_received:
                        # Buffer Length
                        self.processRSHFeedback(complete_data_string)
                    elif self.machine.rsh_feedback_strings[1] in string_received:
                        # Program Status
                        print('program status')
                        self.machine.status = complete_data_string.split(' ')[1].strip()
                        print('set program status to ' + self.machine.status)
                    elif self.machine.rsh_feedback_strings[2] in string_received:
                        # Machine Mode
                        print('mode set')
                        self.machine.mode = complete_data_string.split(' ')[1].strip()
                        print('set machine mode to ' + self.machine.mode)

        # We are done running, clean up sockets
        self.conn.shutdown(socket.SHUT_RDWR)
        self.conn.close()

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

    def setMachineMode(self, mode, timeout = 2):
        # self.writeLineUTF('set mode ' + mode)
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
        if flag != self.loggingMode:
            # Need to toggle logging mode
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

    def padAndFormatAxisPoints(self, points, polylines, blocklength):
        # print(points.shape, blocklength, np.size(points, 0), blocklength-(np.size(points,0)%blocklength))
        pad_points = np.lib.pad(points, ((0, blocklength - (np.size(points, 0) % blocklength)), (0, 0)), 'constant',
                                constant_values=points[-1])
        shape_points = pad_points.reshape((-1, blocklength), order='C')
        # print(shape_points.shape)
        return np.pad(shape_points, ((0, polylines - (np.size(shape_points, 0) % polylines)), (0, 0)), 'constant',
                      constant_values=shape_points[-1, -1])

    def importAxisPoints(self, file, polylines, blocklength):
        points = np.array(list(csv.reader(open(file, "rt"), delimiter=","))).astype("float")
        return self.padAndFormatAxisPoints(points, polylines, blocklength)

    def importAxesPoints(self, file, polylines, blocklength):
        points = np.array(list(csv.reader(open(file, "rt"), delimiter=" "))).astype("float")
        axis_coords = []
        for axis in range(points.shape[1]):
            axis_coords.append(self.padAndFormatAxisPoints(np.asarray([points[:, axis]]).T, polylines, blocklength))
        return np.asarray(axis_coords).transpose(1, 2, 0)

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
        
        #axisCoords = self.importAxesPoints(points_file,polylines,blocklength)
        axisCoords = self.generateMove('trapezoidal',[0,0,0,0,0],[1,1,1,1,1],10,self.machine.max_joint_velocity,self.machine.max_joint_acceleration)
        
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

    ## Trajectory Generation
    def generateMove(self, move_type, start_points, end_points, move_velocity, max_joint_velocities, max_joint_accelerations):
        #FIXME add capability to scale move velocity
        #Trapezoidal velocity profile generation
        #USAGE:
        #   move_type: string
        #   start_points: 1 x num_axes list
        #   end_points: 1 x num_axes list
        #   move_velocity: float
        #   max_joint_velocities: 1 x num_axes list
        #   max_joint_accelerations: 1 x num_axes list

        if move_type == 'trapezoidal':
            move_vector = (np.asarray(end_points) - np.asarray(start_points))
            move_direction = np.sign(move_vector)
            #Check if we can reach cruise for each joint
            #move_direction = np.zeros([1, self.machine.num_joints]).tolist()[0]
            max_move_velocity = np.zeros([1,self.machine.num_joints])[0]
            #move_time = np.zeros([1,self.machine.num_joints]).tolist()[0]
            for joint in range(0,self.machine.num_joints):
                #move_direction[joint] = np.sign(end_points[joint] - start_points[joint])
                no_cruise_time_to_center = math.sqrt(math.fabs(end_points[joint]-start_points[joint])/max_joint_accelerations[joint])
                if (max_joint_accelerations[joint] * no_cruise_time_to_center) >= max_joint_velocities[joint]:
                    #We can reach cruise phase
                    max_move_velocity[joint] = max_joint_velocities[joint]
                else:
                    #Cannot reach cruise phase, move is triangular
                    max_move_velocity[joint] = max_joint_accelerations[joint] * no_cruise_time_to_center

            print('move direction')
            print(move_direction)

            #Calculate move profile for each joint
            time_points = np.zeros([3, self.machine.num_joints])
            #dist_points = np.zeros([3, self.machine.num_joints])
            for joint in range(0, self.machine.num_joints):
                if max_move_velocity[joint] < max_joint_velocities[joint]:
                    #Move too short for cruise phase, acceleration and deceleration intersect at t1=t2
                    time_points[0][joint] = max_move_velocity[joint] / max_joint_accelerations[joint]
                    time_points[1][joint] = time_points[0][joint]
                    time_points[2][joint] = 2*time_points[0][joint]
                    #dist_points[0][joint] = start_points[joint] + move_direction[joint]*(0.5*max_joint_accelerations[joint]*np.power(time_points[0][joint],2))
                    #dist_points[1][joint] = dist_points[0][joint]
                else:
                    #Time and distance to cruise velocity, t1
                    time_points[0][joint] = max_move_velocity[joint] / max_joint_accelerations[joint]
                    print('joint ' + str(joint) + ' can reach cruise')
                    #End of move
                    time_points[2][joint] = (np.fabs(move_vector[joint]) + (max_move_velocity[joint]*time_points[0][joint]))/max_move_velocity[joint]
                    #Start of deceleration phase
                    time_points[1][joint] = time_points[2][joint]-time_points[0][joint]
                    #sanity check
                    #dist_points[2][joint] = dist_points[1][joint] + dist_points[0][joint]

            #Find limiting joint
            slowest_joint = np.where(time_points[:,-1] == np.max(time_points[:,-1]))[0][0]
            print('slowest joint is ' + str(slowest_joint))
            move_time = time_points[2][slowest_joint]
            motion_scale_factors = (np.asarray(time_points[2][:])/move_time)

            #Sample time points to plan trajectories
            servo_times = np.arange(0,math.ceil(move_time/self.machine.servo_dt)*self.machine.servo_dt,self.machine.servo_dt)
            joint_position_samples = np.zeros([np.size(servo_times),self.machine.num_joints])
            #Force initial position
            print(start_points)
            print(np.size(joint_position_samples))
            print(np.size(servo_times))
            joint_position_samples[0][:] = start_points

            #State machine for integration
            last_joint_positions = np.zeros([3,self.machine.num_joints]).tolist()
            phase_switch_times = np.zeros([3,self.machine.num_joints]).tolist()

            max_move_velocity = np.multiply(motion_scale_factors,max_move_velocity)
            max_joint_accelerations = np.multiply(np.power(motion_scale_factors,2), max_joint_accelerations)
            time_points = np.multiply(np.asarray(time_points),1/np.asarray(motion_scale_factors)).tolist()
            for ndx in range(0,np.size(servo_times)):
                t = servo_times[ndx]
                for joint in range(0, self.machine.num_joints):
                    #last_joint_position = joint_position_samples[np.clip(ndx-1,0,np.size(servo_times))][joint]
                    #Phase 1: Acceleration
                    if t < time_points[0][joint]:
                        #We are in phase one (acceleration) for this joint
                        joint_position_samples[ndx][joint] = start_points[joint] + 0.5*move_direction[joint]*max_joint_accelerations[joint]*np.power(t,2)
                        last_joint_positions[0][joint] = joint_position_samples[ndx][joint]
                        phase_switch_times[0][joint] = t
                    #Phase 2: either cruise or deceleration
                    elif t >= time_points[0][joint] and (t < time_points[1][joint] or time_points[0][joint] == time_points[1][joint]):
                        #print('cruising joint ' + str(joint))
                        if time_points[0][joint] != time_points[1][joint]:
                            #We are in cruise phase
                            joint_position_samples[ndx][joint] = last_joint_positions[0][joint] + move_direction[joint]*(t-phase_switch_times[0][joint])*max_move_velocity[joint]
                            last_joint_positions[1][joint] = joint_position_samples[ndx][joint]
                            phase_switch_times[1][joint] = t
                        else:
                            #Move too short for cruise, begin deceleration
                            joint_position_samples[ndx][joint] = last_joint_positions[0][joint] + move_direction[joint]*\
                                                                 ((t-phase_switch_times[0][joint])*max_move_velocity[joint] - 0.5*max_joint_accelerations[joint]*np.power(t-phase_switch_times[0][joint],2))
                            last_joint_positions[1][joint] = joint_position_samples[ndx][joint]
                            phase_switch_times[1][joint] = t
                            last_joint_positions[1][joint] = joint_position_samples[ndx][joint]
                            phase_switch_times[1][joint] = t
                    #Phase 3: Deceleration
                    elif t >= time_points[1][joint]:
                        #print('decelerating joint ' + str(joint))
                        joint_position_samples[ndx][joint] = last_joint_positions[1][joint] + move_direction[joint]*\
                                                             ((t-phase_switch_times[1][joint])*max_move_velocity[joint] -
                                                              0.5 * max_joint_accelerations[joint] * np.power(t-phase_switch_times[1][joint],2))
                        last_joint_positions[2][joint] = joint_position_samples[ndx][joint]
                        phase_switch_times[2][joint] = t
            #Force last position
            joint_position_samples[-1,:] = end_points

            #Done
            plt.plot(servo_times,joint_position_samples[:,0])
            plt.plot(servo_times, joint_position_samples[:, 1])
            plt.plot(servo_times, joint_position_samples[:, 2])
            plt.plot(servo_times, joint_position_samples[:, 3])
            #plt.plot(servo_times, joint_position_samples[:, 4])
            plt.show()
            return joint_position_samples



    def close(self):
        print('in machine controller close')
        self._running = False
        #self.exit()
        #sys.exit()
