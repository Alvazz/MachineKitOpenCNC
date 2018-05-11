import threading
import sys, os
import numpy as np
import struct
import time
import csv
import re
import string
import socket, select
import math
import queue
import signal
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
points_file = 'E:\\SculptPrint\\PocketNC\\Position Sampling\\Diva Head\\Longest Path Yet\\opt_code'
#points_file = 'E:\\SculptPrint\\PocketNC\\Position Sampling\\Diva Head\\Longest Path Yet\\consolidated'

class Move():
    def __init__(self,point_samples,move_type = None, filename = None):
        super(Move, self).__init__()
        self.serial_number = -1
        self.filename = ''
        self.point_samples = point_samples
        self.move_type = move_type

        ## To be populated when move is inserted into MotionController queue
        self.servo_tx_array = -1
        self.polylines = -1
        self.blocklength = -1

        #self.start_points = np.array([],dtype=float)
        self.start_points = point_samples[0, :]
        #self.end_points = np.array([],dtype=float)
        self.end_points = point_samples[-1, :]

class Command():
    #Class for RSH commands, mode switches, etc
    def __init__(self, control_function, control_function_parameters, ack_function, ack_function_parameters = None):
        super(Command, self).__init__()
        self.control_function = control_function
        self.control_function_parameters = control_function_parameters
        self.ack_function = ack_function
        if ack_function_parameters != None:
            self.ack_function_parameters = ack_function_parameters

class MotionController(threading.Thread):
    def __init__(self, machine_controller, machine):
        super(MotionController, self).__init__()
        self.machine = machine
        self.machine_controller = machine_controller
        self._running = False

        self.move_queue = []
        self.move_queue = queue.Queue()
        self.move_in_progress = 0
        self.last_move_serial_number = 0
        self.current_move_serial_number = 0

        ### Network control parameters
        self.polylines = self.machine.polylines_per_tx
        self.blocklength = self.machine.points_per_polyline

    def run(self):
        #while self._running:
        while True:
            #FIFO for point arrays. Initialize as length 0 with last_move_serial_number of -1
            ## FIXME use queue.Queue() here
            if self._running:
            #if len(self.move_queue) > self.last_move_serial_number and len(self.move_queue) > 0:
            #print('length of move queue is ' + str(len(self.move_queue)))
                print('length of move queue is ' + str(self.move_queue.qsize()))
                #Prepare for direct control
                #self.machine_controller.readyMachine()
                if not self.machine_controller.checkMachineReady():
                    # Ensure machine ready to control
                    self.machine_controller.readyMachine()
                    print('machine wasn''t ready in timeout period, aborting...')
                    return
                self.machine_controller.setBinaryMode(1)
                #current_queue_length = len(self.move_queue)
                #There are new moves in the queue, find the one to be executed next
                #self.point_buffer[self.last_move_serial_number + 1].serial_number = self.last_move_serial_number + 1
                ## FIXME make while loop?
                while not self.move_queue.empty() and not self.machine.rsh_error:
                #for move_queue_position in range(self.last_move_serial_number,current_queue_length):
                    move_to_execute = self.move_queue.get()
                    self.current_move_serial_number += 1
                    move_to_execute.serial_number = self.current_move_serial_number

                    #move_to_execute = self.move_queue[self.last_move_serial_number + 1 - 1]
                    print('executing move ' + str(move_to_execute.serial_number))
                    self.move_in_progress = move_to_execute
                    while not self.commandPoints(move_to_execute.servo_tx_array,self.polylines,self.blocklength,-1):
                        if self.machine.rsh_error:
                            break
                        else:
                            pass
                    self.last_move_serial_number = move_to_execute.serial_number
                    self.move_queue.task_done()
                    self.machine_controller.data_store.completed_moves.append(move_to_execute)
                    print('done. last_move_serial_number is now ' + str(self.last_move_serial_number))

                self.machine_controller.setBinaryMode(0)
                self._running = False

                if self.machine.rsh_error:
                    #Clear out move queue
                    print('dumping move queue')
                    self.move_queue = queue.Queue()

            else:
                #thread idle
                pass



    def commandPoints(self, servo_points, polylines, blocklength, commands_to_send = -1):
        # Store imported axis points in data repository
        self.machine_controller.data_store.current_servo_points = servo_points

        if commands_to_send == -1:
            print("sending all points")
            commands_to_send = int(servo_points.shape[0] / polylines)
            print(commands_to_send)

        #Form binary command string
        for command in range(0, commands_to_send):
            commanded_points = []
            frame = bytearray()
            frame.extend(self.machine_controller.convertInt2Bin(polylines))
            frame.extend(self.machine_controller.convertInt2Bin(blocklength))
            for polyline in range(0, polylines):
                # print(command)
                for axis in range(0, servo_points.shape[2]):
                    # if axis <= 2:
                    #     scale = scale_translational
                    # else:
                    #     scale = 1
                    for point in range(0, servo_points.shape[1]):
                        frame.extend(self.machine_controller.convertFloat2Bin(servo_points[(command * polylines) + polyline, point, axis]))
                        commanded_points.append(servo_points[(command * polylines) + polyline, point, axis])
                        #self.machine_controller.data_store
                        # print(axisCoords[(command*polylines)+polyLine,point,axis]*scale)

            frame.extend(struct.pack('!f', np.inf))
            if self.machine.rsh_error:
                print('commandPoints detected RSH error')
                return
            self.machine_controller.rsh_socket.send(frame)
            self.machine_controller.data_store.sent_servo_commands.append(commanded_points)

            ## FIXME store current position from feedback thread
            #self.machine_controller.machine.current_position = np.reshape(axis_coords[-1, -1, :], [1, self.machine.num_joints])[0]

            sleep_time = self.runNetworkPID(self.machine_controller.rsh_buffer_length, blocklength, polylines, self.machine.buffer_level_setpoint)
            # print(sleep_time, self.rsh_buffer_length)
            time.sleep(sleep_time)

        #self.machine_controller.setBinaryMode(0)
        return True

    def runNetworkPID(self, current_buffer_length, block_length, polylines, set_point_buffer_length, Kp=.0115, Ki=0,
                      Kd=0):
        # sleepTime = (blockLength*polyLines)/1000 - (Kp*((1000-self.rshQueueData[-1])))/1000
        if (self.machine.max_buffer_level - current_buffer_length) < 100:
            print('WARNING: Buffer finna overflow')
        #print(current_buffer_length)
        sleep_time = max((block_length * polylines) / 1000 - (Kp * ((set_point_buffer_length - current_buffer_length))) / 1000,0)
        return sleep_time

    def adaptNetworkTxGain(self):
        pass


class MachineController(threading.Thread):
    def __init__(self, socket, machine, data_store, encoder_interface):
        super(MachineController, self).__init__()
        #print('in machine controller init')
        self.rsh_socket = socket
        self.machine = machine
        self.encoder_interface = encoder_interface
        self.motion_controller = MotionController(self,machine)

        self._running = True
        self.received_data_string = ""
        self.data_store = data_store
        self.rsh_buffer_length = 0

        self.command_queue = queue.Queue()

    def run(self):
        self.motion_controller.start()
        while self._running:
            #if self.machine.logging_mode:
            ## FIXME move this to feedback thread
            #self.machine.current_position = self.data_store.stepgen_feedback_positions[-1,:] - self.machine.table_zero
            ## FIXME set up loop for running command queue
            ## FIXME update state machine every loop?
            pass

        # We are done running, clean up sockets
        self.rsh_socket.shutdown(socket.SHUT_RDWR)
        self.rsh_socket.close()

    ######################## Motion Controller Interface ########################
    def insertMove(self, move):
        #move.serial_number = len(self.motion_controller.move_queue) + 1
        # Populate parameters of the move
        move.polylines = self.motion_controller.polylines
        move.blocklength = self.motion_controller.blocklength
        move.servo_tx_array = self.formatPoints(move.point_samples, move.polylines, move.blocklength)
        ##FIXME force start position of move to end position of last move?
        #self.motion_controller.move_queue.append(move)
        self.motion_controller.move_queue.put(move)
        print('put move on queue')
        #print('the move queue is now: ')
        #print(self.motion_controller.move_queue)

    def testMachine(self,X,Y,Z,A,B):
        hold_points = self.generateHoldPositionPoints(1)
        #points = self.generateMove([X, Y, Z, A, B])
        #points = self.generateHoldPosition(5)
        self.insertMove(Move(hold_points,'hold'))
        rapid_to_zero = Move(self.generateMovePoints([0,0,0,0,0]),'trap')
        self.insertMove(rapid_to_zero)
        #self.insertMove(Move(self.generateMove([-1, -1, 0, 0, 0])))
        test_points = self.importAxesPoints(points_file + str('5'))
        #test_points[:,2] = test_points[:,2] + 1.5

        test_move = Move(test_points,'imported')
        rapid_to_start = Move(self.generateMovePoints(test_move.start_points.tolist(),rapid_to_zero.end_points),'trap')

        self.insertMove(Move(self.generateHoldPositionPoints(1,[0,0,0,0,0]),'hold'))
        self.insertMove(rapid_to_start)
        print('inserted rapid')
        self.motion_controller._running = True
        self.insertMove(Move(self.generateHoldPositionPoints(1,test_move.start_points.tolist()),'hold'))
        self.insertMove(test_move)
        for f in range(6,65+1):
            fname = points_file + str(f)
            imported_move = Move(self.importAxesPoints(fname),'imported',fname)
            self.insertMove(imported_move)
        #self.insertMove(Move(self.generateMove([-1.437980999999999954, 1.890032000000000156,-1.515102000000000171, 0.8762108906911221240,-2.578106385065804140])))

        #self.insertMove(Move(self.importAxesPoints(r'E:\SculptPrint\PocketNC\Position Sampling\Diva Head\opt_code',1,25)))

        # #for i in range(0,10):
        # self.insertMove(Move(self.generateMove([1,1,-1,50,50],[-1, -1, 0, 0, 0])))
        # self.insertMove(Move(self.generateMove([-1, -1, 0, 0, 0],[1,1,-1,50,50])))

        #linear_move_1 = Move(points)
        #self.insertMove(linear_move_1)

    def visualizePoints(self):
        points = np.empty((0,5), float)
        for move in self.motion_controller.move_queue:
            points = np.vstack((points,move.point_samples))
        plt.plot(points)
        plt.show()


    ######################## Data Handling ########################
    def convertFloat2Bin(self, num):
        return struct.pack('!f', num)

    def convertInt2Bin(self, num):
        return struct.pack('!i',num)

    def formatBinaryLine(self,axisCoords,polyLines,blockLength,positionInFile):
        return #position in file#

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

    def importAxesPoints(self, file):
        ##FIXME check for overtravel
        points = np.array(list(csv.reader(open(file, "rt"), delimiter=" "))).astype("float")[:,:self.machine.num_joints]
        #points = np.array(list(csv.reader(open(file, "rt"), delimiter=" "))).astype("float")
        #HACK HACK
        #points[:,2] = points[:,2]+1.5
        return points

        # axis_coords = []
        # for axis in range(points.shape[1]):
        #     axis_coords.append(self.padAndFormatAxisPoints(np.asarray([points[:, axis]]).T, polylines, blocklength))
        # return np.asarray(axis_coords).transpose(1, 2, 0)

    def formatPoints(self, points, polylines, block_length):
        axis_coords = []
        #FIXME fix if not divisible by polylines*blocklength
        for axis in range(points.shape[1]):
            axis_coords.append(self.padAndFormatAxisPoints(np.asarray([points[:, axis]]).T, polylines, block_length))
        return np.asarray(axis_coords).transpose(1, 2, 0)

    ######################## Writing Functions ########################
    def writeLineUTF(self,data):
        self.rsh_socket.send((data+'\r').encode('utf-8'))
        time.sleep(0.25)

    def writeLineBin(self,frame):
        frame.extend(struct.pack('!f',np.inf))
        self.rsh_socket.send(frame)

    ######################## API for Common RSH Commands ########################
    ##FIXME implement heartbeat!
    def connectAndLink(self):
        #self.waitForSet(self.login,None,self.getLoginStatus)
        if not self.login()[0]:
            print('No response from RSH, assume crash')
            ##FIXME ssh to machine and restart process - major bandaid
            return False
        else:
            self.waitForSet(self.setEcho, 0, self.getEcho)
            ##FIXME sync clocks here
            self.readyMachine()

    # def login(self):
    #     self.machine.connection_change_event.clear()
    #     self.machine.link_change_event.clear()
    #     self.writeLineUTF('hello EMC robie 1')
    #     self.writeLineUTF('set enable EMCTOO')
    #
    #     #self.modeSwitchWait('auto')
    #     #self.setMachineUnits('inch')

    def login(self, timeout = 0.5):
        self.machine.connection_change_event.clear()
        self.machine.link_change_event.clear()
        self.writeLineUTF('hello EMC robie 1')
        self.writeLineUTF('set enable EMCTOO')
        if self.machine.connection_change_event.wait(timeout) and self.machine.link_change_event.wait(timeout):
            return (True, self.machine.connected, self.machine.linked)
        else:
            return (False, self.machine.connected, self.machine.linked)

    def readyMachine(self):
        self.waitForSet(self.setEstop, 0, self.getEstop)
        self.waitForSet(self.setDrivePower, 1, self.getDrivePower)
        #self.waitForSet(self.setMachineMode, 'manual', self.getMachineMode)
        self.waitForSet(self.setMachineMode, 'auto', self.getMachineMode)
        self.waitForSet(self.setLogging, 1, self.getLogging)

    def homeMachine(self,X=1,Y=1,Z=1,A=1,B=1,C=0):
        #FIXME home each axis individually and wait for all to complete
        self.machine.pushState()
        self.modeSwitchWait('manual')

        for i in range(0,self.machine.num_joints):
            send_str = 'set home'

        if X:
            send_str += ' 1'
        if Y:
            send_str += ' 2'
        if Z:
            send_str += ' 3'
        if A:
            send_str += ' 4'
        if B:
            send_str += ' 5'

        self.writeLineUTF(send_str)

        #Wait for completion for 10 sec
        if not self.commandWaitDone(10):
            print('homing error')
            retval = False
        else:
            self.machine.homed = [X, Y, Z, A, B]
            retval = True

        self.modeSwitchWait(self.machine.popState())
        return retval

    def getEcho(self, timeout = 0.5):
        self.writeLineUTF('get echo')
        if self.machine.echo_change_event.wait(timeout):
            return (True, self.machine.echo)
        else:
            return (False, self.machine.echo)

    def getEstop(self, timeout = 0.5):
        self.writeLineUTF('get estop')
        if self.machine.estop_change_event.wait(timeout):
            return (True, self.machine.estop)
        else:
            return (False, self.machine.estop)

    def getProgramStatus(self, timeout = 0.5):
        self.machine.status_change_event.clear()
        self.writeLineUTF('get program_status')
        if self.machine.status_change_event.wait(timeout):
            return (True, self.machine.status)
        else:
            return (False, self.machine.status)
        return

    def getMachineMode(self, timeout = 0.5):
        self.writeLineUTF('get mode')
        if self.machine.mode_change_event.wait(timeout):
            return (True, self.machine.mode)
        else:
            return (False, self.machine.mode)
        
    def getDrivePower(self, timeout = 0.5):
        self.writeLineUTF('get machine')
        if self.machine.drive_power_change_event.wait(timeout):
            return (True, self.machine.drive_power)
        else:
            return (False, self.machine.drive_power)

    def getLogging(self, timeout = 0.5):
        #FIXME write this to get servo logging parameters
        self.writeLineUTF('get servo_log_params')
        if self.machine.logging_mode_change_event.wait(timeout):
            return (True, self.machine.logging_mode)
        else:
            return (False, self.machine.logging_mode)

    def setBinaryMode(self, flag):
        ## FIXME confirm receipt
        if flag:
            print('setting binary mode on')
            self.writeLineUTF('set comm_mode binary')
            self.machine.binary_mode = 1
        else:
            print('setting binary mode off')
            self.rsh_socket.send(struct.pack('!f',-np.inf))
            self.machine.binary_mode = 0
            time.sleep(0.05)

    def setMachineMode(self, mode):
        self.machine.mode_change_event.clear()
        #print('cleared mode_change_event flag')
        self.writeLineUTF('set mode ' + mode.upper())
        #self.modeSwitchWait(mode.upper(), timeout)
        
    def setMDILine(self,line):
        self.writeLineUTF('set mdi ' + line)

    def setEcho(self,flag):
        sendstr = 'set echo '
        if flag:
            sendstr += 'on'
        else:
            sendstr += 'off'
        self.writeLineUTF(sendstr)
        self.machine.echo_change_event.clear()

    def setDrivePower(self,flag):
        sendstr = 'set machine '
        if flag:
            sendstr += 'on'
        else:
            sendstr += 'off'
        self.machine.drive_power_change_event.clear()
        self.writeLineUTF(sendstr)

    def setEstop(self, flag):
        sendstr = 'set estop '
        if flag:
            sendstr += 'on'
        else:
            sendstr += 'off'
        self.machine.estop_change_event.clear()
        self.writeLineUTF(sendstr)

    def setLogging(self, flag, sub_sample_rate=0, buffer_size=0, axes=0, dump_flag=0, write_buffer=0):
        ## FIXME confirm logging actually set up before changing state machine
        ## FIXME sync clocks here
        if not sub_sample_rate:
            sub_sample_rate = self.machine.servo_log_sub_sample_rate
        if not buffer_size:
            buffer_size = self.machine.servo_log_buffer_size
        if not axes:
            axes = self.machine.servo_log_num_axes

        # Need to toggle logging mode
        if self.machine.binary_mode:
            self.machine.logging_mode_change_event.clear()
            print('disabling binary')
            self.setBinaryMode(0)
            self.setMachineMode('auto')
            self.writeLineUTF('set servo_log_params ' + str(flag) + ' ' + str(axes) +
                              ' ' + str(sub_sample_rate) + ' ' + str(buffer_size) +
                              ' ' + str(dump_flag) + ' ' + str(write_buffer))
            self.setBinaryMode(1)
            #self.machine.logging_mode = not(self.machine.logging_mode)
        else:
            self.machine.logging_mode_change_event.clear()
            #self.setMachineMode('auto')
            self.waitForSet(self.setMachineMode,'auto',self.getMachineMode)
            self.writeLineUTF('set servo_log_params ' + str(flag) + ' ' + str(axes) +
                              ' ' + str(sub_sample_rate) + ' ' + str(buffer_size) +
                              ' ' + str(dump_flag) + ' ' + str(write_buffer))
            #self.machine.logging_mode = not (self.machine.logging_mode)
        #print('logging mode ' + str(int(self.machine.logging_mode)))

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


    # def setHome(self, fX, fY, fZ, fA, fB):
    #     self.saveState()
    #     self.setManualMode()
    #     sendstr = 'set home '
    #     if fX:
    #         sendstr = sendstr + '0 '
    #     if fY:
    #         sendstr = sendstr + '1 '
    #     if fZ:
    #         sendstr = sendstr + '2 '
    #     if fA:
    #         sendstr = sendstr + '3 '
    #     if fB:
    #         sendstr = sendstr + '4 '
    #     self.writeLineUTF(sendstr)

    ######################## Synchronization ########################
    def synchronizeClocks(self):
        pass

    def waitForSet(self, set_function, set_params, get_function, timeout = 2):
        # FIXME something here like commandWaitDone that will take a function handle as an argument
        set_function(set_params)
        while not get_function()[0]:
            print('waiting for ' + str(get_function))
            pass
        ## FIXME call set again after some time?
        print('success: ' + str(get_function) + ' returned True')
        ## FIXME set up timeout here
        #print('logging mode is ' + str(self.machine.logging_mode))

    def commandWaitDone(self, timeout=2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            self.getProgramStatus()
            print('machine status is ' + self.machine.status)
            if self.machine.status == 'IDLE':
                return True
        return False

    def modeSwitchWait(self, mode, timeout=2):
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

    def checkMachineReady(self, timeout = 0.5):
        if self.machine.isAutoMode() and self.getProgramStatus()[0] and self.machine.status == 'IDLE':
            return True
        else:
            print('Machine not ready: isAutoMode returned ' + str(self.machine.isAutoMode()) + ' and machine status is ' + self.machine.status)
            return False

    # def checkMachineReady(self, timeout=2):
    #     start_time = time.time()
    #     while (time.time() - start_time) < timeout:
    #         print('in check machine ready')
    #         self.getProgramStatus()
    #         self.getMachineMode()
    #         if self.machine.status.upper() == 'IDLE' and self.machine.mode == 'AUTO' and not self.machine.rsh_error:
    #             print('machine ready')
    #             return True
    #         elif self.machine.mode != 'AUTO':
    #             print('fallthrough: mode was not AUTO')
    #             self.modeSwitchWait('auto')
    #     print('machine not ready')
    #     return False

    # def readyMachine(self,timeout = 2):
    #     #self.setMachineMode('manual', timeout)
    #     self.modeSwitchWait('manual', timeout)
    #     #time.sleep(0.1)
    #     #self.setMachineMode('auto', timeout)
    #     self.modeSwitchWait('auto', timeout)
    #     #time.sleep(0.1)

    # def resetPosition(self,X,Y,Z,A,B,timeout):
    #     self.modeSwitchWait('mdi',timeout)
    #     self.writeLineUTF('set mdi g0x'+str(X)+'y'+str(Y)+'z'+str(Z)+'a'+str(A)+'b'+str(B))
    #     return self.MDICommandWaitDone(timeout)

    def resetMachine(self, timeout = 2):
        #self.setEstop(0)
        self.setDrivePower(1)

    ######################## Trajectory Generation ########################
    ## FIXME only allow movement generation when logging is enabled?

    def generateHoldPositionPoints(self, hold_time=1,position = [np.nan]):
        if any(np.isnan(position)):
            position_to_hold = self.machine.current_position
        else:
            position_to_hold = np.array(position)

        joint_position_samples = np.zeros([int(np.clip(np.round(hold_time/self.machine.servo_dt),1,np.inf)), int(self.machine.num_joints)]) + position_to_hold
        return joint_position_samples

    def generateMovePoints(self, end_points, start_points = [np.nan], move_velocity = 0, max_joint_velocities = -1, max_joint_accelerations = -1, move_type = 'trapezoidal'):
        fallthrough_points = self.generateHoldPositionPoints(0,start_points)
        #FIXME check for out of limit move generation
        if any(end_points[k] < self.machine.limits[k][0] for k in range(0,len(end_points))) or any(end_points[k] > self.machine.limits[k][1] for k in range(0,len(end_points))):
            print('move exceeds bounds')
            return fallthrough_points

        #Handle arguments
        if any(np.isnan(start_points)):
            start_points = self.machine.current_position
        if max_joint_velocities == -1:
            max_joint_velocities = self.machine.max_joint_velocity
        if max_joint_accelerations == -1:
            max_joint_accelerations = self.machine.max_joint_acceleration
        print('starting move from ')
        print(start_points)
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
            if not np.count_nonzero(move_vector):
                # FIXME return 0 if move_vector == 0
                #Null move
                print('move is null')
                return fallthrough_points

            move_direction = np.sign(move_vector)
            #Check if we can reach cruise for each joint
            max_move_velocity = np.zeros([1,self.machine.num_joints])[0]
            for joint in range(0,self.machine.num_joints):
                no_cruise_time_to_center = math.sqrt(
                    math.fabs(move_vector[joint]) / max_joint_accelerations[joint])
                if move_vector[joint] == 0:
                    #Do not move this joint
                    print('joint ' + str(joint) + ' holding position')
                elif (max_joint_accelerations[joint] * no_cruise_time_to_center) >= max_joint_velocities[joint]:
                    #We can reach cruise phase
                    max_move_velocity[joint] = max_joint_velocities[joint]
                else:
                    #Cannot reach cruise phase, move is triangular
                    max_move_velocity[joint] = max_joint_accelerations[joint] * no_cruise_time_to_center

            print('move direction')
            print(move_direction)

            #Calculate move profile for each joint
            time_points = np.zeros([3, self.machine.num_joints])
            for joint in range(0, self.machine.num_joints):
                if max_move_velocity[joint] < max_joint_velocities[joint]:
                    #Move too short for cruise phase, acceleration and deceleration intersect at t1=t2
                    time_points[0][joint] = max_move_velocity[joint] / max_joint_accelerations[joint]
                    time_points[1][joint] = time_points[0][joint]
                    time_points[2][joint] = 2*time_points[0][joint]
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
            slowest_joint = np.where(time_points[-1,:] == np.max(time_points[-1,:]))[0][0]
            print('slowest joint is ' + str(slowest_joint))
            move_time = time_points[2][slowest_joint]
            print('move time is ' + str(move_time))

            motion_scale_factors = (np.asarray(time_points[2][:])/move_time)
            # Scale waypoint times for slowest axis
            time_points = np.multiply(np.asarray(time_points), np.divide(1,motion_scale_factors,out = np.zeros_like(motion_scale_factors),where = motion_scale_factors != 0)).tolist()

            #Sample time points to plan trajectories
            servo_times = np.arange(0,math.ceil(move_time/self.machine.servo_dt)*self.machine.servo_dt,self.machine.servo_dt)
            joint_position_samples = np.zeros([np.size(servo_times),self.machine.num_joints])
            #Force initial position
            joint_position_samples[0][:] = start_points

            #State machine for integration
            last_joint_positions = np.zeros([3,self.machine.num_joints]).tolist()
            phase_switch_times = np.zeros([3,self.machine.num_joints]).tolist()

            max_move_velocity = np.multiply(motion_scale_factors,max_move_velocity)
            max_joint_accelerations = np.multiply(np.power(motion_scale_factors,2), max_joint_accelerations)

            for ndx in range(0,np.size(servo_times)):
                t = servo_times[ndx]
                for joint in range(0, self.machine.num_joints):
                    #Phase -1: Joint hold position
                    if time_points[0][joint] == 0 and time_points[1][joint] == 0 and time_points[2][joint] == 0:
                        joint_position_samples[ndx][joint] = start_points[joint]
                    #Phase 1: Acceleration
                    elif t < time_points[0][joint]:
                        #We are in phase one (acceleration) for this joint
                        joint_position_samples[ndx][joint] = start_points[joint] + 0.5*move_direction[joint]*max_joint_accelerations[joint]*np.power(t,2)
                        last_joint_positions[0][joint] = joint_position_samples[ndx][joint]
                        phase_switch_times[0][joint] = t
                    #Phase 2: either cruise or deceleration
                    elif t >= time_points[0][joint] and (t < time_points[1][joint] or time_points[0][joint] == time_points[1][joint]):
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
            # plt.plot(servo_times,joint_position_samples[:,0])
            # plt.plot(servo_times, joint_position_samples[:, 1])
            # plt.plot(servo_times, joint_position_samples[:, 2])
            # plt.plot(servo_times, joint_position_samples[:, 3])
            # plt.plot(servo_times, joint_position_samples[:, 4])
            # plt.show()

            return joint_position_samples


    def close(self):
        print('in machine controller close')
        self._running = False
        #self.exit()
        #sys.exit()
