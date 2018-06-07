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
import paramiko
print('here machine control')
#import matplotlib.pyplot as plt
print('here machine control')

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

class SystemController(threading.Thread):
    def __init__(self, machine):
        super(SystemController, self).__init__()
        #FIXME set up SSH session here
        self.machine = machine
        self._running_thread = True
        self._run_ssh_connection = False

        self.ssh_client = paramiko.SSHClient()

    def run(self):
        while self._running_thread:
            if self._run_ssh_connection:
                while True:
                    pass
        self.ssh_client.close()

class MotionController(threading.Thread):
    def __init__(self, machine_controller, machine):
        super(MotionController, self).__init__()
        self.machine = machine
        self.machine_controller = machine_controller

        self._running_motion = False
        self._running_thread = True
        self._shutdown = False

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
        while self._running_thread:
            if self._running_motion:
                print('length of move queue is ' + str(self.move_queue.qsize()))
                #Prepare for direct control
                #self.machine_controller.readyMachine()
                # if not True: #self.machine_controller.checkMachineReady():
                #     print('MOTION CONTROLLER: Abort, machine not ready')
                #     return
                #     # Ensure machine ready to control
                #     self.machine_controller.readyMachine()
                #     print('machine wasn''t ready in timeout period, aborting...')
                #     return
                self.machine_controller.waitForSet(self.machine_controller.setCommMode,1,self.machine_controller.getCommMode)

                #There are new moves in the queue, find the one to be executed next
                while not self.move_queue.empty() and not self.machine.rsh_error:
                #for move_queue_position in range(self.last_move_serial_number,current_queue_length):
                    move_to_execute = self.move_queue.get()
                    self.current_move_serial_number += 1
                    move_to_execute.serial_number = self.current_move_serial_number

                    #move_to_execute = self.move_queue[self.last_move_serial_number + 1 - 1]
                    print('executing move ' + str(move_to_execute.serial_number))
                    self.move_in_progress = move_to_execute
                    while not self.commandPoints(move_to_execute.servo_tx_array,self.polylines,self.blocklength):
                        if self.machine.rsh_error or not self._running_motion or not self._running_thread:
                            print('MOTION CONTROLLER: Exiting motion execution')
                            break
                        else:
                            pass
                    self.last_move_serial_number = move_to_execute.serial_number
                    self.move_queue.task_done()
                    #FIXME use data_store_manager thread
                    self.machine_controller.data_store.completed_moves.append(move_to_execute)
                    print('done. last_move_serial_number is now ' + str(self.last_move_serial_number))

                #self.machine_controller.setBinaryMode(0)
                self._running_motion = False

                if self.machine.rsh_error:
                    #Clear out move queue
                    print('dumping move queue')
                    self.move_queue = queue.Queue()

            else:
                #thread idle, not running motion
                #print('motion controller thread idling')
                pass

        #Thread signaled to shutdown
        print('MOTION CONTROLLER: thread shut down')
        self._shutdown = True

    def commandPoints(self, servo_points, polylines, blocklength, commands_to_send = -1):
        # Store imported axis points in data repository
        #FIXME use DB manager
        self.machine_controller.data_store.current_servo_points = servo_points

        if commands_to_send == -1:
            print("sending all points")
            commands_to_send = int(servo_points.shape[0] / polylines)
            print(commands_to_send)

        #Form binary command string
        for command in range(0, commands_to_send):
            commanded_points = []
            #binary_command = bytearray()#.extend(struct.pack('!3sii','DM '.encode('utf-8'),polylines,blocklength))
            #binary_command.extend(struct.pack('!' + str(len(self.machine.binary_direct_mode_header)) + 'sii', self.machine.binary_direct_mode_header.encode('utf-8'), polylines, blocklength))
            binary_command = (struct.pack('!' + str(len(self.machine.binary_direct_mode_header)) + 'sii',
                                              self.machine.binary_direct_mode_header.encode('utf-8'), polylines,
                                              blocklength))
            # binary_command.extend('DM '.encode('utf-8'))
            # binary_command.extend(self.machine_controller.convertInt2Bin(polylines))
            # binary_command.extend(self.machine_controller.convertInt2Bin(blocklength))
            for polyline in range(0, polylines):
                # print(command)
                for axis in range(0, servo_points.shape[2]):
                    for point in range(0, servo_points.shape[1]):
                        #binary_command.extend(self.machine_controller.convertFloat2Bin(servo_points[(command * polylines) + polyline, point, axis]))
                        binary_command += self.machine_controller.convertFloat2Bin(servo_points[(command * polylines) + polyline, point, axis])
                        commanded_points.append(servo_points[(command * polylines) + polyline, point, axis])
                        #self.machine_controller.data_store
                        # print(axisCoords[(command*polylines)+polyLine,point,axis]*scale)

            #frame.extend(struct.pack('!f', np.inf))
            #binary_command.extend(b'\x7f')
            binary_command += self.machine.binary_line_terminator
            if self.machine.rsh_error:
                print('commandPoints detected RSH error after ' + str(command) + ' commands')
                return

            self.machine.buffer_level_reception_event.clear()
            #FIXME do this better insertion into data_store
            self.machine_controller.data_store.sent_servo_commands.append(commanded_points)
            self.machine_controller.rsh_socket.send(binary_command)
            #print('length is: ' + str(len(binary_command)))
            #print(str(binary_command))
            #print('commanded points are: ' + str(commanded_points))
            #FIXME check buffer was flushed

            ## FIXME store current position from feedback thread
            #self.machine_controller.machine.current_position = np.reshape(axis_coords[-1, -1, :], [1, self.machine.number_of_joints])[0]

            sleep_time = self.runNetworkPID(self.machine.rsh_buffer_level, blocklength, polylines, self.machine.buffer_level_setpoint)
            # print(sleep_time, self.rsh_buffer_level)
            #print(self.machine.rsh_buffer_level)
            #print(sleep_time)
            time.sleep(sleep_time)

        #self.machine_controller.setBinaryMode(0)
        return True

    def runNetworkPID(self, current_buffer_length, block_length, polylines, set_point_buffer_length, Kp=.03, Ki=0,
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
    def __init__(self, machine, data_store):
        super(MachineController, self).__init__()
        #print('in machine controller init')
        self.machine = machine
        self.rsh_socket = self.machine.rsh_socket
        #self.encoder_interface = encoder_interface
        self.motion_controller = MotionController(self,machine)

        self._running_thread = True
        self._shutdown = False
        self.received_data_string = ""
        self.data_store = data_store
        self.rsh_buffer_level = 0

        self.command_queue = queue.Queue()

        self.getFunctions = [self.getMachineMode, self.getEstop]
        self.setFunctions = [self.setEcho, self.setDrivePower, self.setEnable, self.setHomeAll, self.setLogging, self.setMachineMode, self.setEstop]

    def run(self):
        self.motion_controller.start()
        while self._running_thread:
            self.checkSculptPrintUI()
            #if self.machine.logging_mode:
            ## FIXME move this to feedback thread
            #self.machine.current_position = self.data_store.stepgen_feedback_positions[-1,:] - self.machine.table_zero
            ## FIXME set up loop for running command queue
            ## FIXME update state machine every loop?
            pass

        # We are done running, clean up sockets
        self.motion_controller._running_thread = False
        print('MACHINE CONTROLLER: waiting on motion controller shutdown')
        if self.motion_controller.is_alive():
            self.motion_controller.join()

        self.waitForSet(self.setLogging, 0, self.getLogging)
        self.machine.feedback_listener_thread_handle._running_thread = False
        print('MACHINE CONTROLLER: waiting on feedback thread shutdown')
        if self.machine.feedback_listener_thread_handle.is_alive():
            self.machine.feedback_listener_thread_handle.join()

        self.machine.encoder_thread_handle._running_thread = False
        print('MACHINE CONTROLLER: waiting on encoder thread shutdown')
        if self.machine.encoder_thread_handle.is_alive():
            self.machine.encoder_thread_handle.join()

        print('MACHINE CONTROLLER: waiting on database thread shutdown')
        self.machine.data_store_manager_thread_handle._running_thread = False
        if self.machine.data_store_manager_thread_handle.is_alive():
            self.machine.data_store_manager_thread_handle.join()

        self.rsh_socket.shutdown(socket.SHUT_RDWR)
        self.rsh_socket.close()
        self._shutdown = True
        print('MACHINE CONTROLLER: thread shut down')


    ######################## Motion Controller Interface ########################
    def insertMove(self, move):
        #move.serial_number = len(self.motion_controller.move_queue) + 1
        # Populate parameters of the move
        move.polylines = self.motion_controller.polylines
        move.blocklength = self.motion_controller.blocklength
        move.servo_tx_array = self.formatPoints(move.point_samples, move.polylines, move.blocklength)
        self.motion_controller.move_queue.put(move)
        print('Put move ' + str(move.serial_number) + ' with type \"' + str(move.move_type) + '\" on motion controller queue')

    def enqueuePointFiles(self,start_file_number=5,end_file_number=10):
        try:
            start_file_number = 5
            end_file_number = 7
            #hold_points = self.generateHoldPositionPoints(1)
            hold_move = Move(self.generateHoldPositionPoints(5),'hold')
            #first_move_points = self.importAxesPoints(self.machine.point_files_path + self.machine.point_file_prefix + str(start_file))
            first_move = Move(self.importAxesPoints(self.machine.point_files_path + self.machine.point_file_prefix + str(start_file_number)), 'imported')
            rapid_to_start = Move(self.generateMovePoints(first_move.start_points.tolist()),'trap')

            self.insertMove(hold_move)
            #self.insertMove(rapid_to_start)
            #self.insertMove(first_move)

            for f in range(start_file_number + 1, end_file_number):
                fname = self.machine.point_files_path + self.machine.point_file_prefix + str(f)
                imported_move = Move(self.importAxesPoints(fname), 'imported', fname)
                #self.insertMove(imported_move)
        except Exception as error:
            print('Can\'t find those files, error ' + str(error))

    def testMachine(self,X,Y,Z,A,B):
        hold_points = self.generateHoldPositionPoints(1)
        self.insertMove(Move(hold_points,'hold'))
        rapid_to_zero = Move(self.generateMovePoints([0,0,0,0,0]),'trap')
        self.insertMove(rapid_to_zero)
        test_points = self.importAxesPoints(points_file + str('5'))

        test_move = Move(test_points,'imported')
        rapid_to_start = Move(self.generateMovePoints(test_move.start_points.tolist(),rapid_to_zero.end_points),'trap')

        self.insertMove(Move(self.generateHoldPositionPoints(1,[0,0,0,0,0]),'hold'))
        self.insertMove(rapid_to_start)
        print('inserted rapid')
        #self.motion_controller._running_motion = True
        self.insertMove(Move(self.generateHoldPositionPoints(1,test_move.start_points.tolist()),'hold'))
        self.insertMove(test_move)

        #for f in range(6,35+1):
        for f in range(6, 28 + 1):
            fname = points_file + str(f)
            imported_move = Move(self.importAxesPoints(fname),'imported',fname)
            self.insertMove(imported_move)

    def visualizePoints(self):
        points = np.empty((0,5), float)
        for move in self.motion_controller.move_queue:
            points = np.vstack((points,move.point_samples))
        plt.plot(points)
        plt.show()

    ######################## SculptPrint Interface ########################
    def checkSculptPrintUI(self):
        if self.machine.sculptprint_interface.connect_event.isSet():
            self.connectAndLink()
            self.machine.sculptprint_interface.connected = True
            self.machine.sculptprint_interface.connect_event.clear()

        if self.machine.sculptprint_interface.connected:
            if self.machine.sculptprint_interface.enqueue_moves_event.isSet():
                self.enqueuePointFiles(int(self.machine.sculptprint_interface.start_file),int(self.machine.sculptprint_interface.end_file))
                self.machine.sculptprint_interface.moves_queued_event.set()
                self.machine.sculptprint_interface.enqueue_moves_event.clear()

            if self.machine.sculptprint_interface.run_motion_event.isSet():
                if self.machine.sculptprint_interface.moves_queued_event.wait():
                    print('checkSculptPrintInterface running motion')
                    self.motion_controller._running_motion = True
                    self.machine.sculptprint_interface.moves_queued_event.clear()


    ######################## Data Handling ########################
    def convertFloat2Bin(self, num):
        return struct.pack('!f', num)

    def convertBin2Float(self,bytes):
        return struct.unpack('!f', bytes)

    def convertInt2Bin(self, num):
        return struct.pack('!i',num)

    def formatBinaryLine(self,axisCoords,polyLines,blockLength,positionInFile):
        return #position in file#

    def padAndFormatAxisPoints(self, points, polylines, blocklength):
        pad_points = np.lib.pad(points, ((0, blocklength - (np.size(points, 0) % blocklength)), (0, 0)), 'constant',
                                constant_values=points[-1])
        shape_points = pad_points.reshape((-1, blocklength), order='C')
        return np.pad(shape_points, ((0, polylines - (np.size(shape_points, 0) % polylines)), (0, 0)), 'constant',
                      constant_values=shape_points[-1, -1])

    def importAxisPoints(self, file, polylines, blocklength):
        points = np.array(list(csv.reader(open(file, "rt"), delimiter=","))).astype("float")
        return self.padAndFormatAxisPoints(points, polylines, blocklength)

    def importAxesPoints(self, file):
        ##FIXME check for overtravel
        points = np.array(list(csv.reader(open(file, "rt"), delimiter=" "))).astype("float")[:,:self.machine.number_of_joints]
        return points

    def formatPoints(self, points, polylines, block_length):
        axis_coords = []
        #FIXME fix if not divisible by polylines*blocklength
        for axis in range(points.shape[1]):
            axis_coords.append(self.padAndFormatAxisPoints(np.asarray([points[:, axis]]).T, polylines, block_length))
        return np.asarray(axis_coords).transpose(1, 2, 0)

    ######################## Writing Functions ########################
    def writeLineUTF(self,data):
        #print('sending ' + data)
        self.rsh_socket.send((data+'\r\n').encode('utf-8'))
        ##FIXME can we get rid of this?
        #time.sleep(0.25)

    def writeLineBin(self,frame):
        frame.extend(struct.pack('!f',np.inf))
        self.rsh_socket.send(frame)

    ######################## Setup Functions ########################
    ##FIXME implement heartbeat!
    def connectAndLink(self):
        #self.waitForSet(self.login,None,self.getLoginStatus)
        if not self.login()[0]:
            print('No response from RSH or state machine synchronization failure, assume crash. Please investigate')
            ##FIXME ssh to machine and restart process - major bandaid
            return False
        else:
            self.waitForSet(self.setEcho, 0, self.getEcho)
            #self.setBinaryMode(1)

            #FIXME do this a number of times and average
            # if self.getLatencyEstimate()[0]:
            #     print('successful estimation of network latency as ' + str(self.machine.estimated_network_latency))
            # else:
            #     print('failed to get network latency')

            while not self.syncMachineClock():# and self.machine.logging_mode:
                 #Busy wait for clock sync
                 print('waiting for clock sync')

            self.readyMachine()

            if self.machine.encoder_thread_handle.is_alive():
                print('MACHINE CONTROLLER: Waiting for decoder board initialization')
                self.machine.encoder_init_event.wait()

            # while self.machine.encoder_thread_handle.is_alive() and not self.machine.encoder_init_event.isSet():
            #     print('Waiting for encoder initialization')


        self.machine.sculptprint_interface.connect_event.set()


    def login(self, timeout = 0.5):
        self.machine.connection_change_event.clear()
        #self.machine.link_change_event.clear()
        self.writeLineUTF(self.machine.hello_string)
        self.waitForSet(self.setEnable,1,self.getEnable)
        if self.machine.connection_change_event.wait(timeout) and self.machine.link_change_event.wait(timeout):
            return (self.syncStateMachine() and True, self.machine.connected, self.machine.linked)
        else:
            return (False, self.machine.connected, self.machine.linked)

    def readyMachine(self):
        self.waitForSet(self.setEstop, 0, self.getEstop)
        self.waitForSet(self.setDrivePower, 1, self.getDrivePower)
        #self.waitForSet(self.setMachineMode, 'manual', self.getMachineMode)
        if not self.machine.isHomed():
            self.waitForSet(self.setHomeAll,None,self.getAllHomed)
        print('ready machine done homing')
        self.waitForSet(self.setMachineMode, 'auto', self.getMachineMode)
        self.waitForSet(self.setLogging, 1, self.getLogging)

    ############################# GETs #############################

    def getLatencyEstimate(self, timeout = 0.5):
        self.sendPing()
        if self.machine.ping_event.wait(timeout):
            return (True, self.machine.estimated_network_latency)
        else:
            return (False, self.machine.estimated_network_latency)

    def getClock(self):
        self.writeLineUTF('get time')
        return (self.machine.clock_event.wait(), self.machine.last_unix_time)

    def getAllHomed(self, timeout = 0.5):
        #self.writeLineUTF('get joint_homed')
        self.getHomeState()
        if self.machine.home_change_event.wait(timeout) and self.machine.all_homed_event.isSet():
            if not self.machine.restore_mode_event.isSet():
                print('restoring previous state')
                self.machine.restoreState()
            return (True, self.machine.axis_home_state)
        else:
            #print('getAllHomed returning False')
            return (False, self.machine.axis_home_state)

    def getHomeState(self, timeout = 0.5):
        self.machine.home_change_event.clear()
        self.writeLineUTF('get joint_homed')
        if self.machine.home_change_event.wait(timeout):
            return (True, self.machine.axis_home_state)
        else:
            return (False, self.machine.axis_home_state)

    def getEcho(self, timeout = 0.5):
        self.writeLineUTF('get echo')
        if self.machine.echo_change_event.wait(timeout):
            return (True, self.machine.echo)
        else:
            return (False, self.machine.echo)

    def getProgramStatus(self, timeout = 0.5):
        self.machine.status_change_event.clear()
        self.writeLineUTF('get program_status')
        return (self.machine.status_change_event.wait(timeout), self.machine.status)
        # if self.machine.status_change_event.wait(timeout):
        #     return (True, self.machine.status)
        # else:
        #     return (False, self.machine.status)


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

    def getEstop(self, timeout = 0.5):
        self.writeLineUTF('get estop')
        if self.machine.estop_change_event.wait(timeout):
            return (True, self.machine.estop)
        else:
            return (False, self.machine.estop)

    def getEnable(self, timeout = 0.5):
        self.writeLineUTF('get enable')
        if self.machine.link_change_event.wait(timeout):
            return (True, self.machine.linked)
        else:
            return (False, self.machine.linked)

    def getLogging(self, timeout = 0.5):
        #FIXME write this to get servo logging parameters
        self.writeLineUTF('get servo_log_params')
        if self.machine.logging_mode_change_event.wait(timeout):
            return (True, self.machine.logging_mode)
        else:
            return (False, self.machine.logging_mode)

    def getCommMode(self, timeout = 0.5):
        self.writeLineUTF('get comm_mode')
        if self.machine.comm_mode_change_event.wait(timeout):
            return (True, self.machine.comm_mode)
        else:
            return (False, self.machine.comm_mode)

    ############################# SETs #############################
    def sendPing(self):
        self.writeLineUTF('get ping')
        self.machine.ping_tx_time = time.clock()

    def setCommMode(self, flag):
        sendstr = 'set comm_mode '
        if flag:
            sendstr += 'binary'
        else:
            sendstr += 'ascii'
        self.machine.comm_mode_change_event.clear()
        self.writeLineUTF(sendstr)

    # def setBinaryMode(self, flag):
    #     ## FIXME confirm receipt
    #     if flag:
    #         print('setting binary mode on')
    #         self.machine.binary_mode = 1
    #         self.writeLineUTF('set comm_mode binary')
    #     else:
    #         print('setting binary mode off')
    #         self.rsh_socket.send(struct.pack('!f',-np.inf))
    #         self.machine.binary_mode = 0
    #         time.sleep(0.05)

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
        self.machine.echo_change_event.clear()
        self.writeLineUTF(sendstr)

    def setDrivePower(self,flag):
        sendstr = 'set machine '
        if flag:
            sendstr += 'on'
        else:
            sendstr += 'off'
        self.machine.drive_power_change_event.clear()
        self.writeLineUTF(sendstr)

    def setEnable(self,flag):
        sendstr = 'set enable '
        if flag:
            sendstr += 'EMCTOO'
        else:
            sendstr += 'off'
        self.machine.link_change_event.clear()
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
        if not sub_sample_rate:
            sub_sample_rate = self.machine.servo_log_sub_sample_rate
        if not buffer_size:
            buffer_size = self.machine.servo_log_buffer_size
        if not axes:
            axes = self.machine.servo_log_num_axes

        self.machine.logging_mode_change_event.clear()
        self.writeLineUTF('set servo_log_params ' + str(flag) + ' ' + str(axes) +
                          ' ' + str(sub_sample_rate) + ' ' + str(buffer_size) +
                          ' ' + str(dump_flag) + ' ' + str(write_buffer))

        # # Need to toggle logging mode
        # if self.machine.binary_mode:
        #     self.machine.logging_mode_change_event.clear()
        #     #print('disabling binary')
        #     #self.setBinaryMode(0)
        #     self.setMachineMode('auto')
        #     self.writeLineUTF('set servo_log_params ' + str(flag) + ' ' + str(axes) +
        #                       ' ' + str(sub_sample_rate) + ' ' + str(buffer_size) +
        #                       ' ' + str(dump_flag) + ' ' + str(write_buffer))
        #     self.setBinaryMode(1)
        #     #self.machine.logging_mode = not(self.machine.logging_mode)
        # else:
        #     self.machine.logging_mode_change_event.clear()
        #     #self.setMachineMode('auto')
        #     self.waitForSet(self.setMachineMode,'auto',self.getMachineMode)
        #     self.writeLineUTF('set servo_log_params ' + str(flag) + ' ' + str(axes) +
        #                       ' ' + str(sub_sample_rate) + ' ' + str(buffer_size) +
        #                       ' ' + str(dump_flag) + ' ' + str(write_buffer))

    def setHomeAll(self):
        #FIXME implement timeout
        if not self.machine.isManualMode():
            print('home all switching to manual')
            self.machine.pushState()
            self.waitForSet(self.setMachineMode,'manual',self.getMachineMode)

        for axis in range(0,self.machine.number_of_joints):
            self.writeLineUTF('set home ' + str(axis))

        self.machine.restore_mode_event.clear()

    ######################## Synchronization ########################
    def syncStateMachine(self):
        print('Starting initial state machine synchronization...')
        success_flag = True
        self.machine.estop_change_event.clear()
        success_flag and self.getEstop()[0]
        self.machine.drive_power_change_event.clear()
        success_flag and self.getDrivePower()[0]
        self.machine.mode_change_event.clear()
        success_flag and self.getMachineMode()[0]
        self.machine.logging_mode_change_event.clear()
        #print('getting logging mode')
        success_flag and self.getLogging()[0]
        #print('getting home state')
        self.machine.home_change_event.clear()
        success_flag and self.getHomeState()[0]
        #print('getting machine status')
        self.machine.status_change_event.clear()
        success_flag and self.getProgramStatus()[0]
        self.machine.comm_mode_change_event.clear()
        success_flag and self.getCommMode()[0]
        return success_flag

    def syncMachineClock(self):
        #if not self.machine.logging_mode or len(self.data_store.RTAPI_clock_times) < self.machine.servo_log_buffer_size:
        clock_data = self.machine.data_store_manager_thread_handle.pull(['RTAPI_clock_times', 'lowfreq_ethernet_received_times'],[0, 0],[1, 1])
        #pncApp_clock_data = self.machine.data_store_manager_thread_handle.pull(['lowfreq_ethernet_received_times'],[None],[1])

        # if not self.machine.logging_mode or not clock_data[0]:
        #     print('logging disabled or not enough data, can\'t sync clocks')
        #     return False

        if self.getClock()[0]:
            #FIXME this only syncs the RSH clock, which does not match RTAPI clock?
            print('Successful clock synchronization')
            self.machine.OS_clock_offset = self.machine.last_unix_time#clock_data[1][0].item()
            self.machine.pncApp_clock_offset = self.machine.clock_sync_received_time
            return True
        else:
            return False

        # self.machine.OS_clock_offset = clock_data[1][0].item()
        # #self.machine.OS_clock_offset = self.machine.last_unix_time
        # #self.machine.OS_clock_offset = self.data_store.RTAPI_clock_times[self.machine.servo_log_buffer_size-1].item()
        # #FIXME syncing to 0th is not good... subtract servo_period*num_samples
        # #self.machine.pncApp_clock_offset = self.data_store.lowfreq_ethernet_received_times[0].item()
        # self.machine.pncApp_clock_offset = clock_data[1][1].item()
        return True

    def waitForSet(self, set_function, set_params, get_function, timeout = 2):
        if set_params is not None:
            set_function(set_params)
        else:
            set_function()

        while not get_function()[0]:
            #FIXME check that returned value matches set_params
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

    ######################## Trajectory Generation ########################
    ## FIXME only allow movement generation when logging is enabled?

    def generateHoldPositionPoints(self, hold_time=1,position = [np.nan]):
        if any(np.isnan(position)):
            position_to_hold = self.machine.current_position
        else:
            position_to_hold = np.array(position)

        joint_position_samples = np.zeros([int(np.clip(np.round(hold_time/self.machine.servo_dt),1,np.inf)), int(self.machine.number_of_joints)]) + position_to_hold
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
        #print('starting move from ')
        #print(start_points)
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
            max_move_velocity = np.zeros([1,self.machine.number_of_joints])[0]
            for joint in range(0,self.machine.number_of_joints):
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

            #print('move direction')
            #print(move_direction)

            #Calculate move profile for each joint
            time_points = np.zeros([3, self.machine.number_of_joints])
            for joint in range(0, self.machine.number_of_joints):
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
            #print('slowest joint is ' + str(slowest_joint))
            move_time = time_points[2][slowest_joint]
            #print('move time is ' + str(move_time))

            motion_scale_factors = (np.asarray(time_points[2][:])/move_time)
            # Scale waypoint times for slowest axis
            time_points = np.multiply(np.asarray(time_points), np.divide(1,motion_scale_factors,out = np.zeros_like(motion_scale_factors),where = motion_scale_factors != 0)).tolist()

            #Sample time points to plan trajectories
            servo_times = np.arange(0,math.ceil(move_time/self.machine.servo_dt)*self.machine.servo_dt,self.machine.servo_dt)
            joint_position_samples = np.zeros([np.size(servo_times),self.machine.number_of_joints])
            #Force initial position
            joint_position_samples[0][:] = start_points

            #State machine for integration
            last_joint_positions = np.zeros([3,self.machine.number_of_joints]).tolist()
            phase_switch_times = np.zeros([3,self.machine.number_of_joints]).tolist()

            max_move_velocity = np.multiply(motion_scale_factors,max_move_velocity)
            max_joint_accelerations = np.multiply(np.power(motion_scale_factors,2), max_joint_accelerations)

            for ndx in range(0,np.size(servo_times)):
                t = servo_times[ndx]
                for joint in range(0, self.machine.number_of_joints):
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
        self._running_thread = False
        #self.exit()
        #sys.exit()


