import pncLibrary
import struct, time, socket, select, math, queue, signal, paramiko
import numpy as np, matplotlib.pyplot as plt
from multiprocessing import Process
from threading import Thread, current_thread

class SystemController(Thread):
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

class MotionController(Thread):
    def __init__(self, machine):
        super(MotionController, self).__init__()
        #self.parent = current_thread()
        self.machine = machine
        #self.machine_controller = machine_controller

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
                #self.machine_controller.waitForSet(self.machine_controller.setCommMode,1,self.machine_controller.getCommMode)

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
                    self.parent.database_push_queue.put({"EXECUTED_MOVES": move_to_execute})
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
        #self.machine_controller.database_push_queue_proxy.current_servo_points = servo_points

        if commands_to_send == -1:
            print("sending all points")
            commands_to_send = int(servo_points.shape[0] / polylines)
            print(commands_to_send)

        #Form binary command string
        for command in range(0, commands_to_send):
            commanded_points = []
            binary_command = (struct.pack('!' + str(len(self.machine.binary_direct_mode_header)) + 'sii',
                                              self.machine.binary_direct_mode_header.encode('utf-8'), polylines,
                                              blocklength))

            for polyline in range(0, polylines):
                for axis in range(0, servo_points.shape[2]):
                    for point in range(0, servo_points.shape[1]):
                        binary_command += self.machine_controller.convertFloat2Bin(servo_points[(command * polylines) + polyline, point, axis])
                        commanded_points.append(servo_points[(command * polylines) + polyline, point, axis])

            binary_command += self.machine.binary_line_terminator
            if self.machine.rsh_error:
                print('commandPoints detected RSH error after ' + str(command) + ' commands')
                return

            self.machine.buffer_level_reception_event.clear()
            #FIXME do this better insertion into data_store
            #self.machine_controller.data_store.sent_servo_commands.append(commanded_points)
            self.parent.database_push_queue_proxy.put({'commanded_servo_points': commanded_points})
            self.machine_controller.rsh_socket.send(binary_command)
            #FIXME check buffer was flushed

            ## FIXME store current position from feedback thread
            #self.machine_controller.machine.current_position = np.reshape(axis_coords[-1, -1, :], [1, self.machine.number_of_joints])[0]

            sleep_time = self.runNetworkPID(self.machine.rsh_buffer_level, blocklength, polylines, self.machine.buffer_level_setpoint)
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


class MachineController(Process):
    def __init__(self, machine, database_push_queue_proxy):
        super(MachineController, self).__init__()
        #print('in machine controller init')
        self.machine = machine
        self.database_push_queue_proxy = database_push_queue_proxy
        #self.rsh_socket = self.machine.rsh_socket
        #self.encoder_interface = encoder_interface
        #


        self._shutdown = False
        self.received_data_string = ""
        #self.data_store = data_store
        self.rsh_buffer_level = 0

        #self.command_queue = queue.Queue()

        self.getFunctions = [self.getMachineMode, self.getEstop]
        self.setFunctions = [self.setEcho, self.setDrivePower, self.setEnable, self.setHomeAll, self.setLogging, self.setMachineMode, self.setEstop]

        self._running_process = True


    def run(self):
        self.motion_controller = MotionController(self.machine)
        self.motion_controller.start()

        while self._running_process:
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
            self.insertMove(rapid_to_start)
            #self.insertMove(first_move)

            for f in range(start_file_number + 1, end_file_number):
                fname = self.machine.point_files_path + self.machine.point_file_prefix + str(f)
                imported_move = Move(self.importAxesPoints(fname), 'imported', fname)
                #self.insertMove(imported_move)
        except Exception as error:
            print('Can\'t find those files, error ' + str(error))

    ######################## SculptPrint MVC ########################
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

    ######################## Writing Functions ########################
    def writeLineUTF(self,data):
        self.rsh_socket.send((data+'\r\n').encode('utf-8'))

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
            #     print('successful estimation of network latency as ' + str(self.machine.current_estimated_network_latency))
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

    def close(self):
        print('in machine controller close')
        self._running_thread = False
        #self.exit()
        #sys.exit()


