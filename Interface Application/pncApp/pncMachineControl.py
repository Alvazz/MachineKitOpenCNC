import pncLibrary, struct, time, paramiko, websocket, json, asyncio, numpy as np
import websockets
from multiprocessing import Process, Queue, Event, current_process
from threading import Thread, current_thread
from queue import Empty
from io import StringIO

class CloudPathPlanner(Thread):
    def __init__(self, parent):
        super(CloudPathPlanner, self).__init__()
        self.name = "cloud_trajectory_planner"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer
        self.tp_websocket = websocket.WebSocket()
        self.client_GUID = '7ab22c19-3454-42ee-a68d-74c2789c4530'
        self.server_GUID = '1ce49dd2-042c-4bec-95bd-790f0d0ece54'

        self.path_serial_number = 0
        self.raw_point_queue = Queue()
        self.planned_point_queue = Queue()

        self.startup_event = Event()

    def run(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_launch_string, current_process().name,
                                                     self.name)

        self.startup_event.set()
        #with websockets.connect("wss://node2.wsninja.io") as self.tp_websocket:
        self.openConnection()
        while self.synchronizer.t_run_cloud_trajectory_planner.is_set():
            #FIXME use asynchronous receive so can send multiple point sets in one shot
            self.sendPoints()
            self.receivePoints()
            # planned_points = json.loads(self.tp_websocket.recv())
            # if not planned_points['serial_number'] == self.path_serial_number:
            #     raise pncLibrary.WebsocketError('Websocket point serial numbers didn\'t match')
            # self.planned_point_queue.put(self.unStringifyPoints(planned_points))

        # try:
        #     #with websockets.connect("wss://node2.wsninja.io")
        #     self.tp_websocket.connect("wss://node2.wsninja.io")
        #     self.tp_websocket.send(json.dumps({'guid': self.client_GUID}))
        #     ack_message = json.loads(self.tp_websocket.recv())
        #     if not ack_message['accepted']:
        #         raise pncLibrary.WebsocketError('Websocket connection failed')
        #
        #     while self.synchronizer.t_run_cloud_trajectory_planner.is_set():
        #         points_to_plan = self.raw_point_queue.get()
        #         self.path_serial_number += 1
        #         self.tp_websocket.send(json.dumps({'serial_number': str(self.path_serial_number), 'points': self.stringifyPoints(points_to_plan)}))
        #
        #         planned_points = json.loads(self.tp_websocket.recv())
        #         if not planned_points['serial_number'] == self.path_serial_number:
        #             raise pncLibrary.WebsocketError('Websocket point serial numbers didn\'t match')
        #         self.planned_point_queue.put(self.unStringifyPoints(planned_points))
        #
        # except Exception as error:
        #     self.synchronizer.q_print_server_message_queue.put("Websocket error: " + str(error))

        self.tp_websocket.close()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_terminate_string, current_process().name,
                                                     self.name)

    def openConnection(self):
        self.tp_websocket.connect("wss://node2.wsninja.io")
        self.tp_websocket.send(json.dumps({'guid': self.client_GUID}))
        ack_message = json.loads(self.tp_websocket.recv())
        if not ack_message['accepted']:
            raise pncLibrary.WebsocketError('Websocket connection failed')

    def stringifyPoints(self, point_array):
        #FIXME there is no way this can be this clunky
        points_fh = StringIO()
        np.savetxt(points_fh, point_array)
        return points_fh.getvalue().strip()

    def unStringifyPoints(self, point_string):
        return np.loadtxt(StringIO(point_string))

    def sendPoints(self):
        points_to_plan = self.raw_point_queue.get()
        self.path_serial_number += 1
        self.tp_websocket.send(
            json.dumps({'serial_number': str(self.path_serial_number), 'points': self.stringifyPoints(points_to_plan)}))

    def receivePoints(self):
        planned_points = json.loads(self.tp_websocket.recv())
        if not planned_points['serial_number'] == self.path_serial_number:
            raise pncLibrary.WebsocketError('Websocket point serial numbers didn\'t match')
        self.planned_point_queue.put(self.unStringifyPoints(planned_points))


class SystemController(Thread):
    def __init__(self, parent):
        super(SystemController, self).__init__()
        #FIXME set up SSH session here
        self.machine = parent.machine
        self._running_thread = True
        self._run_ssh_connection = False

        self.ssh_client = paramiko.SSHClient()

    def run(self):
        while self.p_run_ssh_client.is_set():
            if self._run_ssh_connection:
                while True:
                    pass
        self.ssh_client.close()

class MotionController(Thread):
    def __init__(self, parent):
        super(MotionController, self).__init__()
        #self.parent = current_thread()
        self.name = "motion_controller"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer

        self.move_queue = Queue()
        self.move_in_progress = 0
        self.last_move_serial_number = 0
        self.current_move_serial_number = 0

        ### Network control parameters
        self.polylines = self.machine.polylines_per_tx
        self.blocklength = self.machine.points_per_polyline

        self.startup_event = Event()

    def run(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.thread_launch_string, current_process().name, self.name)
        self.startup_event.set()

        while self.synchronizer.t_run_motion_controller_event.is_set():
            if self.synchronizer.mc_run_motion_event.wait(self.machine.thread_queue_wait_timeout):
                #print('length of move queue is ' + str(self.move_queue.qsize()))

                #There are new moves in the queue, find the one to be executed next
                while not self.move_queue.empty() and not self.machine.rsh_error:
                    move_to_execute = self.move_queue.get()
                    self.current_move_serial_number += 1
                    move_to_execute.serial_number = self.current_move_serial_number

                    #move_to_execute = self.move_queue[self.last_move_serial_number + 1 - 1]
                    #print('executing move ' + str(move_to_execute.serial_number))
                    #self.move_in_progress = move_to_execute

                    self.synchronizer.q_database_command_queue_proxy.put(
                        pncLibrary.DatabaseCommand('push_object', [{"EXECUTED_MOVES": move_to_execute}]))

                    try:
                        self.commandPoints(move_to_execute.servo_tx_array, self.polylines, self.blocklength)
                    except pncLibrary.RSHError:
                        self.synchronizer.q_print_server_message_queue.put("MOTION CONTROLLER: Detected RSH error, aborting motion")
                        self.move_queue = Queue()
                        break

                    self.last_move_serial_number = move_to_execute.serial_number
                    self.move_queue.task_done()


        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_terminate_string, current_process().name,
                                                     self.name)

    def commandPoints(self, servo_points, polylines, blocklength, commands_to_send = -1):
        if commands_to_send == -1:
            commands_to_send = int(servo_points.shape[0] / polylines)

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
                raise pncLibrary.RSHError("Detected RSH error after " + str(command) + " commands")
                return

            pncLibrary.socketLockedWrite(self.machine, self.synchronizer, binary_command)

            #FIXME check buffer was flushed
            sleep_time = self.runNetworkPID(self.machine.rsh_buffer_level, blocklength, polylines, self.machine.buffer_level_setpoint)
            self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push', [{'COMMANDED_SERVO_POLYLINES': commanded_points, 'NETWORK_PID_DELAYS': sleep_time}]))
            time.sleep(sleep_time)

    def runNetworkPID(self, current_buffer_length, block_length, polylines, set_point_buffer_length, Kp=.03, Ki=0,
                      Kd=0):
        if (self.machine.max_buffer_level - current_buffer_length) < 100:
            print('WARNING: Buffer finna overflow')
        sleep_time = max((block_length * polylines) / 1000 - (Kp * ((set_point_buffer_length - current_buffer_length))) / 1000,0)
        return sleep_time

    def adaptNetworkTxGain(self):
        pass


class MachineController(Process):
    def __init__(self, machine, pipe):
        super(MachineController, self).__init__()
        self.name = "machine_controller"
        self.main_thread_name = self.name + ".MainThread"
        self.machine = machine
        self.feed_pipe = pipe
        #self.command_queue = synchronizer.q_machine_controller_command_queue

        self.getFunctions = [self.getMachineMode, self.getEstop]
        self.setFunctions = [self.setEcho, self.setDrivePower, self.setEnable, self.setHomeAll, self.setServoFeedbackMode, self.setMachineMode, self.setEstop]

        self._running_process = True

    def run(self):
        current_thread().name = self.main_thread_name
        pncLibrary.getSynchronizer(self, self.feed_pipe)
        #self.socket_mutex = threadLock()
        #self.waitForThreadStart()
        pncLibrary.waitForThreadStart(self, MotionController, CloudPathPlanner)

        self.synchronizer.mc_startup_event.set()
        self.synchronizer.process_start_signal.wait()
        time.clock()

        #FIXME protect with try...except to print errors to terminalprintserver
        if self.synchronizer.p_enable_machine_controller_event.is_set():
            self.synchronizer.mc_successful_start_event.set()
            #self.synchronizer.p_run_machine_controller_event.wait()
            while self.synchronizer.p_run_machine_controller_event.is_set():
                #self.checkSculptPrintUI()
                try:
                    command = self.synchronizer.q_machine_controller_command_queue.get(True, self.machine.process_queue_wait_timeout)
                    self.handleCommand(command)
                except Empty:
                    pass
                except Exception as error:
                    self.synchronizer.q_print_server_message_queue.put("MACHINE CONTROLLER: Had error: " + str(error))

                if self.synchronizer.mc_rsh_error_event.is_set():
                    pncLibrary.waitForErrorReset()

        self.synchronizer.t_run_motion_controller_event.clear()
        self.motion_controller.join()

        #FIXME only do this if machine is connected
        if self.synchronizer.mvc_connected_event.is_set():
            self.prepareMachineForDisconnect()

    def handleCommand(self, command):
        if command.command_type == 'CONNECT':
            self.connectAndLink()
            self.synchronizer.mvc_connect_event.set()
        elif command.command_type == "ENQUEUE":
            #self.enqueuePointFiles(command.command_data[0], command.data[2])
            self.enqueuePointFiles(command.command_data[0], command.command_data[1])
        elif command.command_type == "EXECUTE":
            self.synchronizer.mc_run_motion_event.set()


    # def waitForThreadStart(self):
    #     self.motion_controller = MotionController(self.machine, self.synchronizer)
    #     self.cloud_trajectory_planner = CloudPathPlanner(self.machine, self.synchronizer)
    #     self.motion_controller.start()
    #     self.cloud_trajectory_planner.start()
    #     self.motion_controller.startup_event.wait()
    #     self.cloud_trajectory_planner.startup_event.wait()

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
            hold_move = pncLibrary.Move(self.generateHoldPositionPoints(5),'hold')
            #first_move_points = self.importAxesPoints(self.machine.point_files_path + self.machine.point_file_prefix + str(start_file))
            first_move = pncLibrary.Move(self.importAxesPoints(self.machine.point_files_path + self.machine.point_file_prefix + str(start_file_number)), 'imported')
            rapid_to_start = pncLibrary.Move(self.generateMovePoints(first_move.start_points.tolist()),'trap')

            self.insertMove(hold_move)
            self.insertMove(rapid_to_start)
            #self.insertMove(first_move)

            for f in range(start_file_number + 1, end_file_number):
                fname = self.machine.point_files_path + self.machine.point_file_prefix + str(f)
                imported_move = pncLibrary.Move(self.importAxesPoints(fname), 'imported', fname)
                #self.insertMove(imported_move)
        except Exception as error:
            print('Can\'t find those files, error ' + str(error))

    ######################## SculptPrint MVC ########################
    # def checkSculptPrintUI(self):
    #     #FIXME this is a waste of CPU, use a stateupdate event from SP interface
    #     if self.synchronizer.mvc_connect_event.is_set():
    #         self.connectAndLink()
    #         self.machine.sculptprint_interface.connected = True
    #         self.synchronizer.mvc_connect_event.clear()
    #
    #     if self.machine.sculptprint_interface.connected:
    #         if self.synchronizer.mvc_enqueue_moves_event.is_set():
    #             self.enqueuePointFiles(int(self.machine.sculptprint_interface.start_file),int(self.machine.sculptprint_interface.end_file))
    #             self.synchronizer.mvc_moves_queued_event.set()
    #             self.synchronizer.mvc_enqueue_moves_event.clear()
    #
    #         if self.synchronizer.mvc_run_motion_event.is_set():
    #             if self.synchronizer.mvc_moves_queued_event.wait():
    #                 print('checkSculptPrintInterface running motion')
    #                 self.motion_controller._running_motion = True
    #                 self.synchronizer.mvc_moves_queued_event.clear()

        #time.sleep(0.1)

    ######################## Writing Functions ########################
    def socketLockedWrite(self, data):
        with self.synchronizer.mc_socket_lock:
            self.machine.rsh_socket.send(data)

    def writeLineUTF(self,data):
        #FIXME protect with mutex?
        self.socketLockedWrite((data+'\r\n').encode('utf-8'))

    # def writeLineBin(self,data):
    #     data.extend(struct.pack('!f',np.inf))
    #     self.rsh_socket.send(data)

    ######################## Setup Functions ########################
    ##FIXME implement heartbeat!
    def connectAndLink(self):
        #self.waitForSet(self.login,None,self.getLoginStatus)
        if not self.login()[0]:
            self.synchronizer.q_print_server_message_queue.put('MACHINE CONTROLLER: No response from RSH or state machine synchronization failure, assume crash. Please investigate')
            ##FIXME ssh to machine and restart process - major bandaid
            return False
        else:
            self.waitForSet(self.setEcho, 0, self.getEcho)
            try:
                self.readyMachine()
            except pncLibrary.MachineControllerError:
                self.synchronizer.q_print_server_message_queue.put('MACHINE CONTROLLER: Homing timeout')
                return False
            #self.setBinaryMode(1)

            #FIXME do this a number of times and average
            if self.getLatencyEstimate()[0]:
                self.synchronizer.q_print_server_message_queue.put('MACHINE CONTROLLER: Successful estimation of network latency as ' + str(self.machine.estimated_network_latency))
            else:
                self.synchronizer.q_print_server_message_queue.put('MACHINE_CONTROLLER: Failed to get network latency')


            while not self.syncMachineClock():# and self.machine.servo_feedback_mode:
                 #Busy wait for clock sync
                 print('waiting for clock sync')

            if not self.synchronizer.ei_encoder_init_event.is_set():
                print('MACHINE CONTROLLER: Waiting for encoder inital position set')
                self.synchronizer.ei_encoder_init_event.wait()

            self.waitForSet(self.setCommMode, 1, self.getCommMode)

        self.synchronizer.q_print_server_message_queue.put("MACHINE CONTROLLER: Control initialization successful")
        self.synchronizer.mvc_connected_event.set()

    def login(self, timeout = 0.5):
        self.synchronizer.fb_connection_change_event.clear()
        #self.machine.link_change_event.clear()
        self.writeLineUTF(self.machine.hello_string)
        self.waitForSet(self.setEnable,1,self.getEnable)
        if self.synchronizer.fb_connection_change_event.wait(timeout) and self.synchronizer.fb_link_change_event.wait(timeout):
            return (self.syncStateMachine() and True, self.machine.connected, self.machine.linked)
        else:
            return (False, self.machine.connected, self.machine.linked)

    def readyMachine(self):
        self.waitForSet(self.setEstop, 0, self.getEstop)
        self.waitForSet(self.setDrivePower, 1, self.getDrivePower)
        #self.waitForSet(self.setMachineMode, 'manual', self.getMachineMode)

        if not pncLibrary.isHomed(self.machine, self.synchronizer):
            self.waitForSet(self.setHomeAll,None,self.getAllHomed, 10)
            self.synchronizer.q_print_server_message_queue.put("MACHINE CONTROLLER: All axes homed")

        self.synchronizer.q_print_server_message_queue.put("MACHINE CONTROLLER: All axes homed")
        self.waitForSet(self.setMachineMode, 'auto', self.getMachineMode)
        self.waitForSet(self.setServoFeedbackMode, 1, self.getServoFeedbackMode)
        self.waitForSet(self.setBufferLevelFeedbackMode, 1, self.getBufferLevelFeedbackMode)

    ############################# GETs #############################
    def getLatencyEstimate(self, timeout = 0.5):
        self.sendPing()
        if self.synchronizer.fb_ping_event.wait(timeout):
            return (True, self.machine.estimated_network_latency)
        else:
            return (False, self.machine.estimated_network_latency)

    def getClock(self):
        self.writeLineUTF('get time')
        return (self.synchronizer.fb_clock_event.wait(), self.machine.last_unix_time)

    def getAllHomed(self, timeout = 1):
        #self.writeLineUTF('get joint_homed')
        self.getHomeState()
        if self.synchronizer.fb_home_change_event.wait(timeout) and self.synchronizer.fb_all_homed_event.is_set():
            if not self.synchronizer.mc_restore_mode_event.is_set():
                print('restoring previous state')
                pncLibrary.restoreState()
            return (True, self.machine.axis_home_state)
        else:
            #print('getAllHomed returning False')
            return (False, self.machine.axis_home_state)

    def getHomeState(self, timeout = 0.5):
        self.synchronizer.fb_home_change_event.clear()
        self.writeLineUTF('get joint_homed')
        if self.synchronizer.fb_home_change_event.wait(timeout):
            return (True, self.machine.axis_home_state)
        else:
            return (False, self.machine.axis_home_state)

    def getEcho(self, timeout = 0.5):
        self.writeLineUTF('get echo')
        if self.synchronizer.fb_echo_change_event.wait(timeout):
            return (True, self.machine.echo)
        else:
            return (False, self.machine.echo)

    def getProgramStatus(self, timeout = 0.5):
        self.synchronizer.fb_status_change_event.clear()
        self.writeLineUTF('get program_status')
        return (self.synchronizer.fb_status_change_event.wait(timeout), self.machine.status)

    def getMachineMode(self, timeout = None):
        if timeout is None:
            timeout = self.machine.event_wait_timeout
        self.synchronizer.fb_mode_change_event.clear()
        self.writeLineUTF('get mode')
        return (self.synchronizer.fb_mode_change_event.wait(timeout), self.machine.mode)
        # if self.synchronizer.fb_mode_change_event.wait(timeout):
        #     return (True, self.machine.mode)
        # else:
        #     return (False, self.machine.mode)
        
    def getDrivePower(self, timeout = None):
        if timeout is None:
            timeout = self.machine.event_wait_timeout
        self.synchronizer.fb_drive_power_change_event.clear()
        self.writeLineUTF('get machine')
        return (self.synchronizer.fb_drive_power_change_event.wait(timeout),self.machine.drive_power)
        # if self.synchronizer.fb_drive_power_change_event.wait(timeout):
        #     return (True, self.machine.drive_power)
        # else:
        #     return (False, self.machine.drive_power)

    def getEstop(self, timeout = 0.5):
        #FIXME clear event flag before GET from machine for all methods here
        self.writeLineUTF('get estop')
        if self.synchronizer.fb_estop_change_event.wait(timeout):
            return (True, self.machine.estop)
        else:
            return (False, self.machine.estop)

    def getEnable(self, timeout = 0.5):
        self.writeLineUTF('get enable')
        if self.synchronizer.fb_link_change_event.wait(timeout):
            return (True, self.machine.linked)
        else:
            return (False, self.machine.linked)

    def getServoFeedbackMode(self, timeout = 0.5):
        #FIXME write this ALL to get servo logging parameters
        self.writeLineUTF('get servo_log_params')
        if self.synchronizer.fb_servo_logging_mode_change_event.wait(timeout):
            return (True, self.machine.servo_feedback_mode)
        else:
            return (False, self.machine.servo_feedback_mode)

    def getBufferLevelFeedbackMode(self, timeout = None):
        if timeout is None:
            timeout = self.machine.event_wait_timeout
        self.synchronizer.fb_buffer_level_feedback_mode_change_event.clear()
        self.writeLineUTF('get buffer_level_feedback')
        return (self.synchronizer.fb_buffer_level_feedback_mode_change_event.wait(timeout), self.machine.buffer_level_feedback_mode)

    def getCommMode(self, timeout = 0.5):
        self.writeLineUTF('get comm_mode')
        if self.synchronizer.fb_comm_mode_change_event.wait(timeout):
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
        self.synchronizer.fb_comm_mode_change_event.clear()
        self.writeLineUTF(sendstr)

    def setMachineMode(self, mode):
        self.synchronizer.fb_mode_change_event.clear()
        self.writeLineUTF('set mode ' + mode.upper())
        
    def setMDILine(self,line):
        self.writeLineUTF('set mdi ' + line)

    def setEcho(self,flag):
        sendstr = 'set echo '
        if flag:
            sendstr += 'on'
        else:
            sendstr += 'off'
        self.synchronizer.fb_echo_change_event.clear()
        self.writeLineUTF(sendstr)

    def setDrivePower(self,flag):
        sendstr = 'set machine '
        if flag:
            sendstr += 'on'
        else:
            sendstr += 'off'
        self.synchronizer.fb_drive_power_change_event.clear()
        self.writeLineUTF(sendstr)

    def setEnable(self,flag):
        sendstr = 'set enable '
        if flag:
            sendstr += 'EMCTOO'
        else:
            sendstr += 'off'
        self.synchronizer.fb_link_change_event.clear()
        self.writeLineUTF(sendstr)

    def setEstop(self, flag):
        sendstr = 'set estop '
        if flag:
            sendstr += 'on'
        else:
            sendstr += 'off'
        self.synchronizer.fb_estop_change_event.clear()
        self.writeLineUTF(sendstr)

    def setServoFeedbackMode(self, flag, sub_sample_rate=0, buffer_size=0, axes=0, dump_flag=0, write_buffer=0):
        if not sub_sample_rate:
            sub_sample_rate = self.machine.servo_log_sub_sample_rate
        if not buffer_size:
            buffer_size = self.machine.servo_log_buffer_size
        if not axes:
            axes = self.machine.servo_log_num_axes

        self.synchronizer.fb_servo_logging_mode_change_event.clear()
        self.writeLineUTF('set servo_log_params ' + str(flag) + ' ' + str(axes) +
                          ' ' + str(sub_sample_rate) + ' ' + str(buffer_size) +
                          ' ' + str(dump_flag) + ' ' + str(write_buffer))

    def setBufferLevelFeedbackMode(self, flag, feedback_period = None):
        if feedback_period is None:
            feedback_period = self.machine.buffer_level_feedback_period_us
        self.synchronizer.fb_buffer_level_feedback_mode_change_event.clear()
        self.writeLineUTF('set buffer_level_feedback ' + str(flag) + ' ' + str(feedback_period))

    def setHomeAll(self):
        #FIXME implement timeout
        if not pncLibrary.isManualMode(self.machine, self.synchronizer):
            print('home all switching to manual')
            pncLibrary.pushState(self.machine)
            self.waitForSet(self.setMachineMode,'manual',self.getMachineMode)

        self.synchronizer.fb_all_homed_event.clear()
        for axis in range(0,self.machine.number_of_joints):
            self.writeLineUTF('set home ' + str(axis))

        self.synchronizer.mc_restore_mode_event.clear()

    ######################## Synchronization ########################
    def syncStateMachine(self):
        print('Starting initial state machine synchronization...')
        success_flag = True
        self.synchronizer.fb_estop_change_event.clear()
        success_flag and self.getEstop()[0]
        self.synchronizer.fb_drive_power_change_event.clear()
        success_flag and self.getDrivePower()[0]
        self.synchronizer.fb_mode_change_event.clear()
        success_flag and self.getMachineMode()[0]
        self.synchronizer.fb_servo_logging_mode_change_event.clear()
        #print('getting logging mode')
        success_flag and self.getServoFeedbackMode()[0]
        #print('getting home state')
        self.synchronizer.fb_home_change_event.clear()
        success_flag and self.getHomeState()[0]
        #print('getting machine status')
        self.synchronizer.fb_status_change_event.clear()
        success_flag and self.getProgramStatus()[0]
        self.synchronizer.fb_comm_mode_change_event.clear()
        success_flag and self.getCommMode()[0]
        return success_flag

    def syncMachineClock(self):
        if self.getClock()[0]:
            #FIXME this only syncs the RSH clock, which does not match RTAPI clock?
            self.machine.OS_clock_offset = self.machine.last_unix_time
            self.machine.pncApp_clock_offset = self.machine.clock_sync_received_time
            self.synchronizer.mc_clock_sync_event.set()
            self.synchronizer.mc_xenomai_clock_sync_event.wait()
            print('Successful clock synchronization')
            return True
        else:
            return False

    def waitForSet(self, set_function, set_params, get_function, timeout = None):
        ## FIXME really need to implement this timeout
        if timeout is None:
            timeout = np.inf

        start_time = time.clock()

        if set_params is not None:
            set_function(set_params)
        else:
            set_function()

        while not get_function()[0] and (start_time-time.clock() <= timeout):
            #FIXME check that returned value matches set_params
            print('waiting for ' + str(get_function))

        if (start_time-time.clock()) > timeout:
            raise pncLibrary.MachineControllerError('Timeout with ' + str(get_function))
        else:
            print('success: ' + str(get_function) + ' returned True')

        ## FIXME call set again after some time?
        ## FIXME set up timeout here
        #print('logging mode is ' + str(self.machine.servo_feedback_mode))

    def checkMachineReady(self, timeout = 0.5):
        if self.pncLibrary.isAutoMode(self.machine, self.synchronizer) and self.getProgramStatus()[0] and self.machine.status == 'IDLE':
            return True
        else:
            print('Machine not ready: isAutoMode returned ' + str(pncLibrary.isAutoMode(self.machine, self.synchronizer)) + ' and machine status is ' + self.machine.status)
            return False

    def prepareMachineForDisconnect(self):
        return self.waitForSet(self.setEstop, 1, self.getEstop, 1)

    # def close(self):
    #     print('in machine controller close')
    #     #self._running_thread = False
    #     #self.exit()
    #     #sys.exit()


