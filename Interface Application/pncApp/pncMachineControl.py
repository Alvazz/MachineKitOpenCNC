import pncLibrary, struct, time, paramiko, websocket, json, asyncio, numpy as np
#import websockets
from multiprocessing import Process, Queue, Event, current_process
from threading import Thread, current_thread
from queue import Empty
from io import StringIO, BytesIO
from socket import timeout as SocketTimeout

class CloudTrajectoryPlanner(Thread):
    def __init__(self, parent):
        super(CloudTrajectoryPlanner, self).__init__()
        self.name = "cloud_trajectory_planner"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer
        self.tp_websocket = websocket.WebSocket()
        self.tp_websocket.timeout = self.machine.websocket_timeout
        self.websocket_connected_event = Event()
        #self.client_GUID = '7ab22c19-3454-42ee-a68d-74c2789c4530'
        #self.server_GUID = '1ce49dd2-042c-4bec-95bd-790f0d0ece54'

        self.current_requested_sequence_id = 0
        self.current_received_sequence_id = 0

        self.raw_point_queue = Queue()
        self.planned_point_output_queue = Queue()

        self.planned_point_payload_buffer = []
        joint_data_file = "machine"
        tool_data_file = "tool"

        self.startup_event = Event()

    def run(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_launch_string, current_process().name,
                                                     self.name)

        self.startup_event.set()
        #with websockets.connect("wss://node2.wsninja.io") as self.tp_websocket:
        self.openConnection()
        while self.synchronizer.t_run_cloud_trajectory_planner_event.is_set() and self.websocket_connected_event.is_set():
            #FIXME use asynchronous receive so can send multiple point sets in one shot
            self.raw_point_queue.put(np.array([6, 6, 6]))
            self.sendPoints()
            self.receivePoints()
            # planned_points = json.loads(self.tp_websocket.recv())
            # if not planned_points['serial_number'] == self.path_serial_number:
            #     raise pncLibrary.WebsocketError('Websocket point serial numbers didn\'t match')
            # self.planned_point_output_queue.put(self.unStringifyPoints(planned_points))

        self.tp_websocket.send('FIN')
        self.tp_websocket.close()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_terminate_string, current_process().name,
                                                     self.name)

    def openConnection(self):
        self.tp_websocket.connect("wss://node2.wsninja.io")
        self.tp_websocket.send(json.dumps({'guid': self.machine.websocket_client_GUID}))
        ack_message = json.loads(self.tp_websocket.recv())
        if not ack_message['accepted']:
            raise pncLibrary.WebsocketError('Websocket connection failed')
        #else:

        try:
            self.tp_websocket.send('HELLO')
            ack_message = self.tp_websocket.recv()
            if ack_message != 'HELLO_ACK':
                raise pncLibrary.WebsocketError('MukulLab not online')
            self.websocket_connected_event.set()
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         self.machine.trajectory_planner_connection_string, self.name,
                                                         'MukulLab')
        except websocket.WebSocketTimeoutException as error:
            print('MukulLab not online, error: ' + str(error))
            # pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
            #                                              self.machine.trajectory_planner_connection_failed_string, self.name,
            #                                              'MukulLab')
        except pncLibrary.WebsocketError as error:
            print('MukulLab did not ack properly: ' + str(error))




    def stringifyPoints(self, point_array):
        #FIXME there is no way this can be this clunky
        points_fh = StringIO()
        np.savetxt(points_fh, point_array)
        return points_fh.getvalue().strip()

    def unStringifyPoints(self, point_string):
        return np.loadtxt(StringIO(point_string))

    def sendPoints(self):
        points_to_plan = self.raw_point_queue.get()
        data_stream = BytesIO()
        np.savez_compressed(data_stream, pid=points_to_plan[0],
                            tool=points_to_plan[1], joint=points_to_plan[2])
        data_stream.seek(0)
        self.tp_websocket.send_binary(data_stream.read())

        # self.tp_websocket.send(
        #     json.dumps({'serial_number': str(self.path_serial_number), 'points': self.stringifyPoints(points_to_plan)}))

    def receivePoints(self):
        planned_point_payload = np.load(self.tp_websocket.recv())
        self.planned_point_payload_buffer.append(planned_point_payload)

        path_id = planned_point_payload['pid']
        sequence_id = planned_point_payload['sid']
        planned_points = planned_point_payload['planned']

    def sortAndOutputPoints(self):
        for p in range(0,len(self.planned_point_payload_buffer)):
            planned_point_payload = self.planned_point_payload_buffer[p]
            if planned_point_payload['sid'] == self.current_requested_sequence_id:
                self.current_requested_sequence_id += 1
                self.planned_point_output_queue.put(self.planned_point_payload_buffer.pop(p))
                break
        self.sortAndOutputPoints()
            # path_id = planned_point_payload['pid']
            # sequence_id = planned_point_payload['sid']
            # planned_points = planned_point_payload['planned']

    def enqueuePointsForPlanning(self, tool_space_data, joint_space_data):
        self.raw_point_queue.put((self.path_serial_number, tool_space_data, joint_space_data))
        return self.path_serial_number

class OperatingSystemController(Thread):
    def __init__(self, parent):
        super(OperatingSystemController, self).__init__()
        #FIXME set up SSH session here
        self.name = "operating_system_controller"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer

        self.ssh_client = paramiko.SSHClient()
        self.command_queue = Queue()
        self.startup_event = Event()

        self.open_streams = []

    def run(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_launch_string, current_process().name,
                                                     self.name)
        self.startup_event.set()
        self.connectToOperatingSystem(pncLibrary.ssh_wait_timeout)
        while self.synchronizer.t_run_operating_system_controller.is_set() and self.synchronizer.os_ssh_connected_event.is_set():
            try:
                command = self.command_queue.get(True, self.machine.process_queue_wait_timeout)
                self.handleCommand(command)
            except Empty:
                pass

            #FIXME log ssh output of open streams
            # self.logStreams()

            #pass
            # if self._run_ssh_connection:
            #     while True:
            #         pass
        self.ssh_client.close()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.ssh_connection_close_string, self.name,
                                                     self.machine.ssh_credentials[0])
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_terminate_string, current_process().name,
                                                     self.name)

    def connectToOperatingSystem(self, timeout):
        #FIXME raise exception if this doesn't work
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy)
        try:
            self.ssh_client.connect(self.machine.ip_address, self.machine.ssh_port, self.machine.ssh_credentials[0],
                                    self.machine.ssh_credentials[1], timeout=timeout)
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         self.machine.ssh_connection_success_string,
                                                         self.machine.ssh_credentials[0], self.machine.ip_address)
            self.synchronizer.os_ssh_connected_event.set()
        except (TimeoutError, SocketTimeout) as error:
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.ssh_connection_failure_string,
                                                         self.machine.ssh_credentials[0],
                                                         self.machine.ip_address, str(error))


        # if self.ssh_client.connect(self.machine.ip_address, self.machine.ssh_port, self.machine.ssh_credentials[0], self.machine.ssh_credentials[1], timeout=timeout):
        #     pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
        #                                                  self.machine.ssh_connection_success_string, self.machine.ssh_credentials[0],
        #                                                  self.machine.ip_address)
        #     self.synchronizer.os_ssh_connected_event.set()
        #     return True
        # else:
        #     pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
        #                                                  self.machine.ssh_connection_failure_string, self.machine.ssh_credentials[0],
        #                                                  self.machine.ip_address)
        #     return False

    def handleCommand(self, command):
        if command.command_type == 'RUN_CNC':
            stdin, stdout, stderr = self.ssh_client.exec_command('linuxcnc')
        elif command.command_type == 'CHECK_CNC_RUN':
            stdin, stdout, stderr = self.ssh_client.exec_command('ps -e | grep -w linuxcnc')
            self.setEventFlag(stdout, self.synchronizer.os_linuxcnc_running_event)
        elif command.command_type == 'CHECK_RSH_RUN':
            stdin, stdout, stderr = self.ssh_client.exec_command('ps -e | grep -w linuxcncrsh')
            self.setEventFlag(stdout, self.synchronizer.os_linuxcncrsh_running_event)
        elif command.command_type == 'RUN_RSH':
            stdin, stdout, stderr = self.ssh_client.exec_command('linuxcncrsh &')
            self.open_streams.append(stdout)
        elif command.command_type == 'KILL_RSH':
            stdin, stdout, stderr = self.ssh_client.exec_command('sudo pkill linuxcncrsh')

    def runCNC(self):
        if not self.getRSHStatus()[0]:
            self.waitForSet(self.startRSH, None, self.getRSHStatus)

    def setEventFlag(self, stdout, event):
        if stdout.read():
            event.set()
        else:
            event.clear()

    def logStreams(self):
        for stream in self.open_streams:
            stream_data = stream.read()
            if stream_data:
                self.synchronizer.q_database_command_queue_proxy.put('log', 'SSH CLIENT: ' + str(time.clock()) + ' : ' + stream_data.encode())

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

                    self.synchronizer.q_database_command_queue_proxy.put(
                        pncLibrary.DatabaseCommand('push_object', [{"EXECUTED_MOVES": move_to_execute}]))

                    try:
                        self.commandPoints(move_to_execute.servo_tx_array, self.polylines, self.blocklength)
                    except pncLibrary.RSHError:
                        self.synchronizer.q_print_server_message_queue.put("MOTION CONTROLLER: Detected RSH error, aborting motion")
                        self.move_queue = Queue()
                        break

                    self.last_move_serial_number = move_to_execute.serial_number
                    #self.move_queue.task_done()


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
                        binary_command += pncLibrary.convertFloat2Bin(servo_points[(command * polylines) + polyline, point, axis])
                        commanded_points.append(servo_points[(command * polylines) + polyline, point, axis])

            binary_command += self.machine.binary_line_terminator

            if self.machine.rsh_error:
                raise pncLibrary.RSHError("Detected RSH error after " + str(command) + " commands")
                return

            tx_time = time.time()
            pncLibrary.socketLockedWrite(self.machine, self.synchronizer, binary_command)

            #FIXME check buffer was flushed
            sleep_time = self.runNetworkPID(self.machine.rsh_buffer_level, blocklength, polylines, self.machine.buffer_level_setpoint)
            self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push_object', [{'COMMANDED_SERVO_POLYLINES': commanded_points}]))
            self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push', [{'NETWORK_PID_DELAYS': np.array([[sleep_time]]), 'POLYLINE_TRANSMISSION_TIMES': np.array([[tx_time-self.machine.pncApp_clock_offset]])}]))
            time.sleep(sleep_time)

    def runNetworkPID(self, current_buffer_length, block_length, polylines, set_point_buffer_length, Kp=.1, Ki=0,
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

        self.planned_point_buffer = []
        self.support_threads = [OperatingSystemController, MotionController]

        self.getFunctions = [self.getMachineMode, self.getEstop]
        self.setFunctions = [self.setEcho, self.setDrivePower, self.setEnable, self.setHomeAll, self.setServoFeedbackMode, self.setMachineMode, self.setEstop]

        #self._running_process = True

    def run(self):
        current_thread().name = self.main_thread_name
        pncLibrary.getSynchronizer(self, self.feed_pipe)
        #FIXME detect thread launch failure
        pncLibrary.waitForThreadStart(self, MotionController)#, CloudTrajectoryPlanner)

        self.synchronizer.mc_startup_event.set()
        self.synchronizer.process_start_signal.wait()
        time.clock()

        #self.runCNC()

        if self.synchronizer.p_enable_machine_controller_event.is_set():
            self.synchronizer.mc_successful_start_event.set()
            #self.synchronizer.p_run_machine_controller_event.wait()
            while self.synchronizer.p_run_machine_controller_event.is_set():

                try:
                    #FIXME don't catch errors from handleCommand
                    command = self.synchronizer.q_machine_controller_command_queue.get(True, self.machine.process_queue_wait_timeout)
                    self.handleCommand(command)
                except Empty:
                    pass
                except Exception as error:
                    self.synchronizer.q_print_server_message_queue.put("MACHINE CONTROLLER: Had error: " + str(error))

                #self.updateTrajectory()

                if self.synchronizer.mc_rsh_error_event.is_set():
                    pncLibrary.waitForErrorReset()

        pncLibrary.waitForThreadStop(self, self.motion_controller, self.cloud_trajectory_planner, self.operating_system_controller)

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

    def updateTrajectory(self):
        while not self.cloud_trajectory_planner.planned_point_queue.empty():
            planned_move = self.cloud_trajectory_planner.planned_point_queue.get()
            self.insertMove(pncLibrary.Move(planned_move['planned'], 'remotely_planned', planned_move['sid']))
        # if not self.cloud_trajectory_planner.planned_point_queue.empty():
        #     while not self.cloud_trajector

    # path_id = planned_point_payload['pid']
    # sequence_id = planned_point_payload['sid']
    # planned_points = planned_point_payload['planned']
        #if self.cloud_trajectory_planner.is_alive():
    ######################## OS Interface ########################
    def runCNC(self):
        if not self.getRSHStatus()[0]:
            self.waitForSet(self.startRSH, None, self.getRSHStatus)

    ######################## Motion Controller Interface ########################
    def insertMove(self, move):
        # Populate parameters of the move
        move.polylines = self.motion_controller.polylines
        move.blocklength = self.motion_controller.blocklength
        move.servo_tx_array = pncLibrary.TP.formatPoints(move.point_samples, move.polylines, move.blocklength)
        self.motion_controller.move_queue.put(move)
        print('Put move ' + str(move.serial_number) + ' with type \"' + str(move.move_type) + '\" on motion controller queue')

    def enqueuePointFiles(self,start_file_number=5,end_file_number=10):
        start_file_number = 5
        end_file_number = 7
        #hold_points = self.generateHoldPositionPoints(1)
        hold_move = pncLibrary.Move(pncLibrary.TP.generateHoldPositionPoints(self.machine, 5),'hold')
        #first_move_points = self.importAxesPoints(self.machine.point_files_path + self.machine.point_file_prefix + str(start_file))
        first_move = pncLibrary.Move(pncLibrary.TP.importPoints(self.machine, self.machine.point_files_path + self.machine.point_file_prefix + str(start_file_number)), 'imported')
        rapid_to_start = pncLibrary.Move(pncLibrary.TP.generateMovePoints(self.machine, first_move.start_points.tolist()),'trap')

        self.insertMove(hold_move)
        self.insertMove(rapid_to_start)
        self.insertMove(first_move)

        try:
            for f in range(start_file_number + 1, end_file_number):
                fname = self.machine.point_files_path + self.machine.point_file_prefix + str(f)
                imported_move = pncLibrary.Move(pncLibrary.TP.importPoints(self.machine, fname), 'imported', fname)
                self.insertMove(imported_move)
        except Exception as error:
            print('Can\'t find those files, error ' + str(error))

    ######################## Writing Functions ########################
    def socketLockedWrite(self, data):
        with self.synchronizer.mc_socket_lock:
            self.machine.rsh_socket.send(data)

    def writeLineUTF(self,data):
        #FIXME protect with mutex?
        self.socketLockedWrite((data+'\r\n').encode('utf-8'))

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
            if self.getLatencyEstimate(10)[0]:
                self.synchronizer.q_print_server_message_queue.put('MACHINE CONTROLLER: Successful estimation of mean network latency as ' + str(self.machine.current_estimated_network_latency))
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
    def getLinuxCNCStatus(self, timeout = 0.5):
        self.synchronizer.os_linuxcnc_running_event.clear()
        self.operating_system_controller.command_queue.put(pncLibrary.OSCommand('CHECK_CNC_RUN'))
        if self.synchronizer.os_linuxcnc_running_event.wait(timeout):
            return (True, self.synchronizer.os_linuxcnc_running_event.is_set())
        else:
            return (False, self.synchronizer.os_linuxcnc_running_event.is_set())

    def getRSHStatus(self, timeout = 0.5):
        self.synchronizer.os_linuxcncrsh_running_event.clear()
        self.operating_system_controller.command_queue.put(pncLibrary.OSCommand('CHECK_RSH_RUN'))
        if self.synchronizer.os_linuxcncrsh_running_event.wait(timeout):
            return (True, self.synchronizer.os_linuxcncrsh_running_event.is_set())
        else:
            return (False, self.synchronizer.os_linuxcncrsh_running_event.is_set())

    def getLatencyEstimate(self, pings_to_send, timeout = 0.5):
        print('Estimating latency with %i pings' % pings_to_send)
        ping_times = np.array([[]])
        for ping_number in range(0, pings_to_send):
            self.sendPing()
            time.sleep(pncLibrary.machine_ping_delay_time)
            try:
                self.synchronizer.fb_ping_event.wait(timeout)
                ping_times = np.hstack((ping_times, np.array([[self.machine.current_estimated_network_latency]])))
                success_flag = True
            except TimeoutError:
                print('MACHINE CONTROLLER: Latency estimation timed out after %i pings' % ping_number)
                success_flag = False
                break

        self.machine.mean_network_latency = np.mean(ping_times)
        pncLibrary.asynchronousPush(self.synchronizer, {'PINGS': ping_times})
        #self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push', {'PINGS': ping_times}))
        return (success_flag, self.machine.mean_network_latency)

            # if self.synchronizer.fb_ping_event.wait(timeout):
            #     return (True, self.machine.current_estimated_network_latency)
            # else:
            #     return (False, self.machine.current_estimated_network_latency)

    def getClock(self):
        self.writeLineUTF('get time')
        return (self.synchronizer.fb_clock_event.wait(), self.machine.last_unix_time)

    def getAllHomed(self, timeout = 1):
        #self.writeLineUTF('get joint_homed')
        self.getHomeState()
        if self.synchronizer.fb_home_change_event.wait(timeout) and self.synchronizer.fb_all_homed_event.is_set():
            # if not self.synchronizer.mc_restore_mode_event.is_set():
            #     print('restoring previous state')
            #     pncLibrary.restoreState()
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
        
    def getDrivePower(self, timeout = None):
        if timeout is None:
            timeout = self.machine.event_wait_timeout
        self.synchronizer.fb_drive_power_change_event.clear()
        self.writeLineUTF('get machine')
        return (self.synchronizer.fb_drive_power_change_event.wait(timeout),self.machine.drive_power)

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
    def startLinuxCNC(self):
        self.operating_system_controller.command_queue.put(pncLibrary.OSCommand('RUN_CNC'))

    def startRSH(self):
        self.operating_system_controller.command_queue.put(pncLibrary.OSCommand('RUN_RSH'))

    def killRSH(self):
        self.operating_system_controller.command_queue.put(pncLibrary.OSCommand('KILL_RSH'))

    def sendPing(self):
        self.machine.ping_tx_time = time.time()
        self.writeLineUTF('get ping')

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
            #FIXME add some delay?
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


