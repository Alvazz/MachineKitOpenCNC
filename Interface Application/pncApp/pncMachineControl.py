import pncLibrary, time, paramiko, websocket, json, numpy as np
#import websockets
from multiprocessing import Process, Queue, Event, current_process
from threading import Thread, current_thread
from queue import Empty
from io import StringIO, BytesIO
from socket import timeout as SocketTimeout
from pncMotionControl import MotionController

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
        self.connectToTP()
        while self.synchronizer.t_run_cloud_trajectory_planner_event.is_set() and self.websocket_connected_event.is_set():
            #FIXME use asynchronous receive so can send multiple point sets in one shot
            #self.raw_point_queue.put(np.array([6, 6, 6]))
            #self.raw_point_queue.put([np.array([1]), np.random.rand(10,7), np.random.rand(10,8)])
            self.raw_point_queue.put([np.array([1]), np.loadtxt(self.machine.raw_point_files_path + 'tool_short.txt', skiprows=1), np.loadtxt(self.machine.raw_point_files_path + 'machine_short.txt', skiprows=1)])
            self.sendPoints()
            self.receivePoints()

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

        # try:
        #     self.tp_websocket.send('HELLO')
        #     ack_message = self.tp_websocket.recv()
        #     if ack_message != 'HELLO_ACK':
        #         raise pncLibrary.WebsocketError('MukulLab not online')
        #     self.websocket_connected_event.set()
        #     pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
        #                                                  self.machine.trajectory_planner_connection_string, self.name,
        #                                                  'MukulLab')
        # except websocket.WebSocketTimeoutException as error:
        #     print('MukulLab not online, error: ' + str(error))
        #     # pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
        #     #                                              self.machine.trajectory_planner_connection_failed_string, self.name,
        #     #                                              'MukulLab')
        # except pncLibrary.WebsocketError as error:
        #     print('MukulLab did not ack properly: ' + str(error))

    def connectToTP(self):
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
        except pncLibrary.WebsocketError as error:
            print('MukulLab did not ack properly: ' + str(error))

    # def stringifyPoints(self, point_array):
    #     #FIXME there is no way this can be this clunky
    #     points_fh = StringIO()
    #     np.savetxt(points_fh, point_array)
    #     return points_fh.getvalue().strip()
    #
    # def unStringifyPoints(self, point_string):
    #     return np.loadtxt(StringIO(point_string))

    def sendPoints(self):
        points_to_plan = self.raw_point_queue.get()
        data_stream = BytesIO()
        np.savez_compressed(data_stream, pid=points_to_plan[0],
                            tool=points_to_plan[1], joint=points_to_plan[2])
        data_stream.seek(0)
        self.tp_websocket.send_binary(data_stream.read())

    def receivePoints(self):
        payload = self.tp_websocket.recv()
        byte_stream = BytesIO(payload)
        byte_stream.seek(0)
        planned_point_payload = np.load(byte_stream)

        path_id = planned_point_payload['pid']
        sequence_id = planned_point_payload['sid']
        planned_points = planned_point_payload['planned_points']

        self.planned_point_payload_buffer.append(planned_point_payload)

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

    # Transform applies f based on file index. First three coordinates are for the translation axis.
    # Rest are rotational, irrespective of the file type.
    def _units_transform(self, data, dist_slice=slice(None, 3, None), rot_slice=slice(3, None, None),
                         correct_dist=True, correct_rot=True):
        # Convert to radian if rotational axis
        if correct_rot:
            data[rot_slice, :] = (data[rot_slice, :] * np.pi) / 180
        if correct_dist:
            data[dist_slice, :] = data[dist_slice, :] * self.conversion_factor_dist
        return data

    # Returns desired slices of tool and joint data files.
    # Tool file order - x,y,z,volume,a,b
    # Joint file order - X,Y,Z,A,B,S
    def load_sp_data(self, data_folder, joint_data_file, tool_data_file,
                     data_slice=slice(None, None, None),
                     subsample_slice=slice(None, None, None)):
        tool_data = np.loadtxt(data_folder + tool_data_file, skiprows=1)
        joint_data = np.loadtxt(data_folder + joint_data_file, skiprows=1)

        return self.load_sp_data_from_arrays(joint_data, tool_data, subsample_slice)

    # Load SculptPrint data from numpy arrays
    def load_sp_data_from_arrays(self, joint_data, tool_data,
                                 subsample_slice=slice(None, None, None)):
        # Joint file indexing order is a mapping from n+1 -> n+1 that maps coordinate order -> index in file
        # The last index is for the spindle coordinate.
        joint_file_indexing_order = [1, 4, 2, 5, 6, 3]
        # Joint file indexing order is a mapping from n -> n that maps
        # coordinate order -> index in file -- in the file written out by
        # SculptPrint, the 6th index is the volumes, and the 5th is if this is
        # a flag or not.
        tool_file_indexing_order = [0, 1, 2, 6, 3, 4, 5]

        tool_temp_data = tool_data.T
        joint_temp_data = joint_data.T

        tool_analysis_data = np.zeros((len(tool_file_indexing_order), tool_temp_data.shape[1]))
        for index in np.arange(len(tool_file_indexing_order)):
            tool_analysis_data[index] = tool_temp_data[tool_file_indexing_order[index]]
        tool_analysis_data = self._units_transform(tool_analysis_data,
                                                   dist_slice=slice(None, 4, None),
                                                   rot_slice=slice(4, 5, None), correct_rot=False)

        joint_analysis_data = np.zeros((len(joint_file_indexing_order), joint_temp_data.shape[1]))
        for index in np.arange(len(joint_file_indexing_order)):
            joint_analysis_data[index] = joint_temp_data[joint_file_indexing_order[index]]
        joint_analysis_data = self._units_transform(joint_analysis_data)

        return (tool_analysis_data[:, subsample_slice],
                joint_analysis_data[:, subsample_slice],
                tool_analysis_data[-1, subsample_slice].astype(dtype=bool))

    # Given data (:, n) and flag(1, n) of bools, split into
    # contiguous set of slices based on flag
    def obtain_contiguous_sequences(self, flag):
        switch_indices = np.where((flag[1:] - flag[:-1]) != 0)[0]
        switch_indices = np.hstack((np.array([-1]), switch_indices))
        contiguous_lists = []
        all_same = []
        for index in np.arange(switch_indices.size - 1):
            contiguous_lists.append((flag[switch_indices[index] + 1],
                                     slice(switch_indices[index] + 1,
                                           switch_indices[index + 1] + 1, None)))
            if __debug__:
                all_same.append(np.any(flag[switch_indices[index] + 1:
                                            switch_indices[index + 1] + 1] -
                                       flag[switch_indices[index] + 1]))
        if __debug__:
            print("All contiguous flags are same", not
            np.any(np.asarray(all_same)))
        return contiguous_lists

    def get_combined_data(self, joint_data, tool_data):
        return (np.vstack((tool_data[:3], tool_data[4:6],
                           tool_data[3], joint_data[:6],
                           tool_data[6])))

    def write_combined_data(self, joint_data, tool_data, filename):
        combined_analysis_data = self.get_combined_data(joint_data, tool_data).T
        np.savetxt(filename, combined_analysis_data)

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
            self.ssh_client.connect(self.machine.ip_address, self.machine.ssh_port, username=self.machine.ssh_credentials[0],
                                    password=self.machine.ssh_credentials[1], allow_agent=False, look_for_keys=False, timeout=timeout)
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

    def run(self):
        current_thread().name = self.main_thread_name
        pncLibrary.getSynchronizer(self, self.feed_pipe)
        #FIXME detect thread launch failure
        pncLibrary.waitForThreadStart(self, MotionController, CloudTrajectoryPlanner)

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
            #self.enqueuePointFiles(command.command_data[0], command.command_data[1])
            #self.enqueueTrapezoidalTest()
            self.enqueuePointFiles(command.command_data[0], command.command_data[1])
        elif command.command_type == "EXECUTE":
            print('machine mode is ' + self.machine.mode)
            self.waitForSet(self.setMachineMode, 'auto', self.getMachineMode)
            if self.machine.mode != 'AUTO':
                print('mode not auto')
            self.synchronizer.mc_run_motion_event.set()
        elif command.command_type == "RESET_MOTION":
            pass

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
        limit_check = pncLibrary.TP.checkMoveOvertravel(move.point_samples, self.machine.absolute_axis_travel_limits)
        if not limit_check[0]:
            move.polylines = self.motion_controller.polylines
            move.blocklength = self.motion_controller.blocklength
            move.servo_tx_array = pncLibrary.TP.formatPoints(pncLibrary.TP.convertMotionCS(self.machine, 'table center', move.point_samples), move.polylines, move.blocklength)
            self.motion_controller.move_queue.put(move)
            print('Put move ' + str(move.serial_number) + ' with type \"' + str(move.move_type) + '\" on motion controller queue')
            return True
        else:
            print('move exceeds machine limits at point %d for axis ??', limit_check[1])
            return False

    def enqueueTrapezoidalTest(self, iterations=3):
        hold_move = pncLibrary.Move(pncLibrary.TP.generateHoldPositionPoints(self.machine, 5), 'hold')
        rapid_to_start = pncLibrary.Move(pncLibrary.TP.generateMovePoints(self.machine, pncLibrary.TP.convertMotionCS(self.machine, 'absolute', np.zeros(5)), move_type='trapezoidal'), 'trap')

        rapid1 = pncLibrary.Move(
            pncLibrary.TP.generateMovePoints(self.machine,
                                             pncLibrary.TP.convertMotionCS(self.machine, 'absolute', np.array([1,1,-1,20,20])),
                                             pncLibrary.TP.convertMotionCS(self.machine, 'absolute', np.zeros(5)),
                                             move_type='trapezoidal'), 'trap')
        rapid0 = pncLibrary.Move(
            pncLibrary.TP.generateMovePoints(self.machine,
                                             pncLibrary.TP.convertMotionCS(self.machine, 'absolute', np.zeros(5)),
                                             pncLibrary.TP.convertMotionCS(self.machine, 'absolute',
                                                                           np.array([1, 1, -1, 20, 20])),
                                             move_type='trapezoidal'), 'trap')

        self.insertMove(hold_move)
        self.insertMove(rapid_to_start)
        for k in range(0, iterations):
            self.insertMove(rapid1)
            self.insertMove(rapid0)

    def enqueuePointFiles(self,start_file_number=5,end_file_number=10):
        start_file_number = 1
        end_file_number = 20
        #hold_points = self.generateHoldPositionPoints(1)
        hold_move = pncLibrary.Move(pncLibrary.TP.generateHoldPositionPoints(self.machine, 5),'hold')
        #first_move_points = self.importAxesPoints(self.machine.point_files_path + self.machine.point_file_prefix + str(start_file))
        first_move = pncLibrary.Move(pncLibrary.TP.importPoints(self.machine, self.machine.point_files_path + self.machine.point_file_prefix + str(start_file_number)), 'imported')
        rapid_to_start = pncLibrary.Move(pncLibrary.TP.generateMovePoints(self.machine, first_move.start_points, move_type='trapezoidal'),'trap')

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

            if self.getLatencyEstimate(10)[0]:
                self.synchronizer.q_print_server_message_queue.put('MACHINE CONTROLLER: Successful estimation of mean network latency as ' + str(self.machine.current_estimated_network_latency))
            else:
                self.synchronizer.q_print_server_message_queue.put('MACHINE_CONTROLLER: Failed to get network latency')

            try:
                self.readyMachine()
            except pncLibrary.MachineControllerError:
                self.synchronizer.q_print_server_message_queue.put('MACHINE CONTROLLER: Homing timeout')
                return False

            while not self.syncMachineClock():# and self.machine.servo_feedback_mode:
                 #Busy wait for clock sync
                 print('waiting for clock sync')

            if not self.synchronizer.ei_encoder_init_event.is_set() and self.synchronizer.ei_encoder_comm_init_event.is_set():
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


