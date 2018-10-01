import pncLibrary, time, paramiko, numpy as np
from multiprocessing import Process, Queue, Event, current_process
from threading import Thread, current_thread
from queue import Empty
from socket import timeout as SocketTimeout
from pncMotionControl import MotionController
from pncWebInterface import CloudTrajectoryPlannerInterface

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
                                                     pncLibrary.printout_thread_launch_string, current_process().name,
                                                     self.name)
        self.startup_event.set()

        # print("SHORTING OS CONTROLLER")
        # self.synchronizer.os_ssh_connected_event.set()
        self.connectToOperatingSystem(pncLibrary.ssh_wait_timeout)

        while self.synchronizer.t_run_operating_system_controller.is_set() and self.synchronizer.os_ssh_connected_event.is_set():
            try:
                command = self.command_queue.get(True, self.machine.process_queue_wait_timeout)
                self.handleCommand(command)
            except Empty:
                pass

            #FIXME log ssh output of open streams
            # self.logStreams()

        self.ssh_client.close()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_ssh_connection_close_string, self.name,
                                                     self.machine.ssh_credentials[0])
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_thread_terminate_string, current_process().name,
                                                     self.name)

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

    def connectToOperatingSystem(self, timeout):
        #FIXME raise exception if this doesn't work
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy)
        try:
            self.ssh_client.connect(self.machine.ip_address, self.machine.ssh_port, username=self.machine.ssh_credentials[0],
                                    password=self.machine.ssh_credentials[1], allow_agent=False, look_for_keys=False, timeout=timeout)
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_ssh_connection_success_string,
                                                         self.machine.ssh_credentials[0], self.machine.ip_address)
            self.synchronizer.os_ssh_connected_event.set()
        except (TimeoutError, SocketTimeout) as error:
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_ssh_connection_failure_string,
                                                         self.machine.ssh_credentials[0],
                                                         self.machine.ip_address, str(error))

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

        self.planned_point_buffer = []
        self.enqueued_move_serial_number = 0
        self.support_threads = [OperatingSystemController, MotionController]

    def run(self):
        current_thread().name = self.main_thread_name
        pncLibrary.getSynchronizer(self, self.feed_pipe)
        #FIXME detect thread launch failure
        pncLibrary.waitForThreadStart(self, MotionController, CloudTrajectoryPlannerInterface)

        self.synchronizer.mc_startup_event.set()
        self.synchronizer.process_start_signal.wait()
        time.clock()

        #self.runCNC()

        if self.synchronizer.p_enable_machine_controller_event.is_set():
            self.synchronizer.mc_successful_start_event.set()
            #self.synchronizer.p_run_machine_controller_event.wait()
            while self.synchronizer.p_run_machine_controller_event.is_set():
                #self.createRapidTrajectoryPlanRequest(*self.generateRapidPoints(end_joint=np.array([0,0,0,0,0])))
                try:
                    #Handle any errors first
                    if self.synchronizer.mc_rsh_error_event.is_set():
                        self.handleRSHError()

                    #FIXME don't catch errors from handleCommand
                    command = self.synchronizer.q_machine_controller_command_queue.get(True, self.machine.process_queue_wait_timeout)
                    self.handleCommand(command)
                except Empty:
                    pass
                except Exception as error:
                    self.synchronizer.q_print_server_message_queue.put("MACHINE CONTROLLER: Had error: " + str(error))

                #self.updateTrajectoryFromTP()

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
        elif command.command_type == "ENQUEUE_VOXELIZED":
            self.enqueuePointFiles(command.command_data[0], command.command_data[1])
        elif command.command_type == "ENQUEUE_TRAPEZOID":
            #self.enqueueTrapezoidalTest(command.command_data[0])
            self.enqueueTrapezoidalTest(1, 90, 2)
        elif command.command_type == "EXECUTE":
            print('machine mode is ' + self.machine.mode)
            #self.waitForSet(self.setMachineMode, 'auto', self.getMachineMode)
            #self.waitForSet(self.setServoFeedbackMode, 0, self.getServoFeedbackMode)
            # if self.machine.mode != 'AUTO':
            #     print('mode not auto')
            #self.waitForSet(self.setServoFeedbackMode, 0, self.getServoFeedbackMode)
            #self.synchronizer.mc_run_motion_event.set()
            self.beginMotionExecution()
        elif command.command_type == "PLAN_SEQUENCES":
            self.enqueuePointFiles(command.command_data[0], command.command_data[1])
        elif command.command_type == "RESET_MOTION":
            self.motion_controller.motion_queue = Queue()
            pass
        elif command.command_type == 'PLAN':
            self.beginPlanningTrajectory(command.command_data[0])
        elif command.command_type == 'SETUPTOOLPATH':
            self.setupToolpath(command.command_data)

    def handleRSHError(self):
        self.synchronizer.mc_rsh_error_event.clear()
        self.motion_controller.interrupt_motion_event.set()
        self.synchronizer.mc_motion_complete_event.clear()
        self.waitForSet(self.setEstop, 0, self.getEstop)
        self.waitForSet(self.setDrivePower, 1, self.getDrivePower)
        self.waitForSet(self.setMachineMode, 'auto', self.getMachineMode)

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
            move.serial_number = self.enqueued_move_serial_number
            self.enqueued_move_serial_number += 1
            self.motion_controller.move_queue.put(move)
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, pncLibrary.printout_move_queue_insertion_string, str(move.serial_number), move.move_type)
            return True
        else:
            print('MACHINE CONTROLLER: Move exceeds machine limits at point %d for axis ??', limit_check[1])
            return False

    def enqueueTrapezoidalTest(self, translational_distance, angular_distance, iterations=3):
        hold_move = pncLibrary.Move(pncLibrary.TP.generateHoldPositionPoints(self.machine, 2), move_type='hold')
        rapid_to_start = pncLibrary.Move(pncLibrary.TP.generateMovePoints(self.machine, pncLibrary.TP.convertMotionCS(self.machine, 'absolute', np.zeros(5)), move_type='trapezoidal'), 'trap')

        rapid1 = pncLibrary.Move(
            pncLibrary.TP.generateMovePoints(self.machine,
                                             pncLibrary.TP.convertMotionCS(self.machine, 'absolute', np.array([translational_distance, translational_distance, -translational_distance, angular_distance, angular_distance])),
                                             pncLibrary.TP.convertMotionCS(self.machine, 'absolute', np.zeros(5)),
                                             move_type='trapezoidal'), 'trap')
        rapid0 = pncLibrary.Move(
            pncLibrary.TP.generateMovePoints(self.machine,
                                             pncLibrary.TP.convertMotionCS(self.machine, 'absolute', np.zeros(5)),
                                             pncLibrary.TP.convertMotionCS(self.machine, 'absolute',
                                                                           np.array([translational_distance, translational_distance, -translational_distance, angular_distance, angular_distance])),
                                             move_type='trapezoidal'), 'trap')

        hold1 = pncLibrary.Move(pncLibrary.TP.generateHoldPositionPoints(self.machine, 2, np.array([translational_distance, translational_distance, -translational_distance, angular_distance, angular_distance])))
        hold0 = pncLibrary.Move(pncLibrary.TP.generateHoldPositionPoints(self.machine, 2, np.zeros(5)))
        self.insertMove(hold_move)
        self.insertMove(rapid_to_start)
        for k in range(0, iterations):
            self.insertMove(rapid1)
            self.insertMove(hold1)
            self.insertMove(rapid0)
            self.insertMove(hold0)

    def enqueuePointFiles(self,start_file_number=5,end_file_number=10):
        start_file_number = 1
        end_file_number = 8
        #hold_points = self.generateHoldPositionPoints(1)
        hold_move = pncLibrary.Move(pncLibrary.TP.generateHoldPositionPoints(self.machine, 5),move_type='hold')
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

    def generateRapidPoints(self, end_joint=None, start_joint=None, mode='joint_space'):
        if start_joint is None:
            start_joint = self.machine.current_stepgen_position

        if mode == 'joint_space':
            joint_point_samples = pncLibrary.TP.linearlyInterpolateTrajectory(np.hstack((start_joint, -np.pi/2)), np.hstack((end_joint, -np.pi/2)), 0.05)
            tool_point_samples = np.asarray(
                self.machine.FK(*pncLibrary.TP.rotaryAxesToRadians(joint_point_samples).T,
                                *self.machine.tool_translation_vector,
                                self.machine.workpiece_translation_vector))

            return (tool_point_samples.T, np.hstack((pncLibrary.TP.rotaryAxesToDegrees(joint_point_samples[:,:5]), -90. * np.ones_like(joint_point_samples[:,:1]))))

        elif mode == 'tool_space':
            start_tool = np.asarray(self.machine.FK(*pncLibrary.TP.rotaryAxesToRadians(np.array([start_joint]))[0], -np.pi/2.,
                                                    *self.machine.tool_translation_vector,
                                                    self.machine.workpiece_translation_vector))
            end_tool = np.asarray(self.machine.FK(*pncLibrary.TP.rotaryAxesToRadians(np.array([end_joint]))[0], -np.pi/2.,
                                                  *self.machine.tool_translation_vector,
                                                  self.machine.workpiece_translation_vector))

            tool_point_samples = pncLibrary.TP.linearlyInterpolateTrajectory(start_tool, end_tool, 0.002)
            joint_point_samples = np.hstack((pncLibrary.TP.rotaryAxesToDegrees(self.machine.IK(tool_point_samples[:, :3].T,
                (self.machine.tool_translation_vector.T * np.ones((tool_point_samples.shape[0], 3))).T,
                                                  tool_point_samples[:, 3].T, tool_point_samples[:, 4].T,
                                                  -np.pi / 2 + np.zeros_like(tool_point_samples[:, 0]),
                                                  self.machine.workpiece_translation_vector))[:,:5],
                                             -90. * np.ones_like(tool_point_samples[:,:1])))
            return (tool_point_samples, joint_point_samples)

    def createRapidTrajectoryPlanRequest(self, id, tool_points, joint_points):
        path_id = np.array([self.cloud_trajectory_planner.tp_state.rapid_sequence_id])
        full_tool_points = np.vstack((tool_points.T, 1 + np.zeros_like(tool_points.T[0]), np.zeros_like(tool_points.T[0])))[pncLibrary.TP_tool_file_indexing_order]
        full_joint_points = np.vstack((pncLibrary.TP.rotaryAxesToRadians(joint_points).T, -np.pi/2. + np.zeros_like(joint_points.T[0])))  # [joint_indices]
        self.cloud_trajectory_planner.tp_state.rapid_sequence_id += 1
        self.cloud_trajectory_planner.raw_point_queue.put((path_id, full_tool_points, full_joint_points, 'rapid'))

    def initializeTrajectory(self, mode='remote'):
        if mode == 'local':
            hold_move = pncLibrary.Move(pncLibrary.TP.offsetAxes(pncLibrary.TP.generateHoldPositionPoints(
                self.machine, 2), machine=self.machine, direction=1), move_type='hold', offset_axes=False)
            print('generated hold')

            rapid_to_start = pncLibrary.Move(pncLibrary.TP.offsetAxes(pncLibrary.TP.generateMovePoints(
                self.machine, self.motion_controller.motion_queue_feeder.shifted_motion_start_points,
                move_type='trapezoidal'), machine=self.machine, direction=1),
                move_type='trapezoidal', offset_axes=False)
            print('generated rapid')
            self.insertMove(self.motion_controller.buffer_precharge)
            self.insertMove(rapid_to_start)

        elif mode == 'remote':
            #self.synchronizer.tp_first_trajectory_received_event.clear()
            self.motion_controller.motion_start_joint_positions = \
            self.cloud_trajectory_planner.extractStartingVoxelPoints()[0]
            hold_move = pncLibrary.Move(pncLibrary.TP.generateHoldPositionPoints(self.machine, 2), move_type='hold', offset_axes=False)
            #print('generated hold')
            self.motion_controller.buffer_precharge = hold_move
            if np.round(self.machine.current_stepgen_position - self.motion_controller.motion_start_joint_positions,3).any().item():
                self.synchronizer.tp_reposition_move_received_event.clear()
                self.createRapidTrajectoryPlanRequest(-10, *self.generateRapidPoints(end_joint=self.motion_controller.motion_start_joint_positions, mode='joint_space'))
                pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_trajectory_planner_waiting_for_rapid_string,
                                                         self.cloud_trajectory_planner.remote_tp_name)
                self.synchronizer.tp_reposition_move_received_event.wait()
                self.insertMove(self.motion_controller.buffer_precharge)
                self.insertMove(self.synchronizer.q_trajectory_planner_reposition_move_queue.get())
            else:
                self.insertMove(self.motion_controller.buffer_precharge)


    def beginPlanningTrajectory(self, number_of_sequences):
        if not self.cloud_trajectory_planner.tp_state.tp_connected_event.wait(self.machine.event_wait_timeout):
            print("TP connection timed out")

        self.cloud_trajectory_planner.plan_to_index_delta = number_of_sequences
        self.motion_controller.motion_queue_feeder.clearReceivedTrajectories()

        self.cloud_trajectory_planner.enqueueSequencesForPlanning(end_sequence=self.cloud_trajectory_planner.starting_sequence_id +
                                                                  number_of_sequences - 1 if number_of_sequences >= 1
                                                                  else len(self.cloud_trajectory_planner.contiguous_sequences))

        self.synchronizer.tp_plan_motion_event.set()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_trajectory_planner_waiting_for_first_move_string,
                                                         self.cloud_trajectory_planner.remote_tp_name)

        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, pncLibrary.printout_trajectory_initialization_string)


    def beginMotionExecution(self):
        if self.synchronizer.tp_first_trajectory_received_event.is_set():
            self.waitForSet(self.setMachineMode, 'auto', self.getMachineMode)
            self.synchronizer.mc_motion_complete_event.clear()
            self.cloud_trajectory_planner.tp_state.data_flushed_to_websocket_event.clear()

            self.initializeTrajectory()

            if not self.cloud_trajectory_planner.tp_state.all_moves_consumed_event.is_set():
                self.motion_controller.motion_queue_feeder.linkToTP()
            else:
                self.motion_controller.motion_queue_feeder.reenqueueTPMoves()

            self.motion_controller.motion_queue_feeder.startFeed()
            self.synchronizer.mc_run_motion_event.set()
        else:
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_machine_not_ready_for_motion_string)


    ######################## Writing Functions ########################
    def socketLockedWrite(self, data):
        with self.synchronizer.mc_socket_lock:
            self.machine.rsh_socket.send(data)

    def writeLineUTF(self,data):
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
                print('MACHINE CONTROLLER: Waiting for encoder initial position set')
                self.synchronizer.ei_encoder_init_event.wait()

            self.waitForSet(self.setCommMode, 1, self.getCommMode)
            self.waitForSet(self.setEMCTimeout, 0.01, self.getEMCTimeout)

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

    def getEMCTimeout(self, timeout = 0.5):
        self.writeLineUTF('get timeout')
        if self.synchronizer.fb_emc_timeout_change_event.wait(timeout):
            return (True, self.machine.emc_timeout)
        else:
            return (False, self.machine.emc_timeout)

    def getEMCReceiptMode(self):
        self.synchronizer.fb_emc_receipt_mode_change_event.clear()
        self.writeLineUTF('get set_wait')

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

    def setEMCTimeout(self, timeout):
        self.synchronizer.fb_emc_timeout_change_event.clear()
        self.writeLineUTF('set timeout ' + str(timeout))

    def setEMCReceiptMode(self, mode):
        self.synchronizer.fb_emc_receipt_mode_change_event.clear()
        self.writeLineUTF('set set_wait ' + str(mode))

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

    def setServoFeedbackMode(self, flag, log_type=0, sub_sample_rate=0, buffer_size=0, axes=0, dump_flag=0, write_buffer=0):
        if not sub_sample_rate:
            sub_sample_rate = self.machine.servo_log_sub_sample_rate
        if not buffer_size:
            buffer_size = self.machine.servo_log_buffer_size
        if not axes:
            axes = self.machine.servo_log_num_axes
        if not log_type:
            log_type = self.machine.servo_log_type

        self.synchronizer.fb_servo_logging_mode_change_event.clear()
        self.writeLineUTF('set servo_log_params ' + str(flag) + ' ' + str(log_type) + ' ' + str(axes) + ' ' + str(
            sub_sample_rate) + ' ' + str(buffer_size) + ' ' + str(dump_flag) + ' ' + str(write_buffer))
        # self.writeLineUTF('set servo_log_params ' + str(flag) + ' ' + str(axes) + ' ' + str(
        #     sub_sample_rate) + ' ' + str(buffer_size) + ' ' + str(dump_flag) + ' ' + str(write_buffer))

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

    def waitForSetConfirmation(self, set_function, set_params, get_function, timeout = None):
        if timeout is None:
            timeout = np.inf

        confirmation = None
        start_time = time.clock()

        while confirmation != set_params and (time.time() < timeout):
            set_function(set_params)
            confirmation = get_function()[1].lower()

        if (start_time-time.clock()) > timeout:
            raise pncLibrary.MachineControllerError('Timeout with ' + str(get_function))
        else:
            print('success: ' + str(get_function) + ' returned True')


    def checkMachineReady(self, timeout = 0.5):
        if self.pncLibrary.isAutoMode(self.machine, self.synchronizer) and self.getProgramStatus()[0] and self.machine.status == 'IDLE':
            return True
        else:
            print('Machine not ready: isAutoMode returned ' + str(pncLibrary.isAutoMode(self.machine, self.synchronizer)) + ' and machine status is ' + self.machine.status)
            return False

    def prepareMachineForDisconnect(self):
        return self.waitForSet(self.setEstop, 1, self.getEstop, 1)