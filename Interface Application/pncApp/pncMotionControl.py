import pncLibrary, struct, time, numpy as np
from multiprocessing import Queue, Event, current_process
from threading import Thread
from queue import Empty

class MotionController(Thread):
    def __init__(self, parent):
        super(MotionController, self).__init__()
        #self.parent = current_thread()
        self.parent = parent
        self.name = "motion_controller"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer

        self.move_queue = Queue()
        self.motion_queue = Queue()
        self.interrupt_motion_event = Event()

        self.motion_start_joint_positions = np.array([0,0,0,0,0])
        self.move_in_progress = 0
        self.last_move_serial_number = 0
        self.current_move_serial_number = 0
        self.current_move_subserial_number = 0
        self.current_motion_block_serial_number = 0

        ### Network control parameters
        self.polylines = self.machine.polylines_per_tx
        self.blocklength = self.machine.points_per_polyline
        self.prebuffer_size = self.machine.motion_control_data_prebuffer_size
        self.object_prebuffer = []
        self.array_prebuffer = []

        self.buffer_precharge = None
        self.initial_position_rapid_move = None

        self.startup_event = Event()

    def run(self):
        #pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.thread_launch_string, current_process().name, self.name)
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_subthread_launch_string, self.parent.name,
                                                     self.name)
        pncLibrary.waitForThreadStart(self, MotionQueueFeeder)
        self.startup_event.set()

        while self.synchronizer.t_run_motion_controller_event.is_set():
            if self.synchronizer.mc_run_motion_event.wait(self.machine.event_wait_timeout):
                self.motion_queue_feeder.enqueue_blocks_event.set()
                #self.machine.motion_controller_clock_offset = time.time()
                #There are new moves in the queue, find the one to be executed next
                while not self.synchronizer.mc_rsh_error_event.is_set():
                    try:
                        motion_block_to_execute = self.motion_queue.get(True, pncLibrary.queue_wait_timeout)
                        self.synchronizer.mc_motion_complete_event.clear()
                        self.machine.motion_start_time = time.time() - self.machine.pncApp_clock_offset

                        # if not self.synchronizer.mc_motion_started_event.is_set():
                        #     #self.machine.motion_start_time = time.time()-self.machine.pncApp_clock_offset
                        #     self.synchronizer.mc_motion_started_event.set()

                        self.current_move_serial_number = motion_block_to_execute.serial_number
                        self.current_move_subserial_number = motion_block_to_execute.subserial_number

                        self.current_motion_block_serial_number += 1
                        motion_block_to_execute.motion_block_serial_number = self.current_motion_block_serial_number

                        self.synchronizer.q_database_command_queue_proxy.put(
                            pncLibrary.DatabaseCommand('push_object', [{"EXECUTED_MOTION_BLOCKS": motion_block_to_execute}]))

                        try:
                            print('buffer level at start of move subserial ' + str(self.current_move_subserial_number) + ' is ' + str(self.machine.current_buffer_level))
                            self.commandPoints(motion_block_to_execute.servo_tx_array, self.polylines, self.blocklength)
                        except pncLibrary.RSHError as error_message:
                            self.synchronizer.q_print_server_message_queue.put(str(error_message))
                            #self.motion_queue = Queue()
                            self.emptyMotionQueue()
                            self.interrupt_motion_event.clear()
                            self.synchronizer.mc_run_motion_event.clear()
                            break

                        self.prebufferAndFlushMotionRecords()

                        self.last_move_serial_number = motion_block_to_execute.serial_number
                    except Empty:
                        if self.machine.current_buffer_level == 0:
                            time.sleep(self.machine.servo_dt*self.machine.max_buffer_level)
                            self.synchronizer.mc_motion_complete_event.set()
                            self.synchronizer.mc_run_motion_event.clear()
                            print('MOTION CONTROLLER: Motion complete, waiting for next set of motion blocks')
                            break

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

            if self.interrupt_motion_event.is_set():
                raise pncLibrary.RSHError("MOTION CONTROLLER: Detected RSH error after " + str(command) + " commands", command)
                #return

            tx_time = time.time()
            pncLibrary.socketLockedWrite(self.machine, self.synchronizer, binary_command)

            #FIXME check buffer was flushed
            current_BL = self.machine.current_buffer_level
            sleep_time = self.runNetworkPID(int(current_BL), blocklength, polylines, self.machine.buffer_level_setpoint)
            self.prebufferAndFlushMotionRecords(objects={'COMMANDED_SERVO_POLYLINE_OBJECTS': commanded_points},
                                                arrays={'NETWORK_PID_DELAYS': np.array([[sleep_time]]),
                                                  'POLYLINE_TRANSMISSION_TIMES': np.array([[tx_time-self.machine.pncApp_clock_offset]]),
                                                  'INTERPOLATED_POLYLINE_TRANSMISSION_TIMES': np.array([[tx_time-self.machine.pncApp_clock_offset]]) + np.array([np.arange(0,polylines*blocklength)/1e3]).T,
                                                  'COMMANDED_SERVO_POSITIONS': servo_points[command],
                                                  'NETWORK_PID_BUFFER_LEVEL': np.array([current_BL])})

            time.sleep(sleep_time)

    def runNetworkPID(self, current_buffer_level, block_length, poly_lines, set_point_buffer_level, Kp=.05, Ki=0, Kd=0):
        if (self.machine.max_buffer_level - current_buffer_level) < 100:
            print('WARNING: Buffer fidna overflow')
        #sleep_time = max((block_length * polylines) / 1000 - (Kp * ((set_point_buffer_level - current_buffer_level))) / 1000,0)
        sleep_time = max((block_length * poly_lines) / 1000 - (Kp * ((set_point_buffer_level - current_buffer_level))) / 1000, 0)
        return float(sleep_time)

    def prebufferAndFlushMotionRecords(self, objects = None, arrays = None):
        if objects is None and arrays is None:
            if len(self.object_prebuffer) > 0:
                self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push_object', self.object_prebuffer))
                self.object_prebuffer = []
            if len(self.array_prebuffer) > 0:
                self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push', self.array_prebuffer))
                self.array_prebuffer = []
        else:
            if len(self.object_prebuffer) >= self.prebuffer_size:
                self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push_object', self.object_prebuffer))
                self.object_prebuffer = [objects]
            else:
                self.object_prebuffer.append(objects)
            if len(self.array_prebuffer) >= self.prebuffer_size:
                self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push', self.array_prebuffer))
                self.array_prebuffer = [arrays]
            else:
                self.array_prebuffer.append(arrays)

    def adaptNetworkTxGain(self):
        pass

    def emptyMotionQueue(self):
        print('MOTION CONTROLLER: Dumping motion queue')
        while not self.motion_queue.empty():
            block = self.motion_queue.get(True, pncLibrary.queue_wait_timeout)

class MotionQueueFeeder(Thread):
    def __init__(self, parent):
        super(MotionQueueFeeder, self).__init__()
        self.parent = parent
        self.name = "motion_queue_feeder"
        self.synchronizer = parent.synchronizer

        self.max_motion_block_size = self.parent.machine.max_motion_block_size

        self.received_moves = []
        self.move_queue = parent.move_queue
        self.motion_queue = parent.motion_queue
        self.startup_event = Event()
        self.tp_link_event = Event()
        self.enqueue_blocks_event = Event()

        self.processed_move_serial_number = 0
        self.shifted_motion_start_points = None

    def run(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_subthread_launch_string, self.parent.name,
                                                     self.name)
        self.startup_event.set()

        while self.synchronizer.t_run_motion_queue_feeder_event.is_set():
            print('motion queue size is ' + str(self.parent.motion_queue.qsize()))
            print('move queue size is ' + str(self.parent.move_queue.qsize()))
            if self.enqueue_blocks_event.wait(pncLibrary.event_wait_timeout):
                self.updateTrajectoryFromTP()

                try:
                    move_to_execute = self.move_queue.get(True, pncLibrary.queue_move_queue_wait_timeout)
                    self.processed_move_serial_number += 1
                    move_to_execute.serial_number = self.processed_move_serial_number
                    samples_in_block = self.max_motion_block_size/move_to_execute.blocklength

                    if samples_in_block != int(samples_in_block):
                        print('division problem')

                    samples_in_block = int(samples_in_block)

                    self.synchronizer.q_database_command_queue_proxy.put(
                        pncLibrary.DatabaseCommand('push_object', [{"PROCESSED_MOVES": move_to_execute}]))

                    for motion_packet_start_index in range(0, np.shape(move_to_execute.servo_tx_array)[0], samples_in_block):
                        self.insertMotionBlock(move_to_execute, (motion_packet_start_index, motion_packet_start_index+samples_in_block), int(motion_packet_start_index/samples_in_block+1))
                except Empty:
                    if self.synchronizer.tp_planning_finished_event.is_set():
                        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, pncLibrary.printout_motion_queue_feeder_pausing_string)
                        self.unlinkFromTP()
                        self.stopFeed()
                    pass

        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.subthread_terminate_string, self.parent.name,
                                                     self.name)

    def insertMotionBlock(self, original_move, indices, motion_block_id):
        point_samples_flat = original_move.servo_tx_array.reshape((np.shape(original_move.servo_tx_array)[0] * np.shape(original_move.servo_tx_array)[1], -1))
        try:
            motion_block = pncLibrary.Move(point_samples_flat[original_move.blocklength*indices[0]:original_move.blocklength*indices[1]], machine=self.parent.machine, move_type='block', offset_axes=True)
        except Exception as error:
            print('break motion')
        motion_block.servo_tx_array = pncLibrary.TP.offsetAxes(original_move.servo_tx_array[indices[0]:indices[1]], machine=self.parent.machine)
        motion_block.polylines = original_move.polylines
        motion_block.blocklength = original_move.blocklength
        motion_block.serial_number = original_move.serial_number
        motion_block.subserial_number = motion_block_id
        motion_block.sequence_id = original_move.sequence_id

        self.motion_queue.put(motion_block)

    def updateTrajectoryFromTP(self):
        while not self.synchronizer.q_trajectory_planner_planned_move_queue.empty() and self.tp_link_event.is_set():
            remotely_planned_move = self.synchronizer.q_trajectory_planner_planned_move_queue.get_nowait()
            #print('TP putting move on motion queue')
            self.parent.parent.insertMove(remotely_planned_move)
            self.received_moves.append(remotely_planned_move)

    def linkToTP(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_trajectory_planner_motion_queues_linked_string,
                                                     self.parent.parent.cloud_trajectory_planner.name)
        self.tp_link_event.set()

    def unlinkFromTP(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_trajectory_planner_motion_queues_unlinked_string,
                                                     self.parent.parent.cloud_trajectory_planner.name)
        self.tp_link_event.clear()

    def startFeed(self):
        self.enqueue_blocks_event.set()

    def stopFeed(self):
        self.enqueue_blocks_event.clear()

    def reenqueueTPMoves(self):
        for move in self.received_moves:
            self.parent.parent.insertMove(move)

    def clearReceivedTrajectories(self):
        self.synchronizer.tp_first_trajectory_received_event.clear()
        self.received_moves = []
