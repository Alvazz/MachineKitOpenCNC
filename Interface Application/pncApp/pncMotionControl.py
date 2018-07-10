import pncLibrary, struct, time, numpy as np
from multiprocessing import Queue, Event, current_process
from threading import Thread

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

        pncLibrary.waitForThreadStart(self, MotionQueueFeeder)

        self.startup_event.set()

        while self.synchronizer.t_run_motion_controller_event.is_set():
            if self.synchronizer.mc_run_motion_event.wait(self.machine.thread_queue_wait_timeout):

                #There are new moves in the queue, find the one to be executed next
                while not self.move_queue.empty() and not self.synchronizer.mc_rsh_error_event.is_set():
                    move_to_execute = self.move_queue.get()
                    self.current_move_serial_number += 1
                    move_to_execute.serial_number = self.current_move_serial_number

                    self.synchronizer.q_database_command_queue_proxy.put(
                        pncLibrary.DatabaseCommand('push_object', [{"EXECUTED_MOVES": move_to_execute}]))

                    try:
                        print('buffer level at start is ' + str(self.machine.current_buffer_level))
                        self.commandPoints(move_to_execute.servo_tx_array, self.polylines, self.blocklength)
                    except pncLibrary.RSHError:
                        self.synchronizer.q_print_server_message_queue.put("MOTION CONTROLLER: Detected RSH error, aborting motion")
                        self.move_queue = Queue()
                        break

                    self.last_move_serial_number = move_to_execute.serial_number


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

            if self.synchronizer.mc_rsh_error_event.is_set():
                raise pncLibrary.RSHError("Detected RSH error after " + str(command) + " commands", command)
                #return

            tx_time = time.time()
            pncLibrary.socketLockedWrite(self.machine, self.synchronizer, binary_command)

            #FIXME check buffer was flushed
            current_BL = self.machine.current_buffer_level
            sleep_time = self.runNetworkPID(int(current_BL), blocklength, polylines, self.machine.buffer_level_setpoint)
            self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push_object', [{'COMMANDED_SERVO_POLYLINES': commanded_points}]))
            self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push', [{'NETWORK_PID_DELAYS': np.array([[sleep_time]]),
                                                                                                      'POLYLINE_TRANSMISSION_TIMES': np.array([[tx_time-self.machine.pncApp_clock_offset]]),
                                                                                                      'COMMANDED_POSITIONS': servo_points[command], 'NETWORK_PID_BUFFER_LEVEL': np.array([current_BL])}]))
            time.sleep(sleep_time)

    def runNetworkPID(self, current_buffer_level, block_length, poly_lines, set_point_buffer_level, Kp=.05, Ki=0, Kd=0):
        if (self.machine.max_buffer_level - current_buffer_level) < 100:
            print('WARNING: Buffer finna overflow')
        #sleep_time = max((block_length * polylines) / 1000 - (Kp * ((set_point_buffer_level - current_buffer_level))) / 1000,0)
        sleep_time = max((block_length * poly_lines) / 1000 - (Kp * ((set_point_buffer_level - current_buffer_level))) / 1000, 0)
        return float(sleep_time)

    def adaptNetworkTxGain(self):
        pass


class MotionQueueFeeder(Thread):
    def __init__(self, parent):
        super(MotionQueueFeeder, self).__init__()
        self.parent = parent
        self.name = "motion_queue_feeder"
        self.synchronizer = parent.synchronizer

        self.move_queue = parent.move_queue
        self.motion_queue = Queue()
        self.startup_event = Event()

    def run(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_subthread_launch_string, self.parent.name,
                                                     self.name)
        self.startup_event.set()

        while self.synchronizer.t_run_motion_queue_feeder_thread.is_set():
            move_to_execute = self.move_queue.get()
            
            return

        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.subthread_terminate_string, self.parent.name,
                                                     self.name)