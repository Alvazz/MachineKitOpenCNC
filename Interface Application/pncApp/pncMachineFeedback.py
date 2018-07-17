import pncLibrary
from threading import Thread
from multiprocessing import Process, Queue, Event, current_process
from threading import current_thread
import socket, time, select, datetime, struct, os, numpy as np

#global machine

class FeedbackState():
    def __init__(self, machine):
        # Feedback State
        self.byte_string = bytearray()
        self.binary_transmission_length = pncLibrary.calculateBinaryTransmissionLength(machine)
        self.incoming_transmission_length = None
        self.feedback_encoding = None
        self.feedback_type = None
        self.header_delimiter_index = None

        self.transmission_received = False
        self.terminator_received = False
        self.header_processed = False
        self.header_processing_error = False
        self.feedback_data_processed = False
        self.feedback_data_processing_error = False
        self.rsh_error_check = False

class FeedbackData():
    def __init__(self):
        self.data = None

class FeedbackProcessor(Thread):
    def __init__(self, parent):
        super(FeedbackProcessor, self).__init__()
        self.name = "feedback_processor"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer
        self.queue_wait_timeout = self.machine.thread_queue_wait_timeout

        self.process_queue = Queue()
        #self.processed_queue = Queue()
        self.startup_event = Event()

    def run(self):
        self.startup_event.set()
        #pncLibrary.printTerminalString(self.machine.thread_launch_string, current_process().name, self.name)
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_launch_string, current_process().name,
                                                     self.name)
        while self.synchronizer.t_run_feedback_processor_event.is_set():
            #if not self.process_queue.empty():
            try:
                raw_data = self.process_queue.get(True, self.queue_wait_timeout)
                self.processFeedbackData(*raw_data)
            except:
                pass

        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_terminate_string, current_process().name,
                                                     self.name)
        #FIXME flush the rest to DB before shutdown
        #pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.thread_launch_string, current_process().name, self.name)
        # while True:
        #     #FIXME handle rx_received_time accurately here
        #     pass

    def handleRSHError(self):
        self.synchronizer.mc_rsh_error_event.set()
        self.synchronizer.q_print_server_message_queue.put("FEEDBACK HANDLER: RSH error detected by feedback_processor")
        print('flag set')

    def processFeedbackData(self, feedback_encoding, feedback_type, feedback_data, rx_received_time, transmission_length = -1):
        if feedback_type.upper() == 'COMMAND ECHO':
            if self.machine.ascii_rsh_feedback_strings[-1] in feedback_data:
                print('RSH error')
                self.handleRSHError()
            else:
                print('got echo data')
            return

        if feedback_encoding == 'ascii':
            feedback_data = list(map(str.upper,feedback_data))
            if self.machine.ascii_rsh_feedback_strings[0] == feedback_type:
                #Position feedback
                if self.machine.servo_feedback_mode != 1:
                    print('state machine sync error')
                self.processPositionFeedback('ascii', rx_received_time, feedback_data)
            elif self.machine.ascii_rsh_feedback_strings[1] == feedback_type:
                # Buffer Length
                self.processBufferLevelFeedback('ascii', rx_received_time, feedback_data)
            elif self.machine.ascii_rsh_feedback_strings[2] == feedback_type:
                # Program Status
                # print('program status')
                self.machine.status = feedback_data
                # print('set program status to ' + self.machine.status)
                self.synchronizer.fb_status_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[3] == feedback_type:
                # Machine Mode
                # print('mode set')
                #print('got mode ' + feedback_data[0])
                self.machine.mode = feedback_data[0]
                self.synchronizer.fb_mode_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[4] == feedback_type:
                # Servo Log Params
                print('got logging ' + str(feedback_data))
                self.machine.servo_feedback_mode = int(feedback_data[0])
                self.synchronizer.fb_servo_logging_mode_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[5] == feedback_type:
                # Drive Power
                #self.machine.drive_power = self.machine.rsh_feedback_flags.index(feedback_data.upper())
                self.machine.drive_power = pncLibrary.checkOnOff(self.machine, feedback_data[0])
                self.synchronizer.fb_drive_power_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[6] == feedback_type:
                # Echo
                # print('got set echo: ' + data_string)
                self.machine.echo = pncLibrary.checkOnOff(self.machine, feedback_data[0])
                self.synchronizer.fb_echo_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[7] == feedback_type:
                # Hello
                print('got hello')
                self.machine.connected = True
                self.synchronizer.fb_connection_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[8] == feedback_type:
                # Enable
                # print('got enable')
                #self.machine.linked = self.machine.rsh_feedback_flags.index(feedback_data[0].upper())
                self.machine.linked = pncLibrary.checkOnOff(self.machine, feedback_data[0])
                self.synchronizer.fb_link_change_event.set()
                self.synchronizer.mvc_connected_event.set()
            elif self.machine.ascii_rsh_feedback_strings[9] == feedback_type:
                # EStop
                # print('got estop: ' + data_string)
                #self.machine.estop = self.machine.rsh_feedback_flags.index(feedback_data.upper())
                self.machine.estop = pncLibrary.checkOnOff(self.machine, feedback_data[0])
                self.synchronizer.fb_estop_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[10] == feedback_type:
                # Joint homed
                # print('got joint home: ' + data_string)
                home_state = [0]*self.machine.number_of_joints
                for axis in range(0, self.machine.number_of_joints):
                    home_state[axis] = pncLibrary.checkOnOff(self.machine,feedback_data[axis])
                self.machine.axis_home_state = home_state
                if all(self.machine.axis_home_state):
                    print('all joints homed')
                    self.synchronizer.fb_all_homed_event.set()
                self.synchronizer.fb_home_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[11] == feedback_type:
                # PING
                self.machine.ping_rx_time = rx_received_time
                self.machine.current_estimated_network_latency = (self.machine.ping_rx_time - self.machine.ping_tx_time) / 2
                self.synchronizer.fb_ping_event.set()
            elif self.machine.ascii_rsh_feedback_strings[12] == feedback_type:
                # Time
                self.machine.last_unix_time = float(feedback_data[0])
                self.machine.clock_sync_received_time = rx_received_time
                self.synchronizer.fb_clock_event.set()
            elif self.machine.ascii_rsh_feedback_strings[13] == feedback_type:
                # Comm mode
                print('got comm_mode')
                self.machine.comm_mode = pncLibrary.checkOnOff(self.machine, feedback_data[0])
                self.synchronizer.fb_comm_mode_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[14] == feedback_type:
                # BL feedback mode
                #print('got BL feedback params')
                self.machine.buffer_level_feedback_mode = pncLibrary.checkOnOff(self.machine, feedback_data[0])
                self.machine.buffer_level_feedback_period = int(feedback_data[1])
                self.synchronizer.fb_buffer_level_feedback_mode_change_event.set()
            else:
                print('received unrecognized ascii string for header ' + str(feedback_type) + 'with data ' + str(feedback_data))
        elif feedback_encoding == 'binary':
            if any(s in feedback_type for s in self.machine.binary_rsh_feedback_strings):
                if self.machine.binary_rsh_feedback_strings[0] == feedback_type:
                    feedback_data = struct.unpack('!' + 'd' * int((transmission_length - len(
                        self.machine.binary_line_terminator)) / self.machine.size_of_feedback_double), feedback_data)
                    self.processPositionFeedback('binary',rx_received_time,feedback_data)
                    self.machine.servo_feedback_reception_event.set()
                elif self.machine.binary_rsh_feedback_strings[1] == feedback_type:
                    #FIXME unpack binary here
                    self.processBufferLevelFeedback('binary', rx_received_time, feedback_data)
                    self.machine.buffer_level_reception_event.set()
            else:
                pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, 'received unrecognized binary string for header ' + str(feedback_type) + 'with data ' + str(feedback_data))

    def processBufferLevelFeedback(self, feedback_encoding, rx_received_time, feedback_data):
        #print('got buffer_level feedback')
        rx_received_time = rx_received_time - self.machine.pncApp_clock_offset
        if feedback_encoding == 'ascii':
            rsh_clock_time = float(feedback_data[0])
            rsh_buffer_level = int(feedback_data[1])
        elif feedback_encoding == 'binary':
            feedback_data = struct.unpack('!dI', feedback_data)
            rsh_clock_time = feedback_data[0]
            rsh_buffer_level = feedback_data[1]

        record = dict()
        record['HIGHRES_TC_QUEUE_LENGTH'] = np.array([[rsh_buffer_level]])
        #print('buffer level is ' + str(rsh_buffer_level))
        record['HIGHFREQ_ETHERNET_RECEIVED_TIMES'] = np.array([[rx_received_time-self.machine.pncApp_clock_offset]])

        record['RSH_CLOCK_TIMES'] = np.array(
            [[(rsh_clock_time + (self.machine.RT_clock_offset - self.machine.OS_clock_offset) - self.machine.RT_clock_offset)/self.machine.clock_resolution]])

        if self.synchronizer.mc_xenomai_clock_sync_event.is_set():
            self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push', [record]))
            self.machine.current_buffer_level = np.array([rsh_buffer_level])

    def processPositionFeedback(self, feedback_encoding, rx_received_time, feedback_data):
        #rx_received_time = rx_received_time - self.machine.pncApp_clock_offset
        try:
            if feedback_encoding == 'ascii':
                number_of_feedback_points = len(feedback_data)
                #Check end line terminator
                if not feedback_data[-1][-1] == self.machine.ascii_servo_feedback_terminator or len(feedback_data[-1].split(self.machine.ascii_servo_feedback_terminator)) < self.machine.servo_log_num_axes:
                    print('problem with ascii line terminator in servo feedback, dropping last entry')
                    feedback_data = feedback_data[:-1]

                feedback_data = [s.strip(self.machine.ascii_servo_feedback_terminator) for s in feedback_data]

                stepgen_feedback_positions = np.zeros([number_of_feedback_points,self.machine.servo_log_num_axes])
                RTAPI_clock_times = np.zeros([number_of_feedback_points,1])
                received_times_interpolated = np.zeros([number_of_feedback_points, 1])
                RTAPI_feedback_indices = np.zeros([number_of_feedback_points, 1])
                lowfreq_ethernet_received_times = np.zeros([number_of_feedback_points, 1]) + rx_received_time

                for sample_number in range(0,number_of_feedback_points):
                    sample_point = feedback_data[sample_number].split(self.machine.ascii_servo_feedback_terminator)
                    RTAPI_feedback_indices[sample_number, :] += float(sample_number)
                    RTAPI_clock_times[sample_number,:] = float(sample_point[0])
                    stepgen_feedback_positions[sample_number,:] = np.asarray([float(s) for s in sample_point[1:]])+np.asarray(self.machine.axis_offsets)
                    ## FIXME use clock delta between samples instead of straight interpolation
                    received_times_interpolated[sample_number, :] = rx_received_time + self.machine.servo_dt*self.machine.servo_log_sub_sample_rate * float(sample_number)
                    #self.machine.current_stepgen_position = stepgen_feedback_positions[sample_number,:]

            elif feedback_encoding == 'binary':
                number_of_feedback_points = int(len(feedback_data) / (self.machine.servo_log_num_axes + 1))
                step_size = self.machine.servo_log_num_axes + 1

                RTAPI_feedback_indices = np.zeros([number_of_feedback_points, 1])
                RTAPI_clock_times = np.zeros([number_of_feedback_points, 1])
                stepgen_feedback_positions = np.zeros([number_of_feedback_points, self.machine.servo_log_num_axes])
                received_times_interpolated = np.zeros([number_of_feedback_points, 1])
                lowfreq_ethernet_received_times = np.zeros([number_of_feedback_points, 1]) + rx_received_time

                for sample_number in range(0,number_of_feedback_points):
                    sample_index = sample_number*step_size
                    sample_point = feedback_data[sample_index:sample_index + step_size]

                    RTAPI_feedback_indices[sample_number, :] += float(sample_number)
                    RTAPI_clock_times[sample_number, :] = float(sample_point[0])
                    stepgen_feedback_positions[sample_number, :] = np.asarray([round(float(s), 6) for s in sample_point[1:]])
                    received_times_interpolated[sample_number, :] = rx_received_time + self.machine.servo_dt * self.machine.servo_log_sub_sample_rate * float(sample_number)
            else:
                return

            record = dict()
            record['RTAPI_feedback_indices'] = RTAPI_feedback_indices
            record['RTAPI_clock_times'] = (RTAPI_clock_times-self.machine.RT_clock_offset)/self.machine.clock_resolution
            record['stepgen_feedback_positions'] = pncLibrary.TP.convertMotionCS(self.machine, 'absolute', stepgen_feedback_positions)
            if type(stepgen_feedback_positions[0]) == int:
                print('break')
            record['lowfreq_ethernet_received_times'] = lowfreq_ethernet_received_times

            if self.synchronizer.fb_feedback_data_initialized_event.is_set():
                if not self.synchronizer.mc_xenomai_clock_sync_event.is_set():
                    self.machine.RT_clock_offset = RTAPI_clock_times[0].item()
                    self.machine.RT_clock_sync_pncApp_time = rx_received_time-(RTAPI_clock_times[-1].item()-RTAPI_clock_times[0].item())/self.machine.clock_resolution
                    self.synchronizer.mc_xenomai_clock_sync_event.set()
                    return
                self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push', [record]))
            else:
                # Throw away first data point because thread counter in RSH should start from thread_periods % servo_log_flush_samples = 0
                self.synchronizer.fb_feedback_data_initialized_event.set()

        except Exception as error:
            print('had error processing feedback string. The data string is: ' + str(feedback_data))
            print('the error is ' + str(error))
            return

class MachineFeedbackHandler(Process):
    #global machine
    def __init__(self, machine, pipe):
        super(MachineFeedbackHandler, self).__init__()
        self.name = "feedback_handler"
        self.main_thread_name = self.name + ".MainThread"

        # Args
        self.machine = machine
        self.feed_pipe = pipe
        #self.synchronizer = synchronizer

        self.feedback_state = pncLibrary.InternalFeedbackState(self.machine)

        # Feedback State
        self.byte_string = bytearray()
        self.binary_transmission_length = pncLibrary.calculateBinaryTransmissionLength(machine)
        self.incoming_transmission_length = None
        self.feedback_encoding = None
        self.feedback_type = None
        self.header_delimiter_index = None
        self.socket_passes = 0

        # Flags
        self.transmission_received = False
        self.terminator_received = False
        self.header_processed = False
        self.header_processing_error = False
        self.feedback_data_processed = False
        self.feedback_data_processing_error = False
        self.rsh_error_check = False

    def run(self):
        current_thread().name = self.name + '.MainThread'
        pncLibrary.getSynchronizer(self, self.feed_pipe)

        #self.waitForThreadLaunch()
        pncLibrary.waitForThreadStart(self, FeedbackProcessor)
        self.synchronizer.fb_startup_event.set()

        self.synchronizer.process_start_signal.wait()
        time.clock()

        #if self.synchronizer.p_enable_feedback_handler_event.is_set():
        self.synchronizer.fb_successful_start_event.set()
        self.synchronizer.p_run_feedback_handler_event.wait()
        while self.synchronizer.p_run_feedback_handler_event.is_set():
            # Process feedback from EMCRSH
            #FIXME store state in FeedbackState object
            #FIXME ensure that the socket actually exists
            if select.select([self.machine.rsh_socket], [], [], 0)[0]:
                #Clock when these data were received
                rx_received_time = time.time()
                #First parse the header
                bytes_received = self.machine.rsh_socket.recv(self.machine.bytes_to_receive)
                self.byte_string.extend(bytes_received)
                #print('byte_string length is now ' + str(len(self.byte_string)))
                self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('log', bytes_received, None, rx_received_time))

            if len(self.byte_string) >= self.machine.minimum_header_length and not self.header_processed:
                self.header_processed, self.header_processing_error, self.feedback_encoding, self.feedback_type, self.header_delimiter_index, self.rx_received_time = self.assembleFeedbackHeader(self.byte_string, rx_received_time)

            #FIXME should this handle multiple transmissions in one loop?
            if self.header_processed and not self.header_processing_error:
                #Drop header data, it's already good
                old_byte_string_before_header = self.byte_string
                if not self.feedback_data_processing_error:
                    #If this is the first pass on the data, FIXME can move this into assenbleAndProcess, but is it worth it?
                    self.byte_string = self.byte_string[self.header_delimiter_index:]
                else:
                    print('passing again on socket')
                self.feedback_data_processed, self.feedback_data_processing_error, self.feedback_data, self.complete_transmission_delimiter_index = \
                    self.assembleAndProcessFeedbackData(self.feedback_encoding, self.feedback_type, self.byte_string, self.rx_received_time)

                if self.feedback_data_processed and not self.feedback_data_processing_error:
                    self.byte_string = self.byte_string[self.complete_transmission_delimiter_index:]
                    #FIXME use feedbackstate object instead of individual flags
                    self.resetFeedbackState()
                    self.header_processed = False
                    self.header_processing_error = False
                    self.feedback_data_processed = False
                    self.feedback_data_processing_error = False
                    # self.last_processed_byte_string = self.byte_string

        #Flag set, shutdown. Handle socket closure in machine_controller
        pncLibrary.waitForThreadStop(self, self.feedback_processor)

    def resetFeedbackState(self):
        self.header_processed = False
        self.header_processing_error = False
        self.feedback_data_processed = False
        self.feedback_data_processing_error = False
        self.last_processed_byte_string = self.byte_string


    def assembleFeedbackHeader(self, byte_string, rx_received_time):
        #Return data as header processed flag, error flag, encoding, feedback type, header delimiter index
        header_delimiter_index = byte_string.index(self.machine.ascii_header_delimiter_bytes)

        header_bytes = byte_string[:header_delimiter_index]
        try:
            header_string = header_bytes.decode('utf-8').upper()
        except Exception as error:
            print('MACHINE FEEDBACK LISTENER: Had error processing feedback header: ' + str(error))
            return False, True, None, None, header_delimiter_index

        #print('got header data ' + header_string)
        #First check for RSH error
        if self.machine.rsh_error_string.encode('utf-8') in byte_string:
            #Byte string matches, flag possible error
            #FIXME implement this!
            print('potential RSH error detected')
            self.rsh_error_check = True

        #FIXME don't make feedback_type an index
        #Check for binary/ascii/ascii echo
        if any(s in header_string for s in self.machine.ascii_rsh_feedback_strings):
            feedback_encoding = 'ascii'
            if header_string != '*':
                print(header_string)
            #feedback_type = self.machine.ascii_rsh_feedback_strings.index([s for s in self.machine.ascii_rsh_feedback_strings if header_string in s][0])
            feedback_type = header_string.upper()
        elif any(s in header_string for s in self.machine.binary_rsh_feedback_strings):
            feedback_encoding = 'binary'
            #feedback_type = self.machine.binary_rsh_feedback_strings.index([s for s in self.machine.binary_rsh_feedback_strings if header_string in s][0])
            feedback_type = header_string.upper()
            header_delimiter_index += pncLibrary.countTerminatorsToGobble(byte_string[header_delimiter_index:],self.machine.binary_header_delimiter)*len(self.machine.binary_header_delimiter)
        elif any(s in header_string for s in self.machine.rsh_echo_strings):
            feedback_encoding = 'ascii'
            feedback_type = 'command echo'
        else:
            errmsg = 'FEEDBACK HANDLER: Can\'t find header string \"' + header_string + '\"'
            print(errmsg)
            self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('log',errmsg))
            return False, True, None, None, None, None

            # if self.rsh_error_check:
            #     print('definite error')

        return True, False, feedback_encoding, feedback_type, header_delimiter_index, rx_received_time

    def assembleAndProcessFeedbackData(self, feedback_encoding, feedback_type, byte_string, rx_received_time):
        # Now split the command at the delimiter depending on the type
        #if feedback_type.upper() == 'ECHO':
        if feedback_encoding == 'ascii':
            line_terminator = self.machine.ascii_line_terminator.encode('utf-8')
            # Check length of transmission for a complete command
            if line_terminator in byte_string:
                # We have enough data to form a complete transmission
                line_terminator_index = byte_string.index(self.machine.ascii_line_terminator.encode('utf-8'))
                feedback_data = byte_string[:line_terminator_index].decode('utf-8').split()
                # FIXME send to feedback_processor thread
                #self.processFeedbackData(feedback_encoding, feedback_type, feedback_data, rx_received_time)
                self.feedback_processor.process_queue.put((feedback_encoding, feedback_type, feedback_data, rx_received_time))

                # Drop this command from the buffer and gobble remaining terminators
                complete_transmission_delimiter_index = line_terminator_index + pncLibrary.countTerminatorsToGobble(byte_string[line_terminator_index:], line_terminator)*len(line_terminator)
                return True, False, feedback_data, complete_transmission_delimiter_index
            else:
                #No line terminator, go for another pass
                print('making another pass on the socket')
                return False, False, None, None

        elif feedback_encoding == 'binary':
            #The first four bytes give transmission length
            if len(byte_string) >= self.machine.size_of_feedback_int:
                transmission_length = struct.unpack('!I',byte_string[:self.machine.size_of_feedback_int])[0]
                self.old_transmission_length = self.incoming_transmission_length
                if not self.feedback_state.multiple_socket_passes_required:
                    self.incoming_transmission_length = transmission_length
                    byte_string = byte_string[self.machine.size_of_feedback_int:]
                if len(byte_string) >= (self.incoming_transmission_length - self.machine.size_of_feedback_int):
                #if len(byte_string) >= transmission_length:
                    if transmission_length <= len(self.machine.binary_line_terminator):
                        print('break')
                    #Now we have a full transmission and can process it
                    #byte_string = byte_string[:transmission_length+len(self.machine.binary_line_terminator)]
                    #byte_string = byte_string[:transmission_length]
                    byte_string = byte_string[:self.incoming_transmission_length]
                    #Check for complete transmission with terminator
                    try:
                        if byte_string[-len(self.machine.binary_line_terminator)] == ord(self.machine.binary_line_terminator):
                            feedback_data = byte_string[:-len(self.machine.binary_line_terminator)]

                            self.feedback_processor.process_queue.put((feedback_encoding, feedback_type, feedback_data, rx_received_time, transmission_length))
                            complete_transmission_delimiter_index = self.machine.size_of_feedback_int+transmission_length
                            self.socket_passes = 0
                            self.feedback_state.multiple_socket_passes_required = False
                            return True, False, feedback_data, complete_transmission_delimiter_index
                        else:
                            print('binary feedback had incorrect terminator')
                            return True, True, None, None
                    except Exception as error:
                        print('had error: ' + str(error))
                else:
                    #Wait for complete transmission
                    print('waiting for complete transmission')
                    self.feedback_state.multiple_socket_passes_required = True
                    self.socket_passes += 1
                    self.last_byte_string = byte_string
                    if self.socket_passes >= 6:
                        print('socket pass break')
                    return False, False, None, None
            else:
                # Wait for transmission length integer
                print('waiting for transmission length')
                self.feedback_state.multiple_socket_passes_required = True
                return False, False, None, None

    # def handleRSHError(self):
    #     self.machine.rsh_error = 1
    #
    # def close(self):
    #     self._running = True
    #     self._running = False

