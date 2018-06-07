from multiprocessing import Process
import socket, time, select, datetime, struct, numpy as np
import pncLibrary

#global machine

class MachineFeedbackListener(Process):
    #global machine
    def __init__(self, machine, database_push_queue_proxy):
        super(MachineFeedbackListener, self).__init__()
        self.machine = machine
        self.database_push_queue = database_push_queue_proxy

        #self._shutdown = False


        #self.machine_controller = machine_controller
        #self.rsh_socket = self.machine.machine_controller_thread_handle.rsh_socket
        #self.data_store = data_store


        print('Feedback thread started')
        #self.log_file_handle = open('E:\\SculptPrint\\PocketNC\\OpenCNC\\Interface Application\\pncApp\\Logs\\' + datetime.datetime.now().strftime("%Y.%m.%d-%H.%M.%S") + '.txt', 'w')
        self.log_file_handle = open('C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Logs\\' + datetime.datetime.now().strftime("%Y.%m.%d-%H.%M.%S") + '.txt', 'w')

        self.rx_received_time = 0

        #ASCII Feedback
        self.received_data_string = ""

        # Binary Feedback
        self.byte_string = bytearray()
        self.binary_transmission_length = pncLibrary.calculateBinaryTransmissionLength(machine)

        self.transmission_received = False
        self.terminator_received = False

        self.header_processed = False
        self.header_processing_error = False
        self.feedback_data_processed = False
        self.feedback_data_processing_error = False
        self.incoming_transmission_length = None
        self.feedback_encoding = None
        self.feedback_type = None
        self.header_delimiter_index = None
        self.rsh_error_check = False

        self._running_process = True
        print('PROCESS LAUNCHED: RSH feedback handler, PID: ' + self.pid + ' from ' + str(super().pid))

    def run(self):
        print('feedback socket is ' + str(self.machine.rsh_socket))
        while self._running_process:
            # Process feedback from EMCRSH
            data_available = select.select([self.machine.rsh_socket], [], [], 0)
            if data_available[0]:
                #Clock when these data were received
                rx_received_time = time.clock()
                #First parse the header
                #FIXME can never catch up to rate of data transmission due to select.wait
                bytes_received = self.machine.rsh_socket.recv(self.machine.bytes_to_receive)
                self.byte_string.extend(bytes_received)
                print('byte_string length is now ' + str(len(self.byte_string)))
                #FIXME handle this with database manager
                self.log_file_handle.write(str(rx_received_time) + str(bytes_received) + '\n')
                self.log_file_handle.flush()

            if len(self.byte_string) >= self.machine.minimum_header_length and not self.header_processed:
                self.header_processed, self.header_processing_error, self.feedback_encoding, self.feedback_type, self.header_delimiter_index = self.assembleFeedbackHeader(self.byte_string)
                self.rx_received_time = rx_received_time

            #FIXME should this handle multiple transmissions in one loop?
            if self.header_processed and not self.header_processing_error:
                #Drop header data, it's already good
                old_byte_string_before_header = self.byte_string
                self.byte_string = self.byte_string[self.header_delimiter_index:]
                self.feedback_data_processed, self.feedback_data_processing_error, self.feedback_data, self.complete_transmission_delimiter_index = \
                    self.assembleAndProcessFeedbackData(self.feedback_encoding, self.feedback_type, self.byte_string, self.rx_received_time)

                if self.feedback_data_processed and not self.feedback_data_processing_error:
                    #Now drop the complete command from the buffer and reset flags
                    old_byte_string = self.byte_string
                    self.byte_string = self.byte_string[self.complete_transmission_delimiter_index:]
                    # if len(self.byte_string) >= 1:
                    #     if self.byte_string[0] == 67:
                    #         print('break')
                    self.header_processed = False
                    self.header_processing_error = False
                    self.feedback_data_processed = False
                    self.feedback_data_processing_error = False
                    self.last_processed_byte_string = self.byte_string



                    #     #Check for a space character
                    #     if self.machine.header_delimiter_bytes in self.byte_string:
                    #         #Count delimiters in header string
                    #         #?
                    #         print('the byte string is ' + self.byte_string.decode('utf-8'))
                    #         #self.processFeedbackHeader(self.byte_string[:self.byte_string.index(self.machine.header_delimiter_bytes)])
                    #         self.processFeedbackHeader()
                    #     else:
                    #         #No space received yet, make another pass
                    #         pass
                    # else:
                    #     #Not enough data yet, make another pass
                    #     pass


                    # if not len(self.byte_string) >= self.machine.binary_header_length:
                    #     #We haven't received a complete header yet, make another pass
                    #     pass

                    # if self.header_received:
                    #     #bytes_received = self.rsh_socket.recv(self.machine.bytes_to_receive)
                    #     #Receive feedback data each loop
                    #     if len(self.byte_string) > self.feedback_type[1]:
                    #         #Received more bytes than are in a complete transmission, so roll over to the next one
                    #         self.complete_byte_string = self.byte_string[:self.feedback_type[1]]
                    #         #Now process this feedback
                    #         self.processBinaryFeedback(self.rx_received_time, self.complete_byte_string)
                    #
                    #         #This byte_string starts with a header
                    #         self.byte_string = self.byte_string[self.feedback_type[1]:]
                    #         self.header_received = False

                    # else:
                    #     self.feedback_type = self.processBinaryHeader(self.byte_string[:self.machine.binary_header_length])
                    #     self.rx_received_time = rx_received_time
                    #     self.header_received = True
                    #
                    #     #Now drop the header
                    #     self.byte_string = self.byte_string[self.machine.binary_header_length:]
                    #
                    #     #Update this in case it changed since last go round
                    #     self.binary_transmission_length = self.machine.calculateBinaryTransmissionLength()
                    #     #self.incoming_transmission_length = self.feedback_type[1]

        #Flag set, shutdown. Handle socket closure in machine_controller
        print('flagging feedback socket')
        #self.machine_controller._running = False
        self.log_file_handle.close()
        self._shutdown = True

    def gobbleTerminators(self, byte_string, terminator):
        if len(byte_string) >= len(terminator):
            if byte_string[0:len(terminator)] == terminator:
                print('detected useless terminators')
                return self.gobbleTerminators(byte_string[len(terminator):],terminator) or b''
            else:
                return byte_string
        else:
            return byte_string

    def countTerminatorsToGobble(self, byte_string, terminator):
        terminator_count = 0
        if len(byte_string) >= len(terminator):
            for b in range(0,len(byte_string),len(terminator)):
                if byte_string[b:b+len(terminator)] == terminator:
                    terminator_count += 1
                else:
                    break
        return terminator_count


    def assembleFeedbackHeader(self, byte_string):
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
            header_delimiter_index += self.countTerminatorsToGobble(byte_string[header_delimiter_index:],self.machine.binary_header_delimiter)*len(self.machine.binary_header_delimiter)
        elif any(s in header_string for s in self.machine.rsh_echo_strings):
            feedback_encoding = 'ascii'
            feedback_type = 'command echo'
        else:
            print('can\'t find header string')
            return False, False, None, None, None

            # if self.rsh_error_check:
            #     print('definite error')

        return True, False, feedback_encoding, feedback_type, header_delimiter_index
        #print('the complete feedback string is ' + complete_feedback_string)
        #print('determined feedback type ' + self.feedback_type)

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
                self.processFeedbackData(feedback_encoding, feedback_type, feedback_data, rx_received_time)
                # Drop this command from the buffer and gobble remaining terminators
                # remaining_byte_string = self.gobbleTerminators(byte_string[line_terminator_index:],
                #                                                self.machine.ascii_line_terminator.encode('utf-8'))

                #complete_transmission_delimiter_index = line_terminator_index + byte_string[line_terminator_index:].index(self.gobbleTerminators(byte_string[line_terminator_index:],self.machine.ascii_line_terminator.encode('utf-8'))[:])
                #complete_transmission_delimiter_index = line_terminator_index + self.countTerminatorsToGobble(byte_string[line_terminator_index:]*len(line_terminator),line_terminator)
                complete_transmission_delimiter_index = line_terminator_index + self.countTerminatorsToGobble(byte_string[line_terminator_index:], line_terminator)*len(line_terminator)
                # self.gobbleTerminators()
                return True, False, feedback_data, complete_transmission_delimiter_index
            else:
                #No line terminator, go for another pass
                print('making another pass on the socket')
                return False, False, None, None

        elif feedback_encoding == 'binary':
            #The first four bytes give transmission length
            if len(byte_string) >= self.machine.size_of_feedback_int:
                transmission_length = struct.unpack('!I',byte_string[:self.machine.size_of_feedback_int])[0]
                self.incoming_transmission_length = transmission_length
                byte_string = byte_string[self.machine.size_of_feedback_int:]
                if len(byte_string) >= transmission_length:
                    if transmission_length <= len(self.machine.binary_line_terminator):
                        print('break')
                    #Now we have a full transmission and can process it
                    #byte_string = byte_string[:transmission_length+len(self.machine.binary_line_terminator)]
                    byte_string = byte_string[:transmission_length]
                    #Check for complete transmission with terminator
                    if byte_string[-len(self.machine.binary_line_terminator)] == ord(self.machine.binary_line_terminator):
                        feedback_data = byte_string[:-len(self.machine.binary_line_terminator)]

                        #feedback_data = byte_string[:-1]
                        self.processFeedbackData(feedback_encoding, feedback_type, feedback_data, rx_received_time, transmission_length)
                        complete_transmission_delimiter_index = self.machine.size_of_feedback_int+transmission_length
                        return True, False, feedback_data, complete_transmission_delimiter_index
                    else:
                        print('binary feedback had incorrect terminator')
                        return True, True, None, None
                else:
                    #Wait for complete transmission
                    print('waiting for complete transmission')
                    return False, False, None, None
            else:
                # Wait for transmission length integer
                print('waiting for transmission length')
                return False, False, None, None

            #if self.machine.ascii_line_terminator.encode('utf-8') in byte_string:

    def processFeedbackData(self, feedback_encoding, feedback_type, feedback_data, rx_received_time, transmission_length = -1):
        if feedback_type.upper() == 'COMMAND ECHO':
            if self.machine.ascii_rsh_feedback_strings[-1] in feedback_data:
                print('RSH error')
                self.handleRSHError()
            else:
                print('got echo data')
            return

        if feedback_encoding == 'ascii':
            # if type(feedback_type) is str:
            #     print('break')
            #feedback_type = self.machine.ascii_rsh_feedback_strings[feedback_type]
            #feedback_data = [s.upper() for s in feedback_data]
            feedback_data = list(map(str.upper,feedback_data))
            #if any(s in feedback_type for s in self.machine.ascii_rsh_feedback_strings):
            if self.machine.ascii_rsh_feedback_strings[0] == feedback_type:
                #Position feedback
                if self.machine.logging_mode != 1:
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
                self.machine.status_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[3] == feedback_type:
                # Machine Mode
                # print('mode set')
                #print('got mode ' + feedback_data[0])
                self.machine.mode = feedback_data[0]
                self.machine.mode_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[4] == feedback_type:
                # Servo Log Params
                print('got logging ' + str(feedback_data))
                self.machine.logging_mode = int(feedback_data[0])
                self.machine.logging_mode_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[5] == feedback_type:
                # Drive Power
                #self.machine.drive_power = self.machine.rsh_feedback_flags.index(feedback_data.upper())
                self.machine.drive_power = self.machine.checkOnOff(feedback_data[0])
                self.machine.drive_power_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[6] == feedback_type:
                # Echo
                # print('got set echo: ' + data_string)
                self.machine.echo = self.machine.checkOnOff(feedback_data[0])
                self.machine.echo_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[7] == feedback_type:
                # Hello
                print('got hello')
                self.machine.connected = 1
                self.machine.connection_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[8] == feedback_type:
                # Enable
                # print('got enable')
                #self.machine.linked = self.machine.rsh_feedback_flags.index(feedback_data[0].upper())
                self.machine.linked = self.machine.checkOnOff(feedback_data[0])
                self.machine.link_change_event.set()
                self.machine.sculptprint_interface.connect_event.set()
            elif self.machine.ascii_rsh_feedback_strings[9] == feedback_type:
                # EStop
                # print('got estop: ' + data_string)
                #self.machine.estop = self.machine.rsh_feedback_flags.index(feedback_data.upper())
                self.machine.estop = self.machine.checkOnOff(feedback_data[0])
                self.machine.estop_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[10] == feedback_type:
                # Joint homed
                # print('got joint home: ' + data_string)
                for axis in range(0, self.machine.number_of_joints):
                    # self.machine.axis_home_state[axis] = self.machine.rsh_feedback_flags.index(data_string.split()[axis+1].strip().upper()) % 2
                    self.machine.axis_home_state[axis] = self.machine.checkOnOff(feedback_data[axis])
                    # print('joint home state is ' + str(self.machine.axis_home_state))
                if all(self.machine.axis_home_state):
                    print('all joints homed')
                    self.machine.all_homed_event.set()
                    # self.machine.home_change_event.set()
                self.machine.home_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[11] == feedback_type:
                # PING
                self.machine.ping_rx_time = rx_received_time
                self.machine.estimated_network_latency = (self.machine.ping_rx_time - self.machine.ping_tx_time) / 2
                self.machine.ping_event.set()
            elif self.machine.ascii_rsh_feedback_strings[12] == feedback_type:
                # Time
                self.machine.last_unix_time = float(feedback_data[0])*self.machine.clock_resolution
                self.machine.clock_sync_received_time = rx_received_time
                self.machine.clock_event.set()
            elif self.machine.ascii_rsh_feedback_strings[13] == feedback_type:
                # Comm mode
                print('got comm_mode')
                self.machine.comm_mode = self.machine.checkOnOff(feedback_data[0])
                self.machine.comm_mode_change_event.set()
            else:
                print('received unrecognized ascii string for header ' + str(feedback_type) + 'with data ' + str(feedback_data))
        elif feedback_encoding == 'binary':
            #feedback_type = self.machine.binary_rsh_feedback_strings[feedback_type]
            #print('the feedback type is: ' + feedback_type)
            if any(s in feedback_type for s in self.machine.binary_rsh_feedback_strings):
                if self.machine.binary_rsh_feedback_strings[0] == feedback_type:
                    #Servo position feedback
                    # if len(feedback_data) < 2400:
                    #     print('break')
                    feedback_data = struct.unpack('!' + 'd' * int((transmission_length - len(
                        self.machine.binary_line_terminator)) / self.machine.size_of_feedback_double), feedback_data)
                    self.processPositionFeedback('binary',rx_received_time,feedback_data)
                    self.machine.servo_feedback_reception_event.set()
                elif self.machine.binary_rsh_feedback_strings[1] == feedback_type:
                    #FIXME unpack binary here
                    self.processBufferLevelFeedback('binary', rx_received_time, feedback_data)
                    self.machine.buffer_level_reception_event.set()


    def processBufferLevelFeedback(self, feedback_encoding, rx_received_time, feedback_data):
        print('got buffer_level feedback')
        if feedback_encoding == 'ascii':
            #print('trying to parse buffer level feedback string: ' + data_string)
            #rx_received_time = time.clock()
            #m = re.search(self.machine.rsh_feedback_strings[1] + '(\d+)'+self.machine.rsh_feedback_strings[2]+'([+-]?([0-9]*[.])?[0-9]+)', data_string)
            #BL_data = feedback_data.split()
            rsh_clock_time = float(feedback_data[0])
            rsh_buffer_level = int(feedback_data[1])


            # self.rsh_buffer_level = int(m.group(1))
            # rsh_clock_time = float(m.group(2))
            #self.data_store.appendMachineControlRecords([dict([('highres_tcq_length', self.rsh_buffer_level)])])
            # record = dict()
            # record['HIGHRES_TC_QUEUE_LENGTH'] = self.rsh_buffer_level
            # record['highfreq_ethernet_received_times'] = rx_received_time
            # record['rsh_clock_times'] = rsh_clock_time
        elif feedback_encoding == 'binary':
            feedback_data = struct.unpack('!dI', feedback_data)
            rsh_clock_time = feedback_data[0]
            rsh_buffer_level = feedback_data[1]

        record = dict()
        record['HIGHRES_TC_QUEUE_LENGTH'] = rsh_buffer_level
        record['HIGHFREQ_ETHERNET_RECEIVED_TIMES'] = rx_received_time

        #FIXME major band-aid
        record['RSH_CLOCK_TIMES'] = rsh_clock_time-self.machine.OS_clock_offset+self.machine.RT_clock_offset

        #self.data_store.appendMachineFeedbackRecords([record])
        self.machine.data_store_manager_thread_handle.push(record)

    def processPositionFeedback(self, feedback_encoding, rx_received_time, feedback_data):
        print('got position feedback')
        try:
            #machine_feedback_records = []
            if feedback_encoding == 'ascii':
                #samples = feedback_data.strip('*').strip('|').split('*')
                #Handle error for incomplete last entry, FIXME find root cause
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

                # #FIXME move this to DataStoreManager
                # if len(self.data_store.RTAPI_feedback_indices) != 0:
                #     RTAPI_feedback_indices = np.zeros([number_of_feedback_points, 1]) + self.data_store.RTAPI_feedback_indices[-1] + 1
                # else:
                #     RTAPI_feedback_indices = np.zeros([number_of_feedback_points, 1])

                #rx_received_time = time.clock()
                if len(feedback_data) < 10:
                    print('feedback error with data_string = ' + feedback_data)

                for sample_number in range(0,number_of_feedback_points):
                    sample_point = feedback_data[sample_number].split(self.machine.ascii_servo_feedback_terminator)
                    RTAPI_feedback_indices[sample_number, :] += float(sample_number)
                    RTAPI_clock_times[sample_number,:] = float(sample_point[0])
                    stepgen_feedback_positions[sample_number,:] = np.asarray([float(s) for s in sample_point[1:]])+np.asarray(self.machine.axis_offsets)
                    ## FIXME use clock delta between samples instead of straight interpolation
                    received_times_interpolated[sample_number, :] = rx_received_time + self.machine.servo_dt*self.machine.servo_log_sub_sample_rate * float(sample_number)
                    #self.machine.current_position = stepgen_feedback_positions[sample_number,:]

                # record = dict()
                # record['RTAPI_feedback_indices'] = RTAPI_feedback_indices
                # record['RTAPI_clock_times'] = RTAPI_clock_times
                # record['stepgen_feedback_positions'] = stepgen_feedback_positions
                # record['lowfreq_ethernet_received_times'] = lowfreq_ethernet_received_times
                # # record['lowfreq_ethernet_received_times'] = received_times_interpolated

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
                    stepgen_feedback_positions[sample_number, :] = np.asarray([float(s) for s in sample_point[1:]]) + np.asarray(self.machine.axis_offsets)
                    received_times_interpolated[sample_number, :] = rx_received_time + self.machine.servo_dt * self.machine.servo_log_sub_sample_rate * float(sample_number)

            else:
                return

            record = dict()
            record['RTAPI_feedback_indices'] = RTAPI_feedback_indices
            record['RTAPI_clock_times'] = RTAPI_clock_times
            record['stepgen_feedback_positions'] = stepgen_feedback_positions
            record['lowfreq_ethernet_received_times'] = lowfreq_ethernet_received_times

            ## FIXME major band-aid
            if len(self.machine.data_store_manager_thread_handle.data_store.RTAPI_CLOCK_TIMES) == 0:
                self.machine.RT_clock_offset = RTAPI_clock_times[0].item()


        except Exception as error:
            print('had error processing feedback string. The data string is: ' + str(feedback_data))
            print('the error is ' + str(error))
            return

        self.machine.data_store_manager_thread_handle.push(record)

    def handleRSHError(self):
        self.machine.rsh_error = 1

    def close(self):
        self._running = True
        self._running = False


#serialLock = threading.Lock()
# class SerialInterface(threading.Thread):
#     def __init__(self, machine):
#         super(SerialInterface, self).__init__()
#         self.machine = machine
#
#         #self._running = True
#         self._shutdown = False
#
#         self.serial_port = serial.Serial()
#         self.serial_port.port = self.machine.comm_port
#         self.serial_port.baudrate = self.machine.baudrate
#         #FIXME handle if serial is not connected
#
#         try:
#             if self.serial_port.isOpen():
#                 print('serial port already open')
#             self.serial_port.open()
#             time.sleep(0.5)
#             #self.rebootDevice()
#
#             self._running_thread = True
#
#             # if self.machine.initial_position_set_event.wait():
#             #     self.setAllEncoderCounts(self.machine.axes,self.machine.current_position)
#             #     self.machine.encoder_init_event.set()
#             #while not self.machine.data_store_manager_thread_handle.pull(['stepgen_feedback_positions'],[-1],[None])[0]:
#
#         except Exception as error:
#             print('Could not open serial port, error: ' + str(error))
#
#
#         print('serialInterface started')
#         #time.sleep(0.5)
#
#     def run(self):
#         #global encoderData, serialLock
#         #self.setEncoderCount(self.machine.encoder_init)
#         if not self.acknowledgeBoot():
#             print('Decoder board not responsive')
#             return
#         else:
#             print('SUCCESS: Decoder board boot')
#
#         if self.machine.initial_position_set_event.wait():
#             self.setAllEncoderCounts(self.machine.axes, self.positionsToCounts(self.machine.axes,list(map(lambda cp,mz: cp-mz,self.machine.current_position,self.machine.machine_zero))))
#             self.machine.encoder_init_event.set()
#
#         self.setBaudrate(250000)
#         self.serial_port.flushInput()
#
#         while self._running_thread:
#             rx_received_time = time.clock()
#
#             encoder_counts = self.getEncoderCounts('current')
#
#             if encoder_counts[0]:
#                 #Got good data, push to DB
#                 self.machine.data_store_manager_thread_handle.push(
#                     {'ENCODER_FEEDBACK_POSITIONS': np.asarray(self.countsToPositions(self.machine.axes, encoder_counts[1])), 'SERIAL_RECEIVED_TIMES': self.machine.estimateMachineClock(rx_received_time)})
#
#             # start = time.clock()
#
#             # if self.serial_port.in_waiting > self.machine.max_encoder_transmission_length:
#             #     self.serial_port.readline()
#             #     #print(self.serial_port.readline())
#                 #print(time.clock() - start)
#             #else:
#                 #print('looping')
#             #print(self.serial_port.readline())
#             #print(self.serial_port.read_until('\r\n'.encode()))
#             #print(self.serial_port.read_all())
#             #self.serial_port.read_all()
#             #if self.serial_port.
#             #rx_received_time = time.clock()
#             #print(counts)
#             #print(time.clock()-start)
#
#         #Flag set, shutdown
#         print('Closing serial port')
#         print('ENCODER INTERFACE: thread shut down')
#         self.serial_port.close()
#         #self._shutdown = True
#
#     def read(self):
#         line = self.serial_port.readline()
#         return line
#
#     def writeUnicode(self, string):
#         self.serial_port.write(string.encode('utf-8'))
#         #time.sleep(0.01)
#         #self.serial_port.flush()
#
#     ##    def write(self, data):
#     ##        line = self.serial_port.readline()
#     ##        return line
#
#     # def reset(self):
#     #     global encoderData, serialLock
#     #     #serialLock.acquire()
#     #     self.serial_port.close()
#     #     self.serial_port.open()
#     #     encoderData = np.array([0])
#     #     queueData = np.array([0])
#     #     #serialLock.release()
#
#     ########################### UTILITIES ###########################
#
#     def countBytesInTransmission(self, transmission):
#         return math.floor(round(math.log(transmission, 10), 6)) + 1
#
#     def positionToCounts(self, axis, position):
#         axis_index = self.machine.axes.index(axis.upper())
#         return round(position / self.machine.encoder_scale[axis_index]) + self.machine.encoder_offset[axis_index]
#
#     def positionsToCounts(self, axes, positions):
#         counts = []
#         for axis in range(0,len(axes)):
#             axis_index = self.machine.axes.index(axes[axis].upper())
#             counts.append(int(round(positions[axis] / self.machine.encoder_scale[axis_index]) + self.machine.encoder_offset[axis_index]))
#         return counts
#
#     def countsToPosition(self, axis, counts):
#         axis_index = self.machine.axes.index(axis.upper())
#         return round(counts * self.machine.encoder_scale[axis_index]) - self.machine.encoder_offset[axis]
#
#     def countsToPositions(self, axes, counts):
#         positions = []
#         for axis in range(0, len(axes)):
#             axis_index = self.machine.axes.index(axes[axis].upper())
#             positions.append((counts[axis] - self.machine.encoder_offset[axis_index]) * self.machine.encoder_scale[axis_index] + self.machine.machine_zero[axis_index])
#         return positions
#
#     def waitForBytes(self, number_of_bytes, timeout = 0.5):
#         read_byte_count = 0
#         read_data = ''
#         start_time = time.clock()
#         while read_byte_count < number_of_bytes and (time.clock() - start_time) < timeout:
#             read_data += self.serial_port.read(1).decode('utf-8')
#             read_byte_count += 1
#
#         if read_byte_count == number_of_bytes:
#             return (True, read_data)
#         else:
#             return (False, None)
#
#     def waitForString(self, string, timeout = 0.5):
#         # #read_byte_count = 0
#         # read_data = ''
#         # start_time = time.clock()
#         # while string not in read_data and (time.clock() - start_time) < timeout:
#         #     read_data += self.serial_port.readline().decode('utf-8')
#         #     #read_byte_count += 1
#
#         start = time.clock()
#         read_data = self.serial_port.readline().decode('utf-8')
#         print('read took ' + str(time.clock()-start))
#         if string in read_data:
#             return (True, read_data)
#         else:
#             return (False, None)
#
#     def rebootDevice(self):
#         self.serial_port.setDTR(0)
#         self.serial_port.setDTR(1)
#         time.sleep(0.5)
#
#     ########################### COMMS ###########################
#
#     def acknowledgeBoot(self):
#         return self.waitForString(self.machine.encoder_ack_strings[0])[0]
#
#     def setAxisEncoderCount(self, axis, count):
#         #number_of_bytes = math.floor(round(math.log(count, 10),6)) + 1
#         command_string = 'S' + str(axis) + str(self.countBytesInTransmission(count)) + str(count)
#         self.writeUnicode(command_string)
#         if self.waitForString(self.machine.encoder_ack_strings[1]):
#         #if self.waitForBytes(len(self.machine.encoder_ack_strings[0]))[1] == self.machine.encoder_ack_strings[0]:
#             return True
#         else:
#             return False
#
#     def setAllEncoderCounts(self, axes, counts):
#         for axis in axes:
#             axis_index = self.machine.axes.index(axis)
#             self.setAxisEncoderCount(axis_index+1,counts[axis_index])
#
#     # def setEncoderCount(self, count):
#     #     # self.serial_port.write('R'.encode('utf-8'))
#     #     # Calculate number of characters in set command
#     #     numBytes = math.floor(round(math.log(count, 10),6)) + 1
#     #     commandStr = 'S' + str(numBytes) + str(count) + '\n'
#     #     print('setting' + commandStr)
#     #     self.serial_port.write(commandStr.encode('utf-8'))
#     #
#     #     ack = self.waitForBytes(2)[1]
#     #     if ack == 'S&':
#     #         return True
#     #     else:
#     #         return False
#     #     # readData = ''
#     #     # while 'S&' not in readData:
#     #     #     #print("waiting to read")
#     #     #     readData += self.serial_port.read(1).decode("utf-8").strip()
#     #     # print('Successful set of encoder count to ' + str(count))
#
#     def getEncoderCounts(self, request_type):
#         #print('requesting encoder count')
#         #self.serial_port.write('G'.encode('utf-8'))
#         if request_type.upper() == 'STORED':
#             request_string = self.machine.encoder_command_strings[1]
#             ack_string = self.machine.encoder_ack_strings[-1]
#         elif request_type.upper() == 'CURRENT':
#             request_string = self.machine.encoder_command_strings[2]
#             ack_string = self.machine.encoder_ack_strings[-1]
#         else:
#             return (False, None)
#
#         self.writeUnicode(request_string)
#         encoder_bytes = self.serial_port.read(self.machine.max_encoder_transmission_length)
#         encoder_data = struct.unpack('!' + 'c' + self.machine.number_of_encoders * 'I' + 'c', encoder_bytes)
#         if encoder_data[0].decode() == request_string and encoder_data[-1].decode() == ack_string:
#             return (True, encoder_data[1:-1])
#         return(False, None)
#
#         # received_counts = self.waitForString(ack_string)
#         #
#         # if received_counts[0]:
#         #     #First character should be G
#         #     counts = received_counts[1].split(' ')
#         #     if counts[0] == request_string and counts[-1] == ack_string:
#         #         if len(counts[1]) < 9:
#         #             print('break')
#         #         return (True, list(map(int,counts[1:-1])))
#         # #else:
#         # return (False, None)
#
#         # if self.waitForString(self.machine.encoder_ack_strings[1])[0]:
#         #
#         # #time.sleep(0.1)
#         # readData = ''
#         # while 'C&' not in readData:
#         #     #print("waiting to read")
#         #     readData += self.serial_port.read(1).decode("utf-8")
#         # #print('Successful get of encoder count')
#         # #print(readData.strip())
#         # return readData.strip()
#
#     def setBaudrate(self, baudrate):
#         self.writeUnicode(self.machine.encoder_command_strings[3] + str(self.countBytesInTransmission(baudrate)) + str(baudrate))
#         ack = self.waitForString(self.machine.encoder_ack_strings[4])
#         if int(ack[1].split(' ')[1]) == baudrate:
#             time.sleep(0.5)
#             self.serial_port.baudrate = baudrate
#             print('SUCCESS: Decoder board baudrate is now ' + str(baudrate))
#
#     def close(self):
#         print('setting serial port close flag')
#         self._running_thread = False
#         #sys.exit()

