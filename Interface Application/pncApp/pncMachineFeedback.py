import socket
import threading, time
import re
import numpy as np
import serial, math, sys, select, datetime, struct

global machine

class MachineFeedbackListener(threading.Thread):
    global machine
    def __init__(self, machine, machine_controller, data_store):
        super(MachineFeedbackListener, self).__init__()
        self.rsyslog_socket = socket
        self._running_thread = True
        self._shutdown = False

        self.machine = machine
        self.machine_controller = machine_controller
        self.rsh_socket = machine_controller.rsh_socket
        self.data_store = data_store


        print('Feedback thread started')
        #self.log_file_handle = open('E:\\SculptPrint\\PocketNC\\OpenCNC\\Interface Application\\pncApp\\Logs\\' + datetime.datetime.now().strftime("%Y.%m.%d-%H.%M.%S") + '.txt', 'w')
        self.log_file_handle = open('C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Logs' + datetime.datetime.now().strftime("%Y.%m.%d-%H.%M.%S") + '.txt', 'w')

        self.rx_received_time = 0

        #ASCII Feedback
        self.received_data_string = ""

        # Binary Feedback
        self.byte_string = bytearray()
        self.binary_transmission_length = self.machine.calculateBinaryTransmissionLength()

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

    def run(self):
        print('feedback socket is ' + str(self.rsh_socket))
        while self._running_thread:
            # Process feedback from EMCRSH
            data_available = select.select([self.rsh_socket], [], [], 0.5)
            if data_available[0]:
                #Clock when these data were received
                rx_received_time = time.clock()
                #First parse the header

                # if not self.machine.binary_mode and False:
                #     string_received = self.rsh_socket.recv(self.machine.bytes_to_receive).decode("utf-8")
                #     complete_data_string = ''
                #     if '\n' in string_received or '\r' in string_received:
                #         #split_received_strings = string_received.split('\n')
                #         split_received_strings = re.split(r'\r|\n', string_received)
                #         #print('split received string is: ')
                #         #print(split_received_strings)
                #         #Complete the first entry of the split string list from incomplete data, probably doesn't actually happen?
                #         split_received_strings[0] = self.received_data_string + split_received_strings[0]
                #         for string in split_received_strings:
                #             #Drop null strings
                #             #print('going through the split received strings.')
                #             #print(string)
                #             if string:
                #                 self.log_file_handle.write(string + '\n')
                #                 self.log_file_handle.flush()
                #                 self.processRSHString(string.strip(),rx_received_time)
                #
                #         self.received_data_string = ''
                #     elif string_received:
                #         #Incomplete data received
                #         #print('got incomplete data with received = ' + self.received_data_string + ' and string_received = ' + string_received)
                #         self.received_data_string += string_received
                #     else:
                #         #Null string received. RSH disconnected?
                #         print('received null string: ' + string_received)
                #         print('RSH disconnected...')
                #     # print(self.received_data_string)
                #else:
                #FIXME can never catch up to rate of data transmission due to select.wait
                bytes_received = self.rsh_socket.recv(self.machine.bytes_to_receive)
                self.byte_string.extend(bytes_received)

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

                    #FIXME implement logging to file
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
            header_string = header_bytes.decode('utf-8')
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

        #Check for binary/ascii/ascii echo
        if any(s in header_string for s in self.machine.ascii_rsh_feedback_strings):
            feedback_encoding = 'ascii'
            if header_string != '*':
                print(header_string)
            feedback_type = self.machine.ascii_rsh_feedback_strings.index([s for s in self.machine.ascii_rsh_feedback_strings if header_string in s][0])
        elif any(s in header_string for s in self.machine.binary_rsh_feedback_strings):
            feedback_encoding = 'binary'
            feedback_type = self.machine.binary_rsh_feedback_strings.index([s for s in self.machine.binary_rsh_feedback_strings if header_string in s][0])
            header_delimiter_index += self.countTerminatorsToGobble(byte_string[header_delimiter_index:],self.machine.binary_header_delimiter)*len(self.machine.binary_header_delimiter)
        elif any(s in header_string for s in self.machine.rsh_echo_strings):
            feedback_encoding = 'ascii'
            feedback_type = 'echo'
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
        if feedback_encoding == 'ascii':
            if type(feedback_type) is str:
                print('break)')
            feedback_type = self.machine.ascii_rsh_feedback_strings[feedback_type]
            feedback_data = [s.upper() for s in feedback_data]
            #if any(s in feedback_type for s in self.machine.ascii_rsh_feedback_strings):
            ## ['PROGRAM_STATUS', 'MODE', 'SERVO_LOG_PARAMS', 'MACHINE', 'ECHO', 'HELLO', 'ENABLE', 'ESTOP', 'JOINT_HOMED', 'PING', 'TIME', 'NAK']

            if self.machine.ascii_rsh_feedback_strings[-1] in feedback_data and feedback_type == 'echo':
                print('RSH error')
                self.handleRSHError()
            elif self.machine.ascii_rsh_feedback_strings[0] == feedback_type:
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
                for axis in range(0, self.machine.num_joints):
                    # self.machine.axis_home_state[axis] = self.machine.rsh_feedback_flags.index(data_string.split()[axis+1].strip().upper()) % 2
                    self.machine.axis_home_state[axis] = self.machine.checkOnOff(feedback_data[axis])
                    # print('joint home state is ' + str(self.machine.axis_home_state))
                if all(self.machine.axis_home_state):
                    print('all joints homed')
                    self.machine.all_homed_event.set()
                    # self.machine.home_change_event.set()
                self.machine.home_change_event.set()
            elif self.machine.ascii_rsh_feedback_strings[11] == feedback_type:
                self.machine.ping_rx_time = rx_received_time
                self.machine.estimated_network_latency = (self.machine.ping_rx_time - self.machine.ping_tx_time) / 2
                self.machine.ping_event.set()
            elif self.machine.ascii_rsh_feedback_strings[12] == feedback_type:
                # Time
                self.machine.last_unix_time = float(feedback_data[0])
                self.machine.clock_event.set()
            elif self.machine.ascii_rsh_feedback_strings[13] == feedback_type:
                # Comm mode
                print('got comm_mode')
                self.machine.comm_mode = self.machine.checkOnOff(feedback_data[0])
                self.machine.comm_mode_change_event.set()
            else:
                print('received unrecognized ascii string for header ' + str(feedback_type) + 'with data ' + str(feedback_data))
        elif feedback_encoding == 'binary':
            feedback_type = self.machine.binary_rsh_feedback_strings[feedback_type]
            #print('the feedback type is: ' + feedback_type)
            if any(s in feedback_type for s in self.machine.binary_rsh_feedback_strings):
                if self.machine.binary_rsh_feedback_strings[0] == feedback_type:
                    #Servo position feedback
                    if len(feedback_data) < 2400:
                        print('break')
                    feedback_data = struct.unpack('!' + 'd' * int((transmission_length - len(
                        self.machine.binary_line_terminator)) / self.machine.size_of_feedback_double), feedback_data)
                    self.processPositionFeedback('binary',rx_received_time,feedback_data)
                    self.machine.servo_feedback_reception_event.set()
                elif self.machine.binary_rsh_feedback_strings[1] == feedback_type:
                    #FIXME unpack binary here
                    self.processBufferLevelFeedback('binary', rx_received_time, feedback_data)
                    self.machine.buffer_level_reception_event.set()

                #
                #
                #
                # print('received: ' + data_string)
                #
                # elif self.machine.rsh_feedback_strings[-1] in data_string:
                #     # Error in RSH command
                #     print('RSH error')
                #     self.handleRSHError()
                # elif self.machine.rsh_feedback_strings[0] in data_string:
                #     # FIXME raise error if feedback state active but we aren't getting data for 500ms, also measure receiption time of all data
                #     # Position Feedback
                #     # print('processing position feedback with data string: ' + data_string)
                #     # print('processing position feedback')
                #     # Force logging flag
                #
                #     # self.machine.logging_mode = 1
                #     if data_string[0] == self.machine.rsh_feedback_strings[0]:
                #         # Beginning of transmission is good
                #         self.processPositionFeedback('ascii', rx_received_time, data_string)
                #     else:
                #         # Incomplete transmission, discard for now FIXME find root cause
                #         print('got incomplete beginning of transmission')
                #         pass
                # elif self.machine.rsh_feedback_strings[1] in data_string:
                #     # Buffer Length
                #     self.processBLFeedback(data_string, rx_received_time)
                # elif self.machine.rsh_feedback_strings[3] in data_string:
                #     # Program Status
                #     # print('program status')
                #     self.machine.status = data_string.split(' ')[1].strip()
                #     # print('set program status to ' + self.machine.status)
                #     self.machine.status_change_event.set()
                # elif self.machine.rsh_feedback_strings[4] in data_string:
                #     # Machine Mode
                #     # print('mode set')
                #     print('got mode ' + data_string.split(' ')[1].strip())
                #     self.machine.mode = data_string.split(' ')[1].strip()
                #     self.machine.mode_change_event.set()
                #     # print('set machine mode to ' + self.machine.mode)
                # elif self.machine.rsh_feedback_strings[6] in data_string:
                #     # Servo Log Params
                #     # print('got logging ' + data_string.split(' ')[1].strip())
                #     self.machine.logging_mode = int(data_string.split()[1].strip())
                #     self.machine.logging_mode_change_event.set()
                # elif self.machine.rsh_feedback_strings[7] in data_string:
                #     # Drive Power
                #     self.machine.drive_power = self.machine.rsh_feedback_flags.index(
                #         data_string.split()[1].strip().upper())
                #     self.machine.drive_power_change_event.set()
                # elif self.machine.rsh_feedback_strings[8] in data_string:
                #     # Echo
                #     # print('got set echo: ' + data_string)
                #     self.machine.echo = self.machine.rsh_feedback_flags.index(data_string.split()[1].strip().upper())
                #     self.machine.echo_change_event.set()
                # elif self.machine.rsh_feedback_strings[9] in data_string:
                #     # Hello
                #     # print('got hello')
                #     self.machine.connected = 1
                #     self.machine.connection_change_event.set()
                # elif self.machine.rsh_feedback_strings[10] in data_string:
                #     # Enable
                #     # print('got enable')
                #     self.machine.linked = self.machine.rsh_feedback_flags.index(data_string.split()[1].strip().upper())
                #     self.machine.link_change_event.set()
                # elif self.machine.rsh_feedback_strings[11] in data_string:
                #     # EStop
                #     # print('got estop: ' + data_string)
                #     self.machine.estop = self.machine.rsh_feedback_flags.index(data_string.split()[1].strip().upper())
                #     self.machine.estop_change_event.set()
                # elif self.machine.rsh_feedback_strings[12] in data_string:
                #     # Joint homed
                #     # print('got joint home: ' + data_string)
                #     for axis in range(0, self.machine.num_joints):
                #         # self.machine.axis_home_state[axis] = self.machine.rsh_feedback_flags.index(data_string.split()[axis+1].strip().upper()) % 2
                #         self.machine.axis_home_state[axis] = self.machine.checkOnOff(data_string.split()[axis + 1]) % 2
                #         # print('joint home state is ' + str(self.machine.axis_home_state))
                #
                #     if all(self.machine.axis_home_state):
                #         print('all joints homed')
                #         self.machine.all_homed_event.set()
                #         # self.machine.home_change_event.set()
                #
                #     self.machine.home_change_event.set()
                # if self.machine.rsh_feedback_strings[13] in data_string:
                #     self.machine.ping_rx_time = rx_received_time
                #     self.machine.estimated_network_latency = (self.machine.ping_rx_time - self.machine.ping_tx_time) / 2
                #     self.machine.ping_event.set()
                # elif self.machine.rsh_feedback_strings[14] in data_string:
                #     # Time
                #     self.machine.last_unix_time = float(data_string.split()[1])
                #     self.machine.clock_event.set()
                # else:
                #     print('received unrecognized string ' + data_string)

    # def processBinaryHeader(self, bytes):
    #     header_data = struct.unpack('!ccxI',bytes)
    #     #print('got header data ' + str(header_data))
    #     feedback_type = ''.join((c.decode('utf-8') for c in header_data[0:len(header_data)-1]))
    #     return (feedback_type, header_data[-1])
    #
    # def processBinaryFeedback(self, rx_received_time, bytes):
    #     #print('length of transmission is ' + str(self.feedback_type[1]))
    #     unpacked_feedback_data = struct.unpack('!' + 'd' * int((self.feedback_type[1]-1)/self.machine.size_of_feedback_double) + 'c', bytes)
    #     #Transmission error check
    #     if unpacked_feedback_data[-1] != b'\x7f':
    #         print('Binary transmission error')
    #         return False
    #     if self.feedback_type[0] == 'SF':
    #         #This is servo feedback data
    #         self.processPositionFeedback('binary', rx_received_time, unpacked_feedback_data[:-1])
    #     elif self.feedback_type[0] == 'BL':
    #         #This is buffer level and time feedback data
    #         self.processBufferLevelFeedback('binary', rx_received_time, unpacked_feedback_data[:-1])
    #     #elif self.feedback_type[0] == 'AF':
    #         #ASCII feedback


    # def processRSHString(self, data_string, rx_received_time):
    #     ##FIXME detect if RSH crashed
    #     if any(s in data_string for s in self.machine.rsh_feedback_strings):
    #         # #Check ping first
    #         print('received: ' + data_string)
    #         if self.machine.rsh_feedback_strings[13] in data_string:
    #             self.machine.ping_rx_time = rx_received_time
    #             self.machine.estimated_network_latency = (self.machine.ping_rx_time-self.machine.ping_tx_time)/2
    #             self.machine.ping_event.set()
    #         elif self.machine.rsh_feedback_strings[-1] in data_string:
    #             # Error in RSH command
    #             print('RSH error')
    #             self.handleRSHError()
    #         elif self.machine.rsh_feedback_strings[0] in data_string:
    #             #FIXME raise error if feedback state active but we aren't getting data for 500ms, also measure receiption time of all data
    #             # Position Feedback
    #             #print('processing position feedback with data string: ' + data_string)
    #             #print('processing position feedback')
    #             #Force logging flag
    #             if self.machine.logging_mode != 1:
    #                 print('state machine sync error')
    #             #self.machine.logging_mode = 1
    #             if data_string[0] == self.machine.rsh_feedback_strings[0]:
    #                 #Beginning of transmission is good
    #                 self.processPositionFeedback('ascii',rx_received_time,data_string)
    #             else:
    #                 #Incomplete transmission, discard for now FIXME find root cause
    #                 print('got incomplete beginning of transmission')
    #                 pass
    #         elif self.machine.rsh_feedback_strings[1] in data_string:
    #             # Buffer Length
    #             self.processBLFeedback(data_string,rx_received_time)
    #         elif self.machine.rsh_feedback_strings[3] in data_string:
    #             # Program Status
    #             #print('program status')
    #             self.machine.status = data_string.split(' ')[1].strip()
    #             #print('set program status to ' + self.machine.status)
    #             self.machine.status_change_event.set()
    #         elif self.machine.rsh_feedback_strings[4] in data_string:
    #             # Machine Mode
    #             #print('mode set')
    #             print('got mode ' + data_string.split(' ')[1].strip())
    #             self.machine.mode = data_string.split(' ')[1].strip()
    #             self.machine.mode_change_event.set()
    #             #print('set machine mode to ' + self.machine.mode)
    #         elif self.machine.rsh_feedback_strings[6] in data_string:
    #             # Servo Log Params
    #             #print('got logging ' + data_string.split(' ')[1].strip())
    #             self.machine.logging_mode = int(data_string.split()[1].strip())
    #             self.machine.logging_mode_change_event.set()
    #         elif self.machine.rsh_feedback_strings[7] in data_string:
    #             # Drive Power
    #             self.machine.drive_power = self.machine.rsh_feedback_flags.index(data_string.split()[1].strip().upper())
    #             self.machine.drive_power_change_event.set()
    #         elif self.machine.rsh_feedback_strings[8] in data_string:
    #             # Echo
    #             #print('got set echo: ' + data_string)
    #             self.machine.echo = self.machine.rsh_feedback_flags.index(data_string.split()[1].strip().upper())
    #             self.machine.echo_change_event.set()
    #         elif self.machine.rsh_feedback_strings[9] in data_string:
    #             # Hello
    #             #print('got hello')
    #             self.machine.connected = 1
    #             self.machine.connection_change_event.set()
    #         elif self.machine.rsh_feedback_strings[10] in data_string:
    #             # Enable
    #             #print('got enable')
    #             self.machine.linked = self.machine.rsh_feedback_flags.index(data_string.split()[1].strip().upper())
    #             self.machine.link_change_event.set()
    #         elif self.machine.rsh_feedback_strings[11] in data_string:
    #             # EStop
    #             #print('got estop: ' + data_string)
    #             self.machine.estop = self.machine.rsh_feedback_flags.index(data_string.split()[1].strip().upper())
    #             self.machine.estop_change_event.set()
    #         elif self.machine.rsh_feedback_strings[12] in data_string:
    #             #Joint homed
    #             #print('got joint home: ' + data_string)
    #             for axis in range(0,self.machine.num_joints):
    #                 #self.machine.axis_home_state[axis] = self.machine.rsh_feedback_flags.index(data_string.split()[axis+1].strip().upper()) % 2
    #                 self.machine.axis_home_state[axis] = self.machine.checkOnOff(data_string.split()[axis + 1]) % 2
    #                 #print('joint home state is ' + str(self.machine.axis_home_state))
    #
    #             if all(self.machine.axis_home_state):
    #                 print('all joints homed')
    #                 self.machine.all_homed_event.set()
    #                 #self.machine.home_change_event.set()
    #
    #             self.machine.home_change_event.set()
    #         elif self.machine.rsh_feedback_strings[14] in data_string:
    #             #Time
    #             self.machine.last_unix_time = float(data_string.split()[1])
    #             self.machine.clock_event.set()
    #         else:
    #             print('received unrecognized string ' + data_string)


    def processBufferLevelFeedback(self, feedback_encoding, rx_received_time, feedback_data):
        if feedback_encoding == 'ascii':
            #print('trying to parse buffer level feedback string: ' + data_string)
            #rx_received_time = time.clock()
            #m = re.search(self.machine.rsh_feedback_strings[1] + '(\d+)'+self.machine.rsh_feedback_strings[2]+'([+-]?([0-9]*[.])?[0-9]+)', data_string)
            #BL_data = feedback_data.split()
            rsh_clock_time = float(feedback_data[0])
            rsh_buffer_length = int(feedback_data[1])


            # self.rsh_buffer_length = int(m.group(1))
            # rsh_clock_time = float(m.group(2))
            #self.data_store.appendMachineControlRecords([dict([('highres_tcq_length', self.rsh_buffer_length)])])
            # record = dict()
            # record['HIGHRES_TC_QUEUE_LENGTH'] = self.rsh_buffer_length
            # record['highfreq_ethernet_received_times'] = rx_received_time
            # record['rsh_clock_times'] = rsh_clock_time
        elif feedback_encoding == 'binary':
            feedback_data = struct.unpack('!dI', feedback_data)
            rsh_clock_time = feedback_data[0]
            rsh_buffer_length = feedback_data[1]

        record = dict()
        record['HIGHRES_TC_QUEUE_LENGTH'] = rsh_buffer_length

        #FIXME this is shit
        #self.machine.rsh_buffer_length = rsh_buffer_length

        record['highfreq_ethernet_received_times'] = rx_received_time
        record['rsh_clock_times'] = rsh_clock_time

        #self.data_store.appendMachineFeedbackRecords([record])
        self.machine.data_store_manager_thread_handle.push(record)

    def processPositionFeedback(self, feedback_encoding, rx_received_time, feedback_data):
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

                # if len(samples[-1].split('|')) < self.machine.servo_log_num_axes + 1:
                #     print('received incomplete last entry, discarding')
                #     samples = samples[:-1]
                #samples = data_string.split('*')[1:-1]

                feedback_data = [s.strip(self.machine.ascii_servo_feedback_terminator) for s in feedback_data]
                #print('the samples are:: ')
                #print(samples)
                #print('the number of points is ' + str(feedback_num_points))

                stepgen_feedback_positions = np.zeros([number_of_feedback_points,self.machine.servo_log_num_axes])
                RTAPI_clock_times = np.zeros([number_of_feedback_points,1])
                received_times_interpolated = np.zeros([number_of_feedback_points, 1])
                RTAPI_feedback_indices = np.zeros([number_of_feedback_points, 1])

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
                    #print('the sample is: ')
                    #print(sample)
                    #print(stepgen_feedback_positions)
                    stepgen_feedback_positions[sample_number,:] = np.asarray([float(s) for s in sample_point[1:]])+np.asarray(self.machine.axis_offsets)
                    ## FIXME use clock delta between samples instead of straight interpolation
                    received_times_interpolated[sample_number, :] = rx_received_time + self.machine.servo_dt*self.machine.servo_log_sub_sample_rate * float(sample_number)
                    self.machine.current_position = stepgen_feedback_positions[sample_number,:]
                    #np.vstack((stepgen_feedback_positions, sample.split('|')[1:-1]))

                lowfreq_ethernet_received_times = np.zeros([number_of_feedback_points, 1]) + rx_received_time

                record = dict()
                record['RTAPI_feedback_indices'] = RTAPI_feedback_indices
                record['RTAPI_clock_times'] = RTAPI_clock_times
                record['stepgen_feedback_positions'] = stepgen_feedback_positions
                record['lowfreq_ethernet_received_times'] = lowfreq_ethernet_received_times
                # record['lowfreq_ethernet_received_times'] = received_times_interpolated

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

                record = dict()
                record['RTAPI_feedback_indices'] = RTAPI_feedback_indices
                record['RTAPI_clock_times'] = RTAPI_clock_times
                record['stepgen_feedback_positions'] = stepgen_feedback_positions
                record['lowfreq_ethernet_received_times'] = lowfreq_ethernet_received_times


        except Exception as error:
            print('had error processing feedback string. The data string is: ' + str(feedback_data))
            print('the error is ' + str(error))
            return

        #machine_feedback_records.append(record)
        #self.data_store.appendMachineFeedbackRecords([record])
        self.machine.data_store_manager_thread_handle.push(record)
        #return machine_feedback_records

    def handleRSHError(self):
        self.machine.rsh_error = 1

    def close(self):
        self._running = True
        self._running = False


#serialLock = threading.Lock()
class SerialInterface(threading.Thread):
    def __init__(self, machine, data_store):
        super(SerialInterface, self).__init__()
        self._running = True
        self._shutdown = False
        #FIXME handle if serial is not connected
        try:
            self.serialPort = serial.Serial(  # set parameters, in fact use your own :-)
                port="COM12",
                baudrate=115200,
                #bytesize=serial.SEVENBITS,
                #parity=serial.PARITY_EVEN,
                #stopbits=serial.STOPBITS_ONE
            )
            self.serialPort.isOpen()  # try to open port, if possible print message and proceed with 'while True:'
            print("Successful open of serial port")

        except IOError:  # if port is already opened, close it and open it again and print message
            print('excepting')
            #self.serialPort.close()
            #self.serialPort.open()
            #print("port was already open, was closed and opened again!")
            print('port does not exist')
#        try:
#            self.serialPort = serial.Serial('COM12', 115200)
        #except:
            #serial.Serial.close('COM12')
        #self.serialPort = serial.Serial('COM12', 250000)
        self.data_store = data_store
        self.machine = machine
        print('serialInterface started')
        time.sleep(0.5)

    def read(self):
        line = self.serialPort.readline()
        return line

    ##    def write(self, data):
    ##        line = self.serialPort.readline()
    ##        return line

    def reset(self):
        global encoderData, serialLock
        #serialLock.acquire()
        self.serialPort.close()
        self.serialPort.open()
        encoderData = np.array([0])
        queueData = np.array([0])
        #serialLock.release()

    def setEncoderCount(self, count):
        # self.serialPort.write('R'.encode('utf-8'))
        # Calculate number of characters in set command
        numBytes = math.floor(round(math.log(count, 10),6)) + 1
        commandStr = 'S' + str(numBytes) + str(count) + '\n'
        print('setting' + commandStr)
        self.serialPort.write(commandStr.encode('utf-8'))
        readData = ''
        while 'S&' not in readData:
            #print("waiting to read")
            readData += self.serialPort.read(1).decode("utf-8").strip()
        print('Successful set of encoder count to ' + str(count))

    def requestEncoderCount(self):
        #print('requesting encoder count')
        self.serialPort.write('G'.encode('utf-8'))
        #time.sleep(0.1)
        readData = ''
        while 'C&' not in readData:
            #print("waiting to read")
            readData += self.serialPort.read(1).decode("utf-8")
        #print('Successful get of encoder count')
        #print(readData.strip())
        return readData.strip()

    def run(self):
        #global encoderData, serialLock
        self.setEncoderCount(self.machine.encoder_init)
        #print('setting encoder count in run')
        #time.sleep(0.1)
        while self._running_thread:
            #serialLock.acquire()
            counts = self.requestEncoderCount()

            #if line and str.isnumeric(axisCounts[0]):
            if 'C&' in counts:
                rx_received_time = time.clock()
                record = dict()
                counts = counts.split(' ')[0:-1]
                #print(axisCounts[0], axisCounts[1])
                # print('got numeric');
                # encoderData = np.vstack((encoderData,int(line)))
                #encoderData = np.vstack((encoderData, axisCounts[0]))
                encoder_counts = np.asarray(list(map(int, counts)))
                print(encoder_counts)

                # Create feedback record that respects encoder calibration
                #record['encoder_feedback_positions'] = np.multiply(encoder_counts-self.machine.encoder_init,self.machine.encoder_scale) + self.machine.encoder_offset
                ## FIXME add encoder scaling function
                record['encoder_feedback_positions'] = np.multiply(encoder_counts - self.machine.encoder_init,
                                                                   self.machine.encoder_scale) + self.machine.machine_zero
                record['serial_received_times'] = rx_received_time
                #machine_feedback_records.append(record)
                self.data_store.appendMachineFeedbackRecords([record])
                print(record['encoder_feedback_positions'])
                #time.sleep(0.1)
            #serialLock.release()
            #time.sleep(0.2)
            #print('running serial')

        #Flag set, shutdown
        print('Closing serial port')
        print('ENCODER INTERFACE: thread shut down')
        self.serialPort.close()
        self._shutdown = True

    def close(self):
        print('setting serial port close flag')
        self._running_thread = False
        #sys.exit()
