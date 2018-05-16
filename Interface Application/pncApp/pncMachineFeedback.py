import socket
import threading, time
import re
import numpy as np
import serial, math, sys, select, datetime, struct

global machine

class MachineFeedbackListener(threading.Thread):
    global machine
    def __init__(self, socket, machine, machine_controller, data_store):
        super(MachineFeedbackListener, self).__init__()
        self.rsyslog_socket = socket
        self._running_thread = True
        self._shutdown = False

        self.machine = machine
        self.machine_controller = machine_controller
        self.rsh_socket = machine_controller.rsh_socket
        self.data_store = data_store
        self.received_data_string = ""
        self.byte_string = bytearray()
        self.binary_transmission_length = (machine.servo_log_num_axes*machine.servo_log_buffer_size+machine.servo_log_buffer_size)*machine.size_of_feedback_double+1
        print('Feedback thread started')
        self.log_file_handle = open('E:\\SculptPrint\\PocketNC\\OpenCNC\\Interface Application\\pncApp\\Logs\\' + datetime.datetime.now().strftime("%Y.%m.%d-%H.%M.%S") + '.txt', 'w')

    def run(self):
        print('feedback socket is ' + str(self.rsh_socket))
        while self._running_thread:
            # Process feedback from EMCRSH
            data_available = select.select([self.rsh_socket], [], [], 0.5)
            if data_available[0]:
                #Clock when these data were received
                rx_received_time = time.clock()
                if not self.machine.binary_mode:
                    string_received = self.rsh_socket.recv(self.machine.bytes_to_receive).decode("utf-8")
                    complete_data_string = ''
                    if '\n' in string_received or '\r' in string_received:
                        #split_received_strings = string_received.split('\n')
                        split_received_strings = re.split(r'\r|\n', string_received)
                        #print('split received string is: ')
                        #print(split_received_strings)
                        #Complete the first entry of the split string list from incomplete data, probably doesn't actually happen?
                        split_received_strings[0] = self.received_data_string + split_received_strings[0]
                        for string in split_received_strings:
                            #Drop null strings
                            #print('going through the split received strings.')
                            #print(string)
                            if string:
                                self.log_file_handle.write(string + '\n')
                                self.log_file_handle.flush()
                                self.processRSHString(string.strip(),rx_received_time)
                        #data_to_process = self.received_data_string + split_received_string[0]
                        #complete_data_string = self.received_data_string + split_received_string[0]
                        #print('complete data string ' + complete_data_string + '\n')
                        #self.received_data_string = ''.join(split_received_string[1:])

                        # if len(self.received_data_string) > 0:
                        #     print('had incomplete data, parsed ')
                        #     print(split_received_strings)

                        self.received_data_string = ''
                    elif string_received:
                        #Incomplete data received
                        #print('got incomplete data with received = ' + self.received_data_string + ' and string_received = ' + string_received)
                        self.received_data_string += string_received
                    else:
                        #Null string received. RSH disconnected?
                        print('received null string: ' + string_received)
                        print('RSH disconnected...')
                    # print(self.received_data_string)
                else:
                    # Receiving data as binary floats
                    bytes_received = self.rsh_socket.recv(self.machine.bytes_to_receive)
                    self.byte_string.extend(bytes_received)
                    print(len(bytes_received))
                    if len(self.byte_string) >= self.binary_transmission_length:
                        print('byte string length: ' + str(len(self.byte_string)))
                        #Number of bytes in transmission
                        complete_transmission = self.byte_string[:self.binary_transmission_length]
                        #print(str(complete_transmission))
                        unpacked_bytes = struct.unpack('!'+'d'*(self.machine.servo_log_num_axes*self.machine.servo_log_buffer_size+self.machine.servo_log_buffer_size)+'c',complete_transmission)
                        #bytes_received.calcsize('c')
                        print('received: ' + str(unpacked_bytes))
                        self.byte_string = self.byte_string[self.binary_transmission_length:]

        #Flag set, shutdown. Handle socket closure in machine_controller
        print('flagging feedback socket')
        #self.machine_controller._running = False
        self.log_file_handle.close()
        self._shutdown = True
        #self.rsyslog_socket.shutdown(socket.SHUT_RDWR)
        #self.rsyslog_socket.close()

    def processRSHString(self, data_string,rx_received_time):
        ##FIXME detect if RSH crashed
        if any(s in data_string for s in self.machine.rsh_feedback_strings):
            # Check for error first
            if self.machine.rsh_feedback_strings[-1] in data_string:
                # Error in RSH command
                print('RSH error')
                self.handleRSHError()
            elif self.machine.rsh_feedback_strings[0] in data_string:
                # Position Feedback
                #print('processing position feedback with data string: ' + data_string)
                print('processing position feedback')
                #Force logging flag
                if self.machine.logging_mode != 1:
                    print('state machine sync error')
                #self.machine.logging_mode = 1
                if data_string[0] == self.machine.rsh_feedback_strings[0]:
                    #Beginning of transmission is good
                    self.processPositionFeedback(data_string,rx_received_time)
                else:
                    #Incomplete transmission, discard for now FIXME find root cause
                    print('got incomplete beginning of transmission')
                    pass
            elif self.machine.rsh_feedback_strings[1] in data_string:
                # Buffer Length
                self.processBLFeedback(data_string,rx_received_time)
            elif self.machine.rsh_feedback_strings[3] in data_string:
                # Program Status
                print('program status')
                self.machine.status = data_string.split(' ')[1].strip()
                print('set program status to ' + self.machine.status)
                self.machine.status_change_event.set()
            elif self.machine.rsh_feedback_strings[4] in data_string:
                # Machine Mode
                #print('mode set')
                self.machine.mode = data_string.split(' ')[1].strip()
                self.machine.mode_change_event.set()
                #print('set machine mode to ' + self.machine.mode)
            elif self.machine.rsh_feedback_strings[6] in data_string:
                # Servo Log Params
                self.machine.logging_mode = int(data_string.split()[1].strip())
                self.machine.logging_mode_change_event.set()
            elif self.machine.rsh_feedback_strings[7] in data_string:
                # Drive Power
                self.machine.drive_power = self.machine.rsh_feedback_flags.index(data_string.split()[1].strip().upper())
                self.machine.drive_power_change_event.set()
            elif self.machine.rsh_feedback_strings[8] in data_string:
                # Echo
                print('got set echo: ' + data_string)
                self.machine.echo = self.machine.rsh_feedback_flags.index(data_string.split()[1].strip().upper())
                self.machine.echo_change_event.set()
            elif self.machine.rsh_feedback_strings[9] in data_string:
                # Hello
                print('got hello')
                self.machine.connected = 1
                self.machine.connection_change_event.set()
            elif self.machine.rsh_feedback_strings[10] in data_string:
                # Enable
                print('got enable')
                self.machine.linked = 1
                self.machine.link_change_event.set()
            elif self.machine.rsh_feedback_strings[11] in data_string:
                # EStop
                print('got estop: ' + data_string)
                self.machine.estop = self.machine.rsh_feedback_flags.index(data_string.split()[1].strip().upper())
                self.machine.estop_change_event.set()
            else:
                print('received unrecognized string ' + data_string)


    def processBLFeedback(self, data_string, rx_received_time):
        #print('trying to parse buffer level feedback string: ' + data_string)
        #rx_received_time = time.clock()
        m = re.search(self.machine.rsh_feedback_strings[1] + '(\d+)'+self.machine.rsh_feedback_strings[2]+'([+-]?([0-9]*[.])?[0-9]+)', data_string)
        self.rsh_buffer_length = int(m.group(1))
        rsh_clock_time = float(m.group(2))
        #self.data_store.appendMachineControlRecords([dict([('highres_tcq_length', self.rsh_buffer_length)])])
        record = dict()
        record['highres_tcq_length'] = self.rsh_buffer_length
        record['highfreq_ethernet_received_times'] = rx_received_time
        record['rsh_clock_times'] = rsh_clock_time
        self.data_store.appendMachineFeedbackRecords([record])

    def processPositionFeedback(self, data_string, rx_received_time):
        try:
            machine_feedback_records = []
            samples = data_string.strip('*').strip('|').split('*')
            #Handle error for incomplete last entry, FIXME find root cause
            if len(samples[-1].split('|')) < self.machine.servo_log_num_axes + 1:
                print('received imcomplete last entry, discarding')
                samples = samples[:-1]
            #samples = data_string.split('*')[1:-1]
            feedback_num_points = len(samples)
            #print('the samples are:: ')
            #print(samples)
            #print('the number of points is ' + str(feedback_num_points))

            ## FIXME why are these not all column vectors?
            stepgen_feedback_positions = np.zeros([feedback_num_points,self.machine.servo_log_num_axes])
            rtapi_clock_times = np.zeros([feedback_num_points,1])
            received_times_interpolated = np.zeros([feedback_num_points, 1])
            if len(self.data_store.RTAPI_feedback_indices) != 0:
                RTAPI_feedback_indices = np.zeros([feedback_num_points, 1]) + self.data_store.RTAPI_feedback_indices[-1] + 1
            else:
                RTAPI_feedback_indices = np.zeros([feedback_num_points, 1])

            #rx_received_time = time.clock()
            if len(data_string) < 10:
                print('feedback error with data_string = ' + data_string)

            for sample_num in range(0,feedback_num_points):
                sample = samples[sample_num].strip('|').split('|')
                RTAPI_feedback_indices[sample_num, :] += float(sample_num)
                rtapi_clock_times[sample_num,:] = float(sample[0])
                #print('the sample is: ')
                #print(sample)
                #print(stepgen_feedback_positions)
                stepgen_feedback_positions[sample_num,:] = np.asarray([float(s) for s in sample[1:]])+np.asarray(self.machine.axis_offsets)
                ## FIXME use clock delta between samples instead of straight interpolation
                #received_times_interpolated[sample_num,:] = np.hstack([float(sample_num), rx_received_time + self.machine.servo_dt*float(sample_num)])
                ## FIXME if you are going to interpolate, at least do it with the subsample rate...
                received_times_interpolated[sample_num, :] = rx_received_time + self.machine.servo_dt*self.machine.servo_log_sub_sample_rate * float(sample_num)
                self.machine.current_position = stepgen_feedback_positions[sample_num,:]
                #np.vstack((stepgen_feedback_positions, sample.split('|')[1:-1]))

            record = dict()
            record['RTAPI_feedback_indices'] = RTAPI_feedback_indices
            record['rtapi_clock_times'] = rtapi_clock_times
            record['stepgen_feedback_positions'] = stepgen_feedback_positions
            record['lowfreq_ethernet_received_times'] = received_times_interpolated
        except:
            print('had error processing feedback string. The data string is: ' + data_string)

        #machine_feedback_records.append(record)
        self.data_store.appendMachineFeedbackRecords([record])
        return machine_feedback_records

    def handleRSHError(self):
        self.machine.rsh_error = 1

    def resetRSHError(self):
        self.machine.rsh_error = 0

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
