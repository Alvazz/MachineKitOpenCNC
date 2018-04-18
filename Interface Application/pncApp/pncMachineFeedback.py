import socket
import threading
import re
import numpy as np
import serial, math, time, sys, select

global machine

class MachineFeedbackListener(threading.Thread):
    global machine
    def __init__(self, socket, machine, machine_controller, data_store):
        super(MachineFeedbackListener, self).__init__()
        self.rsyslog_socket = socket
        self._running = True

        self.machine = machine
        self.machine_controller = machine_controller
        self.rsh_socket = machine_controller.rsh_socket
        self.data_store = data_store
        self.received_data_string = ""
        print('Feedback thread started')

    def run(self):
        while self._running:
            # Process feedback from EMCRSH
            data_available = select.select([self.rsh_socket], [], [], 0.5)
            if data_available[0]:
                string_received = self.rsh_socket.recv(65536).decode("utf-8")
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
                            self.processRSHString(string.strip())
                    #data_to_process = self.received_data_string + split_received_string[0]
                    #complete_data_string = self.received_data_string + split_received_string[0]
                    #print('complete data string ' + complete_data_string + '\n')
                    #self.received_data_string = ''.join(split_received_string[1:])
                    self.received_data_string = ''
                elif string_received:
                    #Incomplete data received
                    print('got incomplete data with received = ' + self.received_data_string + ' and string_received = ' + string_received)
                    self.received_data_string += string_received
                else:
                    #Null string received
                    print('received null string: ' + string_received)
                # print(self.received_data_string)

        #Flag set, shutdown
        print('flagging feedback socket')
        self.machine_controller._running = False
        #self.rsyslog_socket.shutdown(socket.SHUT_RDWR)
        #self.rsyslog_socket.close()

    def processRSHString(self, data_string):
        if any(s in data_string for s in self.machine.rsh_feedback_strings):
            # Check for error first
            if self.machine.rsh_feedback_strings[-1] in data_string:
                # Error in RSH command
                print('RSH error')
                self.handleRSHError()
            if self.machine.rsh_feedback_strings[0] in data_string:
                #Position feedback
                #print('processing position feedback with data string: ' + data_string)
                self.processPositionFeedback(data_string)
            elif self.machine.rsh_feedback_strings[1] in data_string:
                # Buffer Length
                self.processBLFeedback(data_string)
            elif self.machine.rsh_feedback_strings[2] in data_string:
                # Program Status
                print('program status')
                self.machine.status = data_string.split(' ')[1].strip()
                print('set program status to ' + self.machine.status)
            elif self.machine.rsh_feedback_strings[3] in data_string:
                # Machine Mode
                print('mode set')
                self.machine.mode = data_string.split(' ')[1].strip()
                print('set machine mode to ' + self.machine.mode)
        # if any(s in complete_data_string for s in self.machine.rsh_feedback_strings):
        #     # Check for error first
        #     if self.machine.rsh_feedback_strings[-1] in current_data_string:
        #         # Error in RSH command
        #         print('RSH error')
        #         self.handleRSHError()
        #     if self.machine.rsh_feedback_strings[0] in current_data_string:
        #         #Position feedback
        #         self.processPositionFeedback(complete_data_string)
        #     elif self.machine.rsh_feedback_strings[1] in current_data_string:
        #         # Buffer Length
        #         self.processBLFeedback(complete_data_string)
        #     elif self.machine.rsh_feedback_strings[2] in current_data_string:
        #         # Program Status
        #         print('program status')
        #         self.machine.status = complete_data_string.split(' ')[1].strip()
        #         print('set program status to ' + self.machine.status)
        #     elif self.machine.rsh_feedback_strings[3] in current_data_string:
        #         # Machine Mode
        #         print('mode set')
        #         self.machine.mode = complete_data_string.split(' ')[1].strip()
        #         print('set machine mode to ' + self.machine.mode)

    def processBLFeedback(self, data_string):
        #print('trying to parse buffer level feedback string: ' + data_string)
        m = re.search(self.machine.rsh_feedback_strings[1] + '(\d+)', data_string)
        self.rsh_buffer_length = int(m.group(1))
        self.data_store.appendMachineControlRecords([dict([('highres_tcq_length', self.rsh_buffer_length)])])

    def processPositionFeedback(self, data_string):
        machine_feedback_records = []
        samples = data_string.strip('*').strip('|').split('*')
        #samples = data_string.split('*')[1:-1]
        feedback_num_points = len(samples)
        #print('the samples are:: ')
        #print(samples)
        #print('the number of points is ' + str(feedback_num_points))

        stepgen_feedback_positions = np.zeros([feedback_num_points,self.machine.servo_log_num_axes])
        machine_clock_times = np.zeros([1,feedback_num_points])
        for sample_num in range(0,feedback_num_points):
            sample = samples[sample_num].strip('|').split('|')
            machine_clock_times[0,sample_num] = float(sample[0])
            #print('the sample is: ')
            #print(sample)
            #print(stepgen_feedback_positions)
            stepgen_feedback_positions[sample_num,:] = np.asarray([float(s) for s in sample[1:]])+np.asarray(self.machine.axis_offsets)
            #np.vstack((stepgen_feedback_positions, sample.split('|')[1:-1]))

        record = dict()
        record['machine_clock_times'] = machine_clock_times
        record['stepgen_feedback_positions'] = stepgen_feedback_positions
        #print('logging position feedback')
        # if '0: tc' in data_string:
        #     parsed_string = re.search('tcqLen(.+?)T(.+?)dC(.+?):(.+)', data_string)
        #     if parsed_string:
        #
        #
        #         feedback_num_points = data_string.count('&')
        #         tcq_length = float(parsed_string.group(1))
        #         delta_thread_cycles = float(parsed_string.group(2))
        #         delta_machine_clock = float(parsed_string.group(3))
        #
        #         data_points = parsed_string.group(4).strip()
        #         data_points = re.sub('S[0-9]*:', '', data_points)
        #
        #         # Each time sample delimited with &
        #         samples = data_points.split('&')
        #         coords = [sample.split('|') for sample in samples]
        #         coords = np.asarray(
        #             [[float(coord) for coord in coords[index][:-1]] for index in range(len(coords[:-1]))])
        #
        #         print(self.machine.axis_offsets)
        #         commanded_joint_positions = coords[:, 0:5] + self.machine.axis_offsets
        #         stepgen_feedback_positions = coords[:, 5:] + self.machine.axis_offsets
        #
        #         # Store parsed data
        #         record['machine_time_delta'] = delta_machine_clock
        #         record['machine_times_interpolated'] = np.linspace(delta_machine_clock / feedback_num_points,
        #                                                            delta_machine_clock, feedback_num_points)
        #         record['machine_tcq_length'] = tcq_length
        #         record['commanded_joint_positions'] = commanded_joint_positions
        #         record['stepgen_feedback_positions'] = stepgen_feedback_positions
        #         record['rt_thread_num_executions_delta'] = delta_thread_cycles

        #machine_feedback_records.append(record)
        self.data_store.appendMachineFeedbackRecords([record])
        return machine_feedback_records
        # machine_feedback_records = []
        # if '0: tc' in machine_data_string:
        #     parsed_string = re.search('tcqLen(.+?)T(.+?)dC(.+?):(.+)', machine_data_string)
        #     if parsed_string:
        #         record = dict()
        #
        #         feedback_num_points = machine_data_string.count('&')
        #         tcq_length = float(parsed_string.group(1))
        #         delta_thread_cycles = float(parsed_string.group(2))
        #         delta_machine_clock = float(parsed_string.group(3))
        #
        #         data_points = parsed_string.group(4).strip()
        #         data_points = re.sub('S[0-9]*:', '', data_points)
        #
        #         #Each time sample delimited with &
        #         samples = data_points.split('&')
        #         coords = [sample.split('|') for sample in samples]
        #         coords = np.asarray([[float(coord) for coord in coords[index][:-1]] for index in range(len(coords[:-1]))])
        #
        #         print(self.machine.axis_offsets)
        #         commanded_joint_positions = coords[:,0:5]+self.machine.axis_offsets
        #         stepgen_feedback_positions = coords[:,5:]+self.machine.axis_offsets
        #
        #         #Store parsed data
        #         record['machine_time_delta'] = delta_machine_clock
        #         record['machine_times_interpolated'] = np.linspace(delta_machine_clock/feedback_num_points,
        #                                                            delta_machine_clock,feedback_num_points)
        #         record['machine_tcq_length'] = tcq_length
        #         record['commanded_joint_positions'] = commanded_joint_positions
        #         record['stepgen_feedback_positions'] = stepgen_feedback_positions
        #         record['rt_thread_num_executions_delta'] = delta_thread_cycles
        #
        #         machine_feedback_records.append(record)
        # return machine_feedback_records

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
        while self._running:
            #serialLock.acquire()
            #print('running')
            counts = self.requestEncoderCount()
            #time.sleep(0.1)
            #print(counts)

            #if line and str.isnumeric(axisCounts[0]):
            if 'C&' in counts:
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
                record['encoder_feedback_positions'] = np.multiply(encoder_counts - self.machine.encoder_init,
                                                                   self.machine.encoder_scale) + self.machine.machine_zero
                #machine_feedback_records.append(record)
                self.data_store.appendMachineFeedbackRecords([record])
                print(record['encoder_feedback_positions'])
                #time.sleep(0.1)
            #serialLock.release()
            #time.sleep(0.2)
            #print('running serial')

        #Flag set, shutdown
        print('Closing serial port')
        self.serialPort.close()

    def close(self):
        print('setting serial port close flag')
        self._running = False
        #sys.exit()
