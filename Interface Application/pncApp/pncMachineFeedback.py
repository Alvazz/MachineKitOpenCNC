import socket
import threading
import re
import numpy as np
import serial, math, time, sys, select

global machine

class MachineFeedbackListener(threading.Thread):
    global machine
    def __init__(self, conn, machine, data_store):
        super(MachineFeedbackListener, self).__init__()
        self.conn = conn
        self._running = True

        self.data_store = data_store
        self.machine = machine
        self.received_data_string = ""
        print('Feedback thread started')

    def run(self):
        while self._running:
            data_available = select.select([self.conn], [], [], 0.5)
            #print('waiting on select')
            if data_available[0]:
                (bytes_received, rec_address) = self.conn.recvfrom(65536)
                string_received = bytes_received.decode("utf-8")
                self.received_data_string += string_received

                # A complete record of machine data has been received
                if self.received_data_string.endswith(u"*|"):
                    print('received machine feedback')
                    machine_feedback_records = self.processMachineDataString(self.received_data_string)
                    self.data_store.appendMachineFeedbackRecords(machine_feedback_records)
                    self.received_data_string = ""

        #Flag set, shutdown
        self.conn.shutdown(socket.SHUT_RDWR)
        self.conn.close()

    def processMachineDataString(self, machine_data_string):
        machine_feedback_records = []
        if '0: tc' in machine_data_string:
            parsed_string = re.search('tcqLen(.+?)T(.+?)dC(.+?):(.+)', machine_data_string)
            if parsed_string:
                record = dict()

                feedback_num_points = machine_data_string.count('&')
                tcq_length = float(parsed_string.group(1)) 
                delta_thread_cycles = float(parsed_string.group(2))
                delta_machine_clock = float(parsed_string.group(3))

                data_points = parsed_string.group(4).strip()
                data_points = re.sub('S[0-9]*:', '', data_points)
                
                #Each time sample delimited with &
                samples = data_points.split('&')
                coords = [sample.split('|') for sample in samples]
                coords = np.asarray([[float(coord) for coord in coords[index][:-1]] for index in range(len(coords[:-1]))])
                
                print(self.machine.axis_offsets)
                commanded_joint_positions = coords[:,0:5]+self.machine.axis_offsets
                stepgen_feedback_positions = coords[:,5:]+self.machine.axis_offsets

                #Store parsed data
                record['machine_time_delta'] = delta_machine_clock
                record['machine_times_interpolated'] = np.linspace(delta_machine_clock/feedback_num_points,
                                                                   delta_machine_clock,feedback_num_points)
                record['machine_tcq_length'] = tcq_length              
                record['commanded_joint_positions'] = commanded_joint_positions
                record['stepgen_feedback_positions'] = stepgen_feedback_positions
                record['rt_thread_num_executions_delta'] = delta_thread_cycles

                machine_feedback_records.append(record)
        return machine_feedback_records

    def close(self):
        self._running = True



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
            self.serialPort.close()
            self.serialPort.open()
            print("port was already open, was closed and opened again!")
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
            # print("waiting to read")
            readData += self.serialPort.read(1).decode("utf-8")
        #print('Successful get of encoder count')
        #print(readData.strip())
        return readData.strip()

    def run(self):
        #global encoderData, serialLock
        self.setEncoderCount(self.machine.encoder_init)
        #time.sleep(0.1)
        while self._running:
            #serialLock.acquire()
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
            time.sleep(0.2)

        #Flag set, shutdown
        self.serialPort.close()

    def close(self):
        print('Closing serial port')
        self._running = False

        #sys.exit()
