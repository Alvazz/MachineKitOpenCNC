import socket
import threading
import re
import numpy as np
import serial, math, time

global machine

class MachineFeedbackListener(threading.Thread):
    global machine
    def __init__(self, conn, machine, data_store):
        super(MachineFeedbackListener, self).__init__()
        self.conn = conn
        self.data_store = data_store
        self.machine = machine
        self.received_data_string = ""
        print('Feedback thread started')

    def run(self):
        while True:
            (bytes_received, rec_address) = self.conn.recvfrom(65536)
            string_received = bytes_received.decode("utf-8")
            self.received_data_string += string_received

            # A complete record of machine data has been received
            if self.received_data_string.endswith(u"*|"):
                machine_feedback_records = self.processMachineDataString(self.received_data_string)
                self.data_store.appendMachineFeedbackRecords(machine_feedback_records)
                self.received_data_string = ""

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
        self.conn.shutdown(socket.SHUT_RDWR)
        self.conn.close()


#serialLock = threading.Lock()
class SerialInterface(threading.Thread):
    def __init__(self, machine, data_store):
        super(SerialInterface, self).__init__()
        # self.serialPort = serial.Serial('COM9', 115200)
        self.serialPort = serial.Serial('COM12', 250000)
        self.data_store = data_store
        self.machine = machine
        print('serialInterface started')

    def read(self):
        line = self.serialPort.readline()
        return line

    ##    def write(self, data):
    ##        line = self.serialPort.readline()
    ##        return line

    def reset(self):
        global encoderData, serialLock
        serialLock.acquire()
        self.serialPort.close()
        self.serialPort.open()
        encoderData = np.array([0])
        queueData = np.array([0])
        #serialLock.release()

    def setEncoderCount(self, count):
        # self.serialPort.write('R'.encode('utf-8'))
        # Calculate number of characters in set command
        numBytes = math.floor(math.log(count, 10)) + 1
        commandStr = 'S' + str(numBytes) + str(count)
        self.serialPort.write(commandStr.encode('utf-8'))

    def requestEncoderCount(self):
        self.serialPort.write('G'.encode('utf-8'))

    def run(self):
        #global encoderData, serialLock
        self.setEncoderCount(1000)
        while True:
            #serialLock.acquire()
            self.requestEncoderCount()
            line = self.read().decode("utf-8").strip()
            print(line)
            axisCounts = line.split(' ')
            # print(axisCounts)
            if line and str.isnumeric(axisCounts[0]):
                record = dict()
                #print(axisCounts[0], axisCounts[1])
                # print('got numeric');
                # encoderData = np.vstack((encoderData,int(line)))
                #encoderData = np.vstack((encoderData, axisCounts[0]))
                encoder_counts = np.asarray(list(map(int, axisCounts)))
                print(encoder_counts)

                # Create feedback record that respects encoder calibration
                record['encoder_feedback_positions'] = np.multiply(encoder_counts,self.machine.encoder_scale) + self.machine.encoder_offset
                #machine_feedback_records.append(record)
                self.data_store.appendMachineFeedbackRecords([record])
                time.sleep(0.1)
            #serialLock.release()
            time.sleep(0.04)