import socket
import threading
import re
import numpy as np

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