import numpy as np

# Store feedback data from other modules
# A Machine Feedback record is of the following form:
#    a) A time stamp T of when the record is generated.
#    b) A set of n commanded joint space positions before T
#    c) A set of n stepgen feedback positions before T
#    d) n time values approximating when each stepgen feedback point was generated
#    e) A snapshot of the tcq length at T
class DataStore():
    def __init__(self):
        #Timers for each data source
        self.machine_running_time = 0
        self.encoder_running_time = 0

        #Counters for number of records
        self.machine_feedback_num_records = 0
        self.encoder_feedback_num_records = 0


        ### DATA STORES ###
        #Timers -- time_delta - 1 for each record, times_interpolated 
        self.machine_time_delta = np.zeros(1,dtype=float)
        self.machine_times_interpolated = np.zeros(1,dtype=float)

        #Buffer fill level
        self.machine_tc_queue_length = np.zeros(1,dtype=int)
        self.highres_tc_queue_length = np.zeros(1,dtype=int)

        #Positions from stepgen and encoders
        self.commanded_joint_positions = np.zeros([1,5],dtype=float)

        self.stepgen_feedback_positions = np.zeros([1,5],dtype=float)
        self.encoder_feedback_positions = np.zeros([1,5],dtype=float)

        #Thread counter
        self.rt_thread_num_executions_delta = np.zeros(1,dtype=int)
        
        #Imported command points
        self.imported_axes_points = []


    def appendMachineFeedbackRecords(self, records):
        for record in records:
            if 'commanded_joint_positions' in record:
                self.commanded_joint_positions = np.vstack((self.commanded_joint_positions, record['commanded_joint_positions']))
                ## FIXME increment machine feedback number of records if ANY of these, except encoder data, is provided
                self.machine_feedback_num_records += 1

            if 'stepgen_feedback_positions' in record:
                self.stepgen_feedback_positions = np.vstack((self.stepgen_feedback_positions, record['stepgen_feedback_positions']))

            if 'encoder_feedback_positions' in record:
                self.encoder_feedback_positions = np.vstack((self.encoder_feedback_positions, record['encoder_feedback_positions']))
                self.encoder_feedback_num_records += 1

            if 'machine_time_delta' in record:
                self.machine_time_delta = np.append(self.machine_time_delta, record['machine_time_delta'])

            if 'machine_times_interpolated' in record:
                self.machine_times_interpolated = np.append(self.machine_times_interpolated, record['machine_times_interpolated'])

            if 'machine_tcq_length' in record:
                self.machine_tc_queue_length = np.append(self.machine_tc_queue_length, record['machine_tcq_length'])

            if 'rt_thread_num_executions_delta' in record:
                self.rt_thread_num_executions_delta = np.append(self.rt_thread_num_executions_delta, record['rt_thread_num_executions_delta'])

            if 'machine_time_delta' in record:
                self.machine_running_time += record['machine_time_delta']




    def appendMachineControlRecords(self, records):
        for record in records:
            self.highres_tc_queue_length = np.append(self.highres_tc_queue_length, record['highres_tcq_length'])

    def appendEncoderFeedbackRecords(self, records):
        for record in records:
            pass

