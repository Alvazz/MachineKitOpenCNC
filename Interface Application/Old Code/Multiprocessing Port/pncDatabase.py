from multiprocessing import Process, Value#, Lock
from multiprocessing.managers import NamespaceProxy
from threading import Thread, Event as threadEvent
import queue, time, numpy as np

# Store feedback data from other modules
# A Machine Feedback record is of the following form:
#    a) A time stamp T of when the record is generated.
#    b) A set of n commanded joint space positions before T
#    c) A set of n stepgen feedback positions before T
#    d) n time values approximating when each stepgen feedback point was generated
#    e) A snapshot of the tcq length at T

class DatabaseCommand():
    def __init__(self, command_type, data, timestamp):
        self.command_type = command_type
        self.timestamp = timestamp
        self.data = data

class Record():
    def __init__(self, data_type, data, timestamp):
        self.data_type = data_type
        self.timestamp = timestamp
        self.data = data

class DatabaseOutputArea():
    def __init__(self):
        self.database_output = None

class Synchronizers():
    def __init__(self, manager):
        # self.database_input_queue = manager.Queue()
        # self.database_command_queue = manager.Queue()
        # self.database_output_queue = manager.Queue()

        #State Switch Events
        self.connection_change_event = manager.Event()
        self.link_change_event = manager.Event()
        self.echo_change_event = manager.Event()
        self.estop_change_event = manager.Event()
        self.drive_power_change_event = manager.Event()
        self.status_change_event = manager.Event()
        #self.status_change_event.clear()
        self.mode_change_event = manager.Event()
        self.logging_mode_change_event = manager.Event()
        self.comm_mode_change_event = manager.Event()
        self.home_change_event = manager.Event()
        self.all_homed_event = manager.Event()
        self.restore_mode_event = manager.Event()
        self.ping_event = manager.Event()
        self.clock_event = manager.Event()
        self.buffer_level_reception_event = manager.Event()
        self.servo_feedback_reception_event = manager.Event()
        #FIXME implement this
        self.position_change_event = manager.Event()
        self.initial_position_set_event = manager.Event()
        self.encoder_init_event = manager.Event()

        self.data_store_lock = manager.Lock()

class LoggingServer(Thread):
    def __init__(self, machine):
        super(LoggingServer, self).__init__()
        self.machine = machine
        self.output_directory = self.machine.log_file_output_directory
        self.log_queue = queue.Queue()

    def run(self):
        while True:
            pass

class DatabaseServer(Process):
    def __init__(self, machine, push_queue_proxy, command_queue_proxy, output_queue_proxy):
        super(DatabaseServer, self).__init__()

        self.machine = machine
        self.push_queue = push_queue_proxy
        self.command_queue = command_queue_proxy
        self.output_queue = output_queue_proxy
        #self.output_buffer = output_value_proxy

        #self.synchronizers = synchronizers

        self.synchronizers = Synchronizers(self.machine._manager)
        self.data_store = DataStore()

        # Thread locks
        #self.data_store_lock = lock#self.synchronizers.database_lock
        #self.record_queue = queue.Queue()

        #self.synchronizers.database_queue


        self._running_process = True

    def run(self):
        self.logging_server = LoggingServer(self.machine)
        self.logging_server.start()

        while self._running_process:
            #First handle incoming commands
            if not self.command_queue.empty():
                self.handleCommand(self.command_queue.get())

            if not self.push_queue.empty:
                records = self.record_queue.get()
                #FIXME what about time stamp?
                self.appendMachineFeedbackRecords([records.data])
                self.record_queue.task_done()

            position_update = self.pull(['STEPGEN_FEEDBACK_POSITIONS'], [-1], [None])
            buffer_level_update = self.pull(['HIGHRES_TC_QUEUE_LENGTH'], [-1], [None])
            if position_update[0]:
                self.machine.current_position = position_update[1][0][0].tolist()
                self.synchronizers.initial_position_set_event.set()
            if buffer_level_update[0]:
                self.machine.rsh_buffer_level = int(buffer_level_update[1][0][0].item())


        # while self._running_process:
        #     if not self.record_queue.empty():
        #         records = self.record_queue.get()
        #         #FIXME what about time stamp?
        #         self.appendMachineFeedbackRecords([records.data])
        #         self.record_queue.task_done()
        #
        #     #update_data = self.pull(['STEPGEN_FEEDBACK_POSITIONS', 'HIGHRES_TC_QUEUE_LENGTH'],[-1,-1],[None, None])
        #     position_update = self.pull(['STEPGEN_FEEDBACK_POSITIONS'], [-1], [None])
        #     buffer_level_update = self.pull(['HIGHRES_TC_QUEUE_LENGTH'], [-1], [None])
        #     if position_update[0]:
        #         self.machine.current_position = position_update[1][0][0].tolist()
        #         self.machine.initial_position_set_event.set()
        #     if buffer_level_update[0]:
        #         self.machine.rsh_buffer_level = int(buffer_level_update[1][0][0].item())


    def push(self, records):
        #FIXME implement type to log to file
        #Upper all keys
        records_upper = {}
        #for record_key in records:
#            record_upper = {}
        for key, value in records.items():
            records_upper[key.upper()] = value
        #records_upper.append(record_upper)
        if self.machine.clock_event.isSet():
            #Only push data to DB after clocks have been synced
            self.record_queue.put(Record(records_upper, time.clock()))

    def pull(self, data_types, start_indices, end_indices):
        if type(data_types) is list:
            return_data = []
            success_flag = True
            self.synchronizers.data_store_lock.acquire()
            for k in range(0,len(data_types)):
                data_type = data_types[k]
                data_array = self.data_store.lookupDataType(data_type.upper())
                start_index = start_indices[k]
                end_index = end_indices[k]
                if np.shape(data_array)[0] == 0:
                    #print('the shape is ' + str(np.shape(data_array)))
                    #print('data array for type ' + str(data_type) + ' is empty')
                    if self.machine.rsh_buffer_level > 0:
                        print('break')
                    #return_data.append(None)
                    return_data.append(np.empty((0,data_array.shape[1])))

                    success_flag = success_flag and False
                elif (start_index or 0) >= np.shape(data_array)[0] or (end_index or 0) > np.shape(data_array)[0]:
                    #print('data index for type ' + str(data_type) + ' out of range')
                    #return_data.append(None)
                    return_data.append(np.empty((0, data_array.shape[1])))
                    success_flag = success_flag and False
                else:
                    return_data.append(data_array[start_index:end_index,:])
                    if len(data_array[start_index:end_index,:]) == 0:
                        print('break')
                    success_flag = success_flag and True

            self.synchronizers.data_store_lock.release()

            #if None in return_data:
            try:
                if any([d is None for d in return_data]):
                    if success_flag is True:
                        print('break')
            except:
                print('break')
            return (success_flag, return_data)

        # elif type(data_types) is str:
        #     #FIXME will this work?
        #     data_array = self.data_store.lookupDataType(data_types.upper())
        #     if start_indices > np.shape(data_array)[0] or end_indices > np.shape(data_array)[0]:
        #         print('data indices out of range')
        #         return None
        #
        #     self.data_store_lock.acquire()
        #     data = data_array[start_index:end_index,:]
        #     self.data_store_lock.release()
        #     return data

        ##FIXME implement data store manager class for threadlocks, methods, etc
    def appendMachineFeedbackRecords(self, records):
        self.data_store_lock.acquire()
        for record in records:
            if 'RTAPI_FEEDBACK_INDICES' in record:
                self.data_store.RTAPI_FEEDBACK_INDICES = np.vstack(
                    (self.data_store.RTAPI_FEEDBACK_INDICES, record['RTAPI_FEEDBACK_INDICES']+len(self.data_store.RTAPI_FEEDBACK_INDICES)))

            if 'commanded_joint_positions' in record:
                self.data_store.commanded_joint_positions = np.vstack(
                    (self.data_store.commanded_joint_positions, record['commanded_joint_positions']))
                ## FIXME increment machine feedback number of records if ANY of these, except encoder data, is provided
                self.data_store.machine_feedback_num_records += 1

            if 'STEPGEN_FEEDBACK_POSITIONS' in record:
                self.data_store.STEPGEN_FEEDBACK_POSITIONS = np.vstack(
                    (self.data_store.STEPGEN_FEEDBACK_POSITIONS, record['STEPGEN_FEEDBACK_POSITIONS']))

            if 'ENCODER_FEEDBACK_POSITIONS' in record:
                self.data_store.ENCODER_FEEDBACK_POSITIONS = np.vstack(
                    (self.data_store.ENCODER_FEEDBACK_POSITIONS, record['ENCODER_FEEDBACK_POSITIONS']))
                #print('current encoder record is ' + str(self.data_store.encoder_feedback_num_records))
                #self.data_store.encoder_feedback_num_records += 1

            if 'HIGHRES_TC_QUEUE_LENGTH' in record:
                self.data_store.HIGHRES_TC_QUEUE_LENGTH = np.vstack(
                    (self.data_store.HIGHRES_TC_QUEUE_LENGTH, record['HIGHRES_TC_QUEUE_LENGTH']))

            if 'machine_clock_times' in record:
                self.data_store.machine_clock_times = np.vstack((self.data_store.machine_clock_times, record['machine_clock_times']))

            if 'machine_time_delta' in record:
                self.data_store.machine_time_delta = np.append(self.data_store.machine_time_delta, record['machine_time_delta'])

            if 'MACHINE_TIMES_INTERPOLATED' in record:
                self.data_store.MACHINE_TIMES_INTERPOLATED = np.append(self.data_store.MACHINE_TIMES_INTERPOLATED,
                                                                       record['MACHINE_TIMES_INTERPOLATED'])

            if 'machine_tcq_length' in record:
                self.data_store.machine_tc_queue_length = np.append(self.data_store.machine_tc_queue_length, record['machine_tcq_length'])

            if 'rt_thread_num_executions_delta' in record:
                self.data_store.rt_thread_num_executions_delta = np.append(self.data_store.rt_thread_num_executions_delta,
                                                                record['rt_thread_num_executions_delta'])

            if 'machine_time_delta' in record:
                self.data_store.machine_running_time += record['machine_time_delta']

            if 'LOWFREQ_ETHERNET_RECEIVED_TIMES' in record:
                self.data_store.LOWFREQ_ETHERNET_RECEIVED_TIMES = np.vstack(
                    (self.data_store.LOWFREQ_ETHERNET_RECEIVED_TIMES, record['LOWFREQ_ETHERNET_RECEIVED_TIMES']))

            if 'RTAPI_CLOCK_TIMES' in record:
                #print('inserting RTAPI_clock_times')
                self.data_store.RTAPI_CLOCK_TIMES = np.vstack((self.data_store.RTAPI_CLOCK_TIMES, record['RTAPI_CLOCK_TIMES']))

            if 'HIGHFREQ_ETHERNET_RECEIVED_TIMES' in record:
                self.data_store.HIGHFREQ_ETHERNET_RECEIVED_TIMES = np.vstack(
                    (self.data_store.HIGHFREQ_ETHERNET_RECEIVED_TIMES, record['HIGHFREQ_ETHERNET_RECEIVED_TIMES']))

            if 'RSH_CLOCK_TIMES' in record:
                self.data_store.RSH_CLOCK_TIMES = np.vstack((self.data_store.RSH_CLOCK_TIMES, record['RSH_CLOCK_TIMES']))

            if 'SERIAL_RECEIVED_TIMES' in record:
                self.data_store.SERIAL_RECEIVED_TIMES = np.vstack(
                    (self.data_store.SERIAL_RECEIVED_TIMES, record['SERIAL_RECEIVED_TIMES']))

            if 'NETW0RK_PID_DELAY' in record:
                self.data_store.network_PID_delays = np.vstack((self.data_store.network_PID_delays, record['NETWORK_PID_DELAY']))

        self.data_store_lock.release()


class DataStore():
    def __init__(self):
        #Timers for each data source
        self.machine_running_time = 0
        self.encoder_running_time = 0

        #Counters for number of records
        self.machine_feedback_num_records = 0
        self.encoder_feedback_num_records = 0
        self.machine_feedback_written_record_id = -1
        self.encoder_feedback_written_record_id = -1

        ### DATA STORES ###
        #Timers -- time_delta - 1 for each record, times_interpolated 
        #self.machine_time_delta = np.zeros(1,dtype=float)
        #self.machine_clock_times = np.zeros(1,dtype=float)
        #self.machine_times_interpolated = np.zeros(1,dtype=float)
        self.machine_time_delta = np.empty((0,1), float)
        self.machine_clock_times = np.empty((0,1), float)
        self.MACHINE_TIMES_INTERPOLATED = np.empty((0, 1), float)

        #Received time vectors on PC end, would be interesting to correlate with machine time. Do we need tx number here?
        #self.lowfreq_ethernet_received_times = np.zeros(1,dtype=float)
        self.PING_TIMES = np.empty((0, 2), float)
        self.LOWFREQ_ETHERNET_RECEIVED_TIMES = np.empty((0, 1), float)
        self.RTAPI_CLOCK_TIMES = np.empty((0, 1), float)
        #self.highfreq_ethernet_received_times = np.zeros(1, dtype=float)
        self.HIGHFREQ_ETHERNET_RECEIVED_TIMES = np.empty((0, 1), float)
        self.RSH_CLOCK_TIMES = np.empty((0, 1), float)
        #self.SERIAL_RECEIVED_TIMES = np.zeros(1, dtype=float)
        self.SERIAL_RECEIVED_TIMES = np.empty((0, 1), float)

        #Buffer fill level
        self.machine_tc_queue_length = np.empty((0,1), float)
        self.HIGHRES_TC_QUEUE_LENGTH = np.empty((0, 1), float)

        #Positions from stepgen and encoders
        #self.commanded_joint_positions = np.zeros([1,5],dtype=float)
        self.commanded_joint_positions = np.empty((0,5), float)
        self.sent_servo_commands = []

        #self.RTAPI_feedback_indices = np.zeros(1, dtype=float)
        self.RTAPI_FEEDBACK_INDICES = np.empty((0, 1), float)
        #self.STEPGEN_FEEDBACK_POSITIONS = np.zeros([1, 5], dtype=float)
        self.STEPGEN_FEEDBACK_POSITIONS = np.empty((0,5), float)
        #self.ENCODER_FEEDBACK_POSITIONS = np.zeros([1,5],dtype=float)
        #self.stepgen_feedback_positions = np.empty((0,5), float)
        self.ENCODER_FEEDBACK_POSITIONS = np.empty((0, 5), float)

        #Thread counter
        #self.rt_thread_num_executions_delta = np.zeros(1,dtype=int)
        self.rt_thread_num_executions_delta = np.empty((0,1), float)
        
        #Imported command points
        self.imported_axes_points = []

        #Successfully executed moves
        self.completed_moves = []


        # self.data_descriptors = ['RTAPI_feedback_indices', 'commanded_joint_positions', 'stepgen_feedback_postitions',
        #                          'ENCODER_FEEDBACK_POSITIONS', 'HIGHRES_TC_QUEUE_LENGTH',
        #                          'lowfreq_ethernet_received_times', 'highfreq_ethernet_received_times']

        self.data_descriptors = ['RTAPI_FEEDBACK_INDICES', 'COMMANDED_JOINT_POSITIONS', 'STEPGEN_FEEDBACK_POSITIONS',
                                 'ENCODER_FEEDBACK_POSITIONS', 'HIGHRES_TC_QUEUE_LENGTH', 'RTAPI_CLOCK_TIMES',
                                 'LOWFREQ_ETHERNET_RECEIVED_TIMES', 'HIGHFREQ_ETHERNET_RECEIVED_TIMES', 'RSH_CLOCK_TIMES',
                                 'SERIAL_RECEIVED_TIMES', 'ENCODER_FEEDBACK_POSITIONS']

        self.data_handles = [self.RTAPI_FEEDBACK_INDICES, self.commanded_joint_positions,
                             self.STEPGEN_FEEDBACK_POSITIONS, self.ENCODER_FEEDBACK_POSITIONS,
                             self.HIGHRES_TC_QUEUE_LENGTH, self.RTAPI_CLOCK_TIMES, self.LOWFREQ_ETHERNET_RECEIVED_TIMES,
                             self.HIGHFREQ_ETHERNET_RECEIVED_TIMES]

    def lookupDataType(self, data_type):
        data_type = data_type.upper()
        if data_type in self.data_descriptors:
            #return self.RTAPI_clock_times
            #return self.data_handles[self.data_descriptors.index(data_type)]
            return getattr(self,data_type)
        else:
            print('data does not exist')
            return None

class DatabaseServerProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')

class DatabaseOutputProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')

