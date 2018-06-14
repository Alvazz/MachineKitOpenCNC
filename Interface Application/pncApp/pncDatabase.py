import pncLibrary
from multiprocessing import Process, Event, Lock, Queue, current_process#Value#, Lock
#from multiprocessing.synchronize import Event
from multiprocessing.managers import NamespaceProxy
from threading import Thread, current_thread#, Event as threadEvent
import queue, time, datetime, numpy as np

# Store feedback data from other modules
# A Machine Feedback record is of the following form:
#    a) A time stamp T of when the record is generated.
#    b) A set of n commanded joint space positions before T
#    c) A set of n stepgen feedback positions before T
#    d) n time values approximating when each stepgen feedback point was generated
#    e) A snapshot of the tcq length at T

# class DatabaseCommand():
#     def __init__(self, command_type, data, timestamp):
#         self.input_output
#         self.command_type = command_type
#         self.timestamp = timestamp
#         self.data = data

class Record():
    def __init__(self, data_type, data, timestamp):
        self.data_type = data_type
        self.timestamp = timestamp
        self.data = data

# class ReturnRecord(Record):
#     def __init__(self, record_id, data):
#         super(Record, self).__init__(None, None, None)
#         self.id = record_id
#         self.output_data = data

class TerminalLoggingServer(Thread):
    def __init__(self, machine, synchronizer):
        super(TerminalLoggingServer, self).__init__()
        self.name = "logging_server"
        self.machine = machine
        self.synchronizer = synchronizer
        self.output_directory = self.machine.log_file_output_directory

        self.log_queue = Queue()
        self.startup_event = Event()

    def run(self):
        self.startup_event.set()

class LoggingServer(Thread):
    def __init__(self, machine, synchronizer):
        super(LoggingServer, self).__init__()
        self.name = "logging_server"
        self.machine = machine
        self.synchronizer = synchronizer
        self.output_directory = self.machine.log_file_output_directory

        self.log_queue = Queue()
        self.startup_event = Event()

    def run(self):
        try:
            self.log_file_handle = open(self.output_directory + datetime.datetime.now().strftime("%Y.%m.%d-%H.%M.%S") + '.txt', 'w')
            self.startup_event.set()
            pncLibrary.printTerminalString(self.machine.thread_launch_string, current_process().name, self.name)
        except Exception as log_open_error:
            print('Log file open error: ' + str(log_open_error))
            return

        while self.synchronizer.t_run_logger_event.is_set():
            #FIXME don't use spinlock
            try:
                log_time, log_message = self.log_queue.get(True, self.machine.thread_queue_wait_timeout)
                self.log_file_handle.write(str(log_time) + ': ' + str(log_message))
                self.log_file_handle.flush()
            except:
                pass

        self.log_file_handle.close()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_terminate_string, current_process().name,
                                                     self.name)


class Puller(Thread):
    def __init__(self, machine, synchronizer, data_store):
        super(Puller, self).__init__()
        self.name = "database_puller"
        self.machine = machine
        self.synchronizer = synchronizer
        self.data_store = data_store
        self.output_queue = synchronizer.q_database_output_queue_proxy
        self.data_store_lock = synchronizer.db_data_store_lock

        self.pull_queue = Queue()
        self.startup_event = Event()

    def run(self):
        self.startup_event.set()
        pncLibrary.printTerminalString(self.machine.thread_launch_string, current_process().name, self.name)
        while self.synchronizer.t_run_puller_event.is_set():
            try:
                pull_request = self.pull_queue.get(True, self.machine.thread_queue_wait_timeout)
                output_data = self.pull(pull_request[0], pull_request[1], pull_request[2])
                self.output_queue.put(output_data)
            except:
                pass

        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         self.machine.thread_terminate_string, current_process().name,
                                                         self.name)
            # if not self.pull_queue.empty():
            #     pull_request = self.pull_queue.get()
            #     #with self.data_store_lock:
            #     output_data = self.pull(pull_request[0], pull_request[1], pull_request[2])
            #     self.output_queue.put(output_data)
            #     #self.pull_queue.task_done()

    def pull(self, data_types, start_indices, end_indices):
        #FIXME return a success flag for each data item returned, also this is gross
        # if type(data_types) is not list:
        #     data_types, start_indices, end_indices = [[d] for d in [data_types, start_indices, end_indices]]
        data_types, start_indices, end_indices = self.formatPullRequest(data_types, start_indices, end_indices)
        return_data = []
        success_flag = True
        #self.synchronizer.db_data_store_lock.acquire()
        with self.data_store_lock:
            for k in range(0,len(data_types)):
                data_type = data_types[k]
                data_array = self.data_store.lookupDataType(data_type.upper())
                start_index = start_indices[k]
                end_index = end_indices[k]
                if np.shape(data_array)[0] == 0:
                    #print('the shape is ' + str(np.shape(data_array)))
                    #print('data array for type ' + str(data_type) + ' is empty')
                    if self.machine.rsh_buffer_level > 0:
                        print('pull break')
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

        #if None in return_data:
        try:
            #FIXME what is this shit
            if any([d is None for d in return_data]):
                if success_flag is True:
                    print('break')
        except:
            print('break')
        return (success_flag, return_data)

    def formatPullRequest(self, data_types, start_indices, end_indices):
        if type(data_types) is not list:
            data_types = [data_types]
        if type(start_indices) is not list:
            start_indices = [start_indices]
        if type(end_indices) is not list:
            end_indices = [end_indices]
        return data_types, start_indices, end_indices

class Pusher(Thread):
    def __init__(self, machine, synchronizer, data_store):
        super(Pusher, self).__init__()
        self.name = "database_pusher"
        self.machine = machine
        self.synchronizer = synchronizer
        self.data_store = data_store
        self.data_store_lock = synchronizer.db_data_store_lock

        self.push_queue = Queue()
        self.startup_event = Event()

    def run(self):
        self.startup_event.set()
        pncLibrary.printTerminalString(self.machine.thread_launch_string, current_process().name, self.name)
        while self.synchronizer.t_run_pusher_event.is_set():
            try:
                push_request = self.push_queue.get(True, self.machine.thread_queue_wait_timeout)
                # if 'ENCODER_FEEDBACK_POSITIONS' in push_request[0]:
                #     print('break')
                self.push(push_request)
            except:
                pass

        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_terminate_string, current_process().name,
                                                     self.name)

    def push(self, records):
        records = self.formatPushRequest(records)
        self.appendMachineFeedbackRecords(records)

    def formatPushRequest(self, records):
        records_upper = []
        for record in records:
            record_upper = {}
            for key, value in record.items():
                # if type(value) != np.ndarray:
                #     value = np.array([[value]])
                #if len(np.shape(value)) < 2
                record_upper[key.upper()] = value
                # if key == 'ENCODER_FEEDBACK_POSITIONS':
                #     print('break')
            records_upper.append(record_upper)
        return records_upper

    def appendMachineFeedbackRecords(self, records):
        with self.data_store_lock:
            for record in records:
                for key, value in record.items():
                    try:
                        #FIXME this is going to get very slow
                        setattr(self.data_store, key, np.append(getattr(self.data_store, key),value,0))
                    except Exception as error:
                        print("Feedback pusher could not append data with type ID: " + str(key) + ', had error: ' + str(error))

class StateManipulator(Thread):
    def __init__(self, machine, synchronizer):
        super(StateManipulator, self).__init__()
        self.name = "machine_state_manipulator"
        self.machine = machine
        self.synchronizer = synchronizer
        self.machine_state_lock = synchronizer.db_machine_state_lock

        self.state_change_queue = Queue()
        self.startup_event = Event()

    def run(self):
        self.startup_event.set()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.thread_launch_string, current_process().name, self.name)
        #while True:
            #pass

class DatabaseServer(Process):
    def __init__(self, machine, synchronizer):
        super(DatabaseServer, self).__init__()
        self.name = "database"
        self.main_thread_name = self.name + ".MainThread"
        self.machine = machine
        self.synchronizer = synchronizer

        self.command_queue = self.synchronizer.q_database_command_queue_proxy
        self.output_queue = self.synchronizer.q_database_output_queue_proxy
        self.data_store_lock = self.synchronizer.db_data_store_lock
        #self.output_buffer = output_value_proxy

        #self.synchronizers = Synchronizers(self.machine._manager)
        self.data_store = DataStore()

    def run(self):
        current_thread().name = self.main_thread_name
        self.waitForDatabaseHelperThreads()
        self.synchronizer.db_startup_event.set()

        self.synchronizer.process_start_signal.wait()
        time.clock()

        if self.synchronizer.p_enable_database_event.is_set():
            self.synchronizer.p_run_database_event.wait()
            while self.synchronizer.p_run_database_event.is_set():
                #First handle incoming commands
                if not self.command_queue.empty():
                    command = self.command_queue.get()
                    self.handleCommand(command)

                # if not self.push_queue.empty:
                #     records = self.record_queue.get()
                #     #FIXME what about time stamp?
                #     self.appendMachineFeedbackRecords([records.data])
                #     self.record_queue.task_done()
                self.updateMachineState()
            self.synchronizer.t_run_pusher_event.clear()
            self.pusher.join()

            self.synchronizer.t_run_puller_event.clear()
            self.puller.join()

            self.synchronizer.t_run_state_manipulator_event.clear()
            self.state_manipulator.join()

            self.synchronizer.t_run_logger_event.clear()
            self.logging_server.join()
                # position_update = self.pull('STEPGEN_FEEDBACK_POSITIONS', -1, None)
                # buffer_level_update = self.pull('HIGHRES_TC_QUEUE_LENGTH', -1, None)
                # if position_update[0]:
                #     self.machine.current_position = position_update[1][0][0].tolist()
                #     self.synchronizers.initial_position_set_event.set()
                # if buffer_level_update[0]:
                #     self.machine.rsh_buffer_level = int(buffer_level_update[1][0][0].item())


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

    def waitForDatabaseHelperThreads(self):
        self.pusher = Pusher(self.machine, self.synchronizer, self.data_store)
        self.puller = Puller(self.machine, self.synchronizer, self.data_store)
        self.state_manipulator = StateManipulator(self.machine, self.synchronizer)
        self.logging_server = LoggingServer(self.machine, self.synchronizer)

        self.state_manipulator.start()
        self.pusher.start()
        self.puller.start()
        self.logging_server.start()

        self.state_manipulator.startup_event.wait()
        self.pusher.startup_event.wait()
        self.puller.startup_event.wait()
        self.logging_server.startup_event.wait()

    def handleCommand(self, command):
        #FIXME do I need separate pusher/puller threads?
        if command.command_type == 'push':
            # records_upper = {}
            # for key, value in command.data.items():
            #     records_upper[key.upper()] = value
            self.pusher.push_queue.put(command.data)

            #FIXME this is a band-aid, perhaps the database should only be started once clocks are synced
            # if self.synchronizer.mc_clock_sync_event.is_set() or 1:
            #     # Only push data to DB after clocks have been synced
            #     self.appendMachineFeedbackRecords([records_upper])
            #     #self.record_queue.put(Record(records_upper, time.clock()))

        elif command.command_type == 'pull':
            data_types = command.data
            start_indices, end_indices = command.command_parameters
            self.puller.pull_queue.put((data_types, start_indices, end_indices))
            #self.synchronizer.database_output_queue_proxy.put(self.pull(data_types, start_indices, end_indices))

        elif command.command_type == "machine_model_update":
            #setattr(self.machine,command.data) = command.data
            pass

        elif command.command_type == 'log':
            self.logging_server.log_queue.put((command.time, command.data))

    # def lockedPull(self, data_types, start_indices, end_indices):
    #     with self.synchronizer.db_data_store_lock:
    #         return self.puller.pull(data_types, start_indices, end_indices)

    def updateMachineState(self):
        position_update = self.puller.pull('STEPGEN_FEEDBACK_POSITIONS', -1, None)
        buffer_level_update = self.puller.pull('HIGHRES_TC_QUEUE_LENGTH', -1, None)
        if position_update[0]:
            self.machine.current_position = position_update[1][0][0].tolist()
            self.synchronizer.mc_initial_position_set_event.set()
        if buffer_level_update[0]:
            self.machine.rsh_buffer_level = int(buffer_level_update[1][0][0].item())

#     def push(self, records):
#         #FIXME implement type to log to file
#         #Upper all keys
#         records_upper = {}
#         #for record_key in records:
# #            record_upper = {}
#         for key, value in records.items():
#             records_upper[key.upper()] = value
#         #records_upper.append(record_upper)
#         if self.machine.clock_event.isSet():
#             #Only push data to DB after clocks have been synced
#             self.record_queue.put(Record(records_upper, time.clock()))

    # def pull(self, data_types, start_indices, end_indices):
    #     #FIXME return a success flag for each data item returned
    #     if type(data_types) is not list:
    #         data_types, start_indices, end_indices = [[d] for d in [data_types, start_indices, end_indices]]
    #
    #     return_data = []
    #     success_flag = True
    #     #self.synchronizer.db_data_store_lock.acquire()
    #     with self.synchronizer.db_data_store_lock:
    #         for k in range(0,len(data_types)):
    #             data_type = data_types[k]
    #             data_array = self.data_store.lookupDataType(data_type.upper())
    #             start_index = start_indices[k]
    #             end_index = end_indices[k]
    #             if np.shape(data_array)[0] == 0:
    #                 #print('the shape is ' + str(np.shape(data_array)))
    #                 #print('data array for type ' + str(data_type) + ' is empty')
    #                 if self.machine.rsh_buffer_level > 0:
    #                     print('pull break')
    #                 #return_data.append(None)
    #                 return_data.append(np.empty((0,data_array.shape[1])))
    #
    #                 success_flag = success_flag and False
    #             elif (start_index or 0) >= np.shape(data_array)[0] or (end_index or 0) > np.shape(data_array)[0]:
    #                 #print('data index for type ' + str(data_type) + ' out of range')
    #                 #return_data.append(None)
    #                 return_data.append(np.empty((0, data_array.shape[1])))
    #                 success_flag = success_flag and False
    #             else:
    #                 return_data.append(data_array[start_index:end_index,:])
    #                 if len(data_array[start_index:end_index,:]) == 0:
    #                     print('break')
    #                 success_flag = success_flag and True
    #
    #
    #     #if None in return_data:
    #     try:
    #         if any([d is None for d in return_data]):
    #             if success_flag is True:
    #                 print('break')
    #     except:
    #         print('break')
    #     return (success_flag, return_data)

        # elif type(data_types) is str:
        #     #FIXME will this work?
        #     data_array = self.data_store.lookupDataType(data_types.upper())
        #     if start_indices > np.shape(data_array)[0] or end_indices > np.shape(data_array)[0]:
        #         print('data indices out of range')
        #         return None
        #
        #     self.db_data_store_lock.acquire()
        #     data = data_array[start_index:end_index,:]
        #     self.db_data_store_lock.release()
        #     return data

    # def appendMachineFeedbackRecords(self, records):
    #     self.synchronizer.db_data_store_lock.acquire()
    #     for record in records:
    #         if 'RTAPI_FEEDBACK_INDICES' in record:
    #             self.data_store.RTAPI_FEEDBACK_INDICES = np.vstack(
    #                 (self.data_store.RTAPI_FEEDBACK_INDICES, record['RTAPI_FEEDBACK_INDICES']+len(self.data_store.RTAPI_FEEDBACK_INDICES)))
    #
    #         if 'commanded_joint_positions' in record:
    #             self.data_store.commanded_joint_positions = np.vstack(
    #                 (self.data_store.commanded_joint_positions, record['commanded_joint_positions']))
    #             ## FIXME increment machine feedback number of records if ANY of these, except encoder data, is provided
    #             #self.data_store.machine_feedback_num_records += 1
    #
    #         if 'STEPGEN_FEEDBACK_POSITIONS' in record:
    #             self.data_store.STEPGEN_FEEDBACK_POSITIONS = np.vstack(
    #                 (self.data_store.STEPGEN_FEEDBACK_POSITIONS, record['STEPGEN_FEEDBACK_POSITIONS']))
    #
    #         if 'ENCODER_FEEDBACK_POSITIONS' in record:
    #             self.data_store.ENCODER_FEEDBACK_POSITIONS = np.vstack(
    #                 (self.data_store.ENCODER_FEEDBACK_POSITIONS, record['ENCODER_FEEDBACK_POSITIONS']))
    #             #print('current encoder record is ' + str(self.data_store.encoder_feedback_num_records))
    #             #self.data_store.encoder_feedback_num_records += 1
    #
    #         if 'HIGHRES_TC_QUEUE_LENGTH' in record:
    #             self.data_store.HIGHRES_TC_QUEUE_LENGTH = np.vstack(
    #                 (self.data_store.HIGHRES_TC_QUEUE_LENGTH, record['HIGHRES_TC_QUEUE_LENGTH']))
    #
    #         if 'machine_clock_times' in record:
    #             self.data_store.machine_clock_times = np.vstack((self.data_store.machine_clock_times, record['machine_clock_times']))
    #
    #         if 'machine_time_delta' in record:
    #             self.data_store.machine_time_delta = np.append(self.data_store.machine_time_delta, record['machine_time_delta'])
    #
    #         if 'MACHINE_TIMES_INTERPOLATED' in record:
    #             self.data_store.MACHINE_TIMES_INTERPOLATED = np.append(self.data_store.MACHINE_TIMES_INTERPOLATED,
    #                                                                    record['MACHINE_TIMES_INTERPOLATED'])
    #
    #         if 'machine_tcq_length' in record:
    #             self.data_store.machine_tc_queue_length = np.append(self.data_store.machine_tc_queue_length, record['machine_tcq_length'])
    #
    #         if 'rt_thread_num_executions_delta' in record:
    #             self.data_store.rt_thread_num_executions_delta = np.append(self.data_store.rt_thread_num_executions_delta,
    #                                                             record['rt_thread_num_executions_delta'])
    #
    #         if 'machine_time_delta' in record:
    #             self.data_store.machine_running_time += record['machine_time_delta']
    #
    #         if 'LOWFREQ_ETHERNET_RECEIVED_TIMES' in record:
    #             self.data_store.LOWFREQ_ETHERNET_RECEIVED_TIMES = np.vstack(
    #                 (self.data_store.LOWFREQ_ETHERNET_RECEIVED_TIMES, record['LOWFREQ_ETHERNET_RECEIVED_TIMES']))
    #
    #         if 'RTAPI_CLOCK_TIMES' in record:
    #             #print('inserting RTAPI_clock_times')
    #             self.data_store.RTAPI_CLOCK_TIMES = np.vstack((self.data_store.RTAPI_CLOCK_TIMES, record['RTAPI_CLOCK_TIMES']))
    #
    #         if 'HIGHFREQ_ETHERNET_RECEIVED_TIMES' in record:
    #             self.data_store.HIGHFREQ_ETHERNET_RECEIVED_TIMES = np.vstack(
    #                 (self.data_store.HIGHFREQ_ETHERNET_RECEIVED_TIMES, record['HIGHFREQ_ETHERNET_RECEIVED_TIMES']))
    #
    #         if 'RSH_CLOCK_TIMES' in record:
    #             self.data_store.RSH_CLOCK_TIMES = np.vstack((self.data_store.RSH_CLOCK_TIMES, record['RSH_CLOCK_TIMES']))
    #
    #         if 'SERIAL_RECEIVED_TIMES' in record:
    #             self.data_store.SERIAL_RECEIVED_TIMES = np.vstack(
    #                 (self.data_store.SERIAL_RECEIVED_TIMES, record['SERIAL_RECEIVED_TIMES']))
    #
    #         if 'NETWORK_PID_DELAY' in record:
    #             self.data_store.network_PID_delays = np.vstack((self.data_store.network_PID_delays, record['NETWORK_PID_DELAY']))
    #
    #     self.synchronizer.db_data_store_lock.release()


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
        self.machine_tc_queue_length = np.empty((0, 1), float)
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

        # self.data_handles = [self.RTAPI_FEEDBACK_INDICES, self.commanded_joint_positions,
        #                      self.STEPGEN_FEEDBACK_POSITIONS, self.ENCODER_FEEDBACK_POSITIONS,
        #                      self.HIGHRES_TC_QUEUE_LENGTH, self.RTAPI_CLOCK_TIMES, self.LOWFREQ_ETHERNET_RECEIVED_TIMES,
        #                      self.HIGHFREQ_ETHERNET_RECEIVED_TIMES]

    def lookupDataType(self, data_type):
        data_type = data_type.upper()
        if data_type in self.data_descriptors:
            return getattr(self,data_type)
        else:
            print('data does not exist')
            return None

class DatabaseServerProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')

class DatabaseOutputProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')

class SynchronizersProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')

