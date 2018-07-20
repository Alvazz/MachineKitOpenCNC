import pncLibrary
from multiprocessing import Process, Event, Lock, Queue, current_process#Value#, Lock
#from multiprocessing.synchronize import Event
from multiprocessing.managers import NamespaceProxy
from threading import Thread, current_thread#, Event as threadEvent
from queue import Empty
import time, datetime, collections, pickle, numpy as np
import cProfile, pstats

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

# class Record():
#     def __init__(self, data_type, data, timestamp):
#         self.data_type = data_type
#         self.timestamp = timestamp
#         self.data = data

# class ReturnRecord(Record):
#     def __init__(self, record_id, data):
#         super(Record, self).__init__(None, None, None)
#         self.id = record_id
#         self.output_data = data

# class TerminalLoggingServer(Thread):
#     def __init__(self, machine, synchronizer):
#         super(TerminalLoggingServer, self).__init__()
#         self.name = "logging_server"
#         self.machine = machine
#         self.synchronizer = synchronizer
#         self.output_directory = self.machine.log_file_output_directory
#
#         self.log_queue = Queue()
#         self.startup_event = Event()
#
#     def run(self):
#         self.startup_event.set()

class LoggingServer(Thread):
    def __init__(self, parent):
        super(LoggingServer, self).__init__()
        self.name = "logging_server"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer
        self.output_directory = self.machine.log_file_output_directory

        self.log_queue = Queue()
        self.circular_log_buffer = collections.deque(maxlen=20)
        self.startup_event = Event()

    def run(self):
        try:
            self.log_file_handle = open(self.output_directory + datetime.datetime.now().strftime("%Y.%m.%d-%H.%M.%S") + '.txt', 'w')
            self.startup_event.set()
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         self.machine.thread_launch_string, current_process().name,
                                                         self.name)
        except Exception as log_open_error:
            print('Log file open error: ' + str(log_open_error))
            return

        while self.synchronizer.t_run_logging_server_event.is_set():
            #FIXME don't use spinlock
            try:
                log_time, log_message = self.log_queue.get(True, self.machine.thread_queue_wait_timeout)
                self.circular_log_buffer.append(str(log_time) + ': ' + str(log_message))
                self.log_file_handle.write('\n' + str(log_time) + ': ' + str(log_message))
                self.log_file_handle.flush()
            except Empty:
                pass

        self.log_file_handle.close()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_terminate_string, current_process().name,
                                                     self.name)


class Puller(Thread):
    def __init__(self, parent):
        super(Puller, self).__init__()
        self.name = "database_puller"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer
        self.data_store = parent.data_store
        self.output_queue = self.synchronizer.q_database_output_queue_proxy
        self.data_store_lock = self.synchronizer.db_data_store_lock

        self.pull_queue = Queue()
        self.startup_event = Event()

    def run(self):
        self.startup_event.set()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.thread_launch_string, current_process().name, self.name)
        while self.synchronizer.t_run_database_puller_event.is_set():
            try:
                pull_request = self.pull_queue.get(True, self.machine.thread_queue_wait_timeout)
                output_data = self.pull(pull_request[0], pull_request[1], pull_request[2])
                self.output_queue.put(output_data)
            except Empty:
                pass
            except Exception as error:
                print('had pull error: ' + str(error))
                pass

        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         self.machine.thread_terminate_string, current_process().name,
                                                         self.name)

    def pull(self, data_types, start_indices, end_indices):
        #FIXME return a success flag for each data item returned, also this is gross
        # if type(data_types) is not list:
        #     data_types, start_indices, end_indices = [[d] for d in [data_types, start_indices, end_indices]]
        data_types, start_indices, end_indices = self.formatPullRequest(data_types, start_indices, end_indices)
        return_data = []
        #print('pulling %d records' % len(data_types))
        # if len(data_types) > 1:
        #     print('break')
        success_flag = True
        with self.data_store_lock:
            start_time = time.clock()
            for k in range(0,len(data_types)):
                data_type = data_types[k]
                data_array = self.data_store.lookupDataType(data_type.upper())
                start_index = start_indices[k]
                end_index = end_indices[k]
                #FIXME handle if data does not exist, consider returning a dict of the successfully retrieved values
                if data_array is None:
                    return_data.append(np.empty((0, 1)))
                    success_flag = success_flag and False
                elif np.shape(data_array)[0] == 0:
                    if self.machine.current_buffer_level > 0:
                        print('pull break on ' + str(data_type[k]))
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
                        print('break 123')
                    success_flag = success_flag and True

        #if None in return_data:
        try:
            #FIXME what is this shit
            if any([d is None for d in return_data]):
                if success_flag is True:
                    print('break sdf')
        except:
            print('break ghfdg')
        #print('in pull operation ' + str(time.clock() - start_time))
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
    def __init__(self, parent):
        super(Pusher, self).__init__()
        self.name = "database_pusher"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer
        self.data_store = parent.data_store
        self.data_store_lock = self.synchronizer.db_data_store_lock

        self.push_queue = Queue()
        self.startup_event = Event()

    def run(self):
        self.startup_event.set()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_launch_string, current_process().name,
                                                     self.name)
        while self.synchronizer.t_run_database_pusher_event.is_set():
            try:
                push_request = self.push_queue.get(True, self.machine.thread_queue_wait_timeout)
                self.push(push_request)
            except:
                pass

        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_terminate_string, current_process().name,
                                                     self.name)

    def push(self, push_request):
        records = self.formatPushRequest(push_request[0])
        if push_request[1] == 'numpy':
            # if len(records) > 1:
            #     print('multirecord')
            self.appendMachineFeedbackRecords(records)
        elif push_request[1] == 'object':
            self.appendObjects(records)

    def formatPushRequest(self, records):
        records_upper = []
        for record in records:
            record_upper = {}
            for key, value in record.items():
                record_upper[key.upper()] = value
            records_upper.append(record_upper)
        return records_upper

    def appendMachineFeedbackRecords(self, records):
        with self.data_store_lock:
            if len(records) > 2:
                print('appending multi record')
            for record in records:
                for key, value in record.items():
                    try:
                        #FIXME this is going to get very slow
                        setattr(self.data_store, key, np.append(getattr(self.data_store, key),value,0))
                    except AttributeError:
                        pncLibrary.printTerminalString(pncLibrary.printout_database_field_creation_string, key, value.size)
                        setattr(self.data_store, key, np.append(np.empty((0, value.shape[1]), float), value, 0))
                        self.data_store.data_descriptors.append(key)
                    except Exception as error:
                        print("Feedback pusher could not append numpy data with type ID: " + str(key) + ', had error: ' + str(error))

    def appendObjects(self, records):
        with self.data_store_lock:
            for record in records:
                for key, value in record.items():
                    try:
                        #FIXME this is going to get very slow
                        #setattr(self.data_store, key, np.append(getattr(self.data_store, key),value,0))
                        getattr(self.data_store, key).append(value)
                    except AttributeError:
                        pncLibrary.printTerminalString(pncLibrary.printout_database_object_list_creation_string, key, len(records))
                        setattr(self.data_store, key, [value])
                    except Exception as error:
                        print("Feedback pusher could not append object with type ID: " + str(key) + ', had error: ' + str(error))

class StateManipulator(Thread):
    def __init__(self, parent):
        super(StateManipulator, self).__init__()
        self.name = "machine_state_manipulator"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer
        self.machine_state_lock = self.synchronizer.db_machine_state_lock

        self.state_change_queue = Queue()
        self.startup_event = Event()

    def run(self):
        self.startup_event.set()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.thread_launch_string, current_process().name, self.name)

        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_terminate_string, current_process().name,
                                                     self.name)

class DatabaseServer(Process):
    def __init__(self, machine, pipe):
        super(DatabaseServer, self).__init__()
        self.name = "database"
        self.main_thread_name = self.name + ".MainThread"
        self.machine = machine
        self.feed_pipe = pipe

        self.data_store = DataStore(self.machine)

    def run(self):
        current_thread().name = self.main_thread_name
        pncLibrary.getSynchronizer(self, self.feed_pipe)

        pncLibrary.waitForThreadStart(self, Pusher, Puller, StateManipulator, LoggingServer)
        self.synchronizer.db_startup_event.set()

        self.synchronizer.process_start_signal.wait()
        time.clock()

        if self.synchronizer.p_enable_database_event.is_set():
            self.synchronizer.db_successful_start_event.set()
            self.synchronizer.p_run_database_event.wait()
            while self.synchronizer.p_run_database_event.is_set():
                try:
                    command = self.synchronizer.q_database_command_queue_proxy.get(True, pncLibrary.queue_database_command_queue_wait_timeout)
                    self.handleCommand(command)
                except Empty:
                    pass

                self.updateMachineState()
                if self.synchronizer.q_database_command_queue_proxy.qsize() > 0:
                    print('db cmd q size is ' + str(self.synchronizer.q_database_command_queue_proxy.qsize()))


            pncLibrary.waitForThreadStop(self, self.database_pusher, self.database_puller, self.machine_state_manipulator, self.logging_server)

    def handleCommand(self, command):
        #FIXME do I need separate pusher/puller threads?
        if command.command_type == 'push':
            self.database_pusher.push_queue.put((command.data, 'numpy'))

        elif command.command_type == 'push_object':
            self.database_pusher.push_queue.put((command.data, 'object'))

        elif command.command_type == 'pull':
            data_types = command.data
            start_indices, end_indices = command.command_parameters
            self.database_puller.pull_queue.put((data_types, start_indices, end_indices))
            #self.synchronizer.database_output_queue_proxy.put(self.pull(data_types, start_indices, end_indices))

        elif command.command_type == "machine_model_update":
            #setattr(self.machine,command.data) = command.data
            pass

        elif command.command_type == 'log':
            self.logging_server.log_queue.put((command.time, command.data))

        elif command.command_type == 'update':
            self.updateMachineState()

        elif command.command_type == 'flush_to_websocket':
            self.writeDatabaseToWebsocket()

    def updateMachineState(self):
        #state_updates = self.database_puller.pull(self.synchronizer.state_streams, -1, None)
        #for state_update in state_updates:

        for k in range(0, len(self.machine.motion_states)-1):
            state_update = self.database_puller.pull(self.machine.state_streams[k], -1, None)
            if state_update[0]:
                setattr(self.machine, self.machine.motion_states[k], state_update[1][0][0])
                getattr(self.synchronizer, self.machine.state_initialization_events[k]).set()

    def archiveRecords(self):
        pass

    def writeDatabaseToWebsocket(self):
        self.synchronizer.q_trajectory_planner_data_return_queue.put({'stepgen': self.data_store.STEPGEN_FEEDBACK_POSITIONS, 'commanded': self.data_store.COMMANDED_SERVO_POSITIONS})

    def writeDatabaseToFile(self):
        np.save(self.machine.database_output_directory + 'stepgen_feedback', self.data_store.STEPGEN_FEEDBACK_POSITIONS)
        np.save(self.machine.database_output_directory + 'stepgen_time', self.data_store.RTAPI_CLOCK_TIMES)
        #fh = open(self.machine.database_output_directory + self.machine.database_file_name, 'wb')
        #fh.write(pickle.dumps(self.data_store))
        #fh.close()

class DataStore():
    def __init__(self, machine_statics):
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
        self.machine_time_delta = np.empty((0,1), float)
        self.machine_clock_times = np.empty((0,1), float)
        self.MACHINE_TIMES_INTERPOLATED = np.empty((0, 1), float)

        #Received time vectors on PC end, would be interesting to correlate with machine time. Do we need tx number here?
        self.LOWFREQ_ETHERNET_RECEIVED_TIMES = np.empty((0, 1), float)
        self.RTAPI_CLOCK_TIMES = np.empty((0, 1), float)
        self.HIGHFREQ_ETHERNET_RECEIVED_TIMES = np.empty((0, 1), float)
        self.RSH_CLOCK_TIMES = np.empty((0, 1), float)
        self.SERIAL_RECEIVED_TIMES = np.empty((0, 1), float)

        #Buffer fill level
        self.machine_tc_queue_length = np.empty((0, 1), float)
        self.HIGHRES_TC_QUEUE_LENGTH = np.empty((0, 1), float)

        #Positions from stepgen and encoders
        self.COMMANDED_SERVO_POLYLINE_OBJECTS = []
        #self.sent_servo_commands = []

        self.RTAPI_FEEDBACK_INDICES = np.empty((0, 1), float)
        self.STEPGEN_FEEDBACK_POSITIONS = np.empty((0, machine_statics.number_of_joints), float)
        self.ENCODER_FEEDBACK_POSITIONS = np.empty((0, machine_statics.number_of_joints), float)
        self.COMMANDED_SERVO_POSITIONS = np.empty((0, machine_statics.number_of_joints), float)

        #Thread counter
        #self.rt_thread_num_executions_delta = np.zeros(1,dtype=int)
        self.rt_thread_num_executions_delta = np.empty((0,1), float)
        
        #Imported command points
        self.imported_axes_points = []

        #Successfully executed moves
        self.NETWORK_PID_DELAYS = np.empty((0,1), float)
        self.POLYLINE_TRANSMISSION_TIMES = np.empty((0, 1), float)
        self.EXECUTED_MOVES = []
        #self.PROCESSED_MOVES = []

        self.DATA_ARCHIVE = {}

        #FIXME not needed
        self.data_descriptors = ['RTAPI_FEEDBACK_INDICES', 'COMMANDED_JOINT_POSITIONS', 'STEPGEN_FEEDBACK_POSITIONS',
                                 'ENCODER_FEEDBACK_POSITIONS', 'HIGHRES_TC_QUEUE_LENGTH', 'RTAPI_CLOCK_TIMES',
                                 'LOWFREQ_ETHERNET_RECEIVED_TIMES', 'HIGHFREQ_ETHERNET_RECEIVED_TIMES', 'RSH_CLOCK_TIMES',
                                 'SERIAL_RECEIVED_TIMES', 'ENCODER_FEEDBACK_POSITIONS', 'COMMANDED_SERVO_POLYLINE_OBJECTS',
                                 'NETWORK_PID_DELAYS', 'POLYLINE_TRANSMISSION_TIMES', 'COMMANDED_POSITIONS']

    def lookupDataType(self, data_type):
        data_type = data_type.upper()
        # if data_type in self.data_descriptors:
        #     return getattr(self,data_type)
        try:
            return getattr(self, data_type)
        except AttributeError:
            print('DATA STORE: Data type %s does not exist' % data_type)
            return None

class DatabaseServerProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')

class DatabaseOutputProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')

class SynchronizersProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')

