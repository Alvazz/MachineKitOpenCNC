import pncTrajectoryGeneration as TP, pncSculptPrintIntegration as SP
import sys, pickle, time, select
from threading import Thread
from multiprocessing import Event, current_process

######################## Statics ########################
socket_localhost = '127.0.0.1'
socket_interface_socket_port = 666
socket_sculptprint_ipc_pipe_name = 'pncApp_ipc_pipe'

#IPC Parameters
socket_wait_timeout_initial = 100
socket_wait_timeout = 2
socket_wait_interval = 0.01

#SSH Parameters
ssh_wait_timeout = 1
ssh_port = 22
ssh_credentials = ('pocketnc', 'pocketnc')
ssh_opts = '-X'
ssh_hosts_path = 'E:\\SculptPrint\\PocketNC\\OpenCNC\\Interface Application\\pncApp\\Support Files\\known_hosts'

#Terminal printouts
printout_connection_string = "CONNECTED to {} remote shell at {}:{}"
printout_failed_connection_string = "CONNECTION FAILED: Could not connect to {0} remote shell at {1}:{2}. Is {0} ON?"
printout_manager_launch_string = "PROCESS LAUNCHED: {} PID: {}"
printout_process_launch_string = "PROCESS LAUNCHED: {} PID: {} from master {} PID: {}"
printout_process_terminate_string = "PROCESS STOPPED: {} PID: {}"
printout_process_self_terminate_string = "PROCESS STOPPED: {} PID: {} shut itself down"
printout_process_force_terminate_string = "PROCESS FORCE STOPPED: {} PID: {}"
printout_process_error_string = "PROCESS ERROR: {} encountered error: {}"
printout_pncApp_launch_string = "SUCCESS: pncApp launched on {} CPUs..."
printout_pncApp_launch_failure_string = "FAILURE: pncApp cannot launch, cleaning up..."
printout_pncApp_terminate_string = "SUCCESS: pncApp terminated without mess"
printout_thread_launch_string = "THREAD LAUNCHED: {} process started {}"
printout_thread_terminate_string = "THREAD STOPPED: {} process stopped {}"
printout_device_boot_string = "DEVICE BOOTSTRAPPED: {} on {}"
printout_device_comm_initialization_string = "DEVICE COMMUNICATION INITIALIZED: {} comm running on {} process"
printout_connection_open_string = "CONNECTED to {} on {}"
printout_connection_close_string = "DISCONNECTED from {} on {}"
printout_connection_failed_string = "CONNECTION FAILED on {}, error: "
printout_sculptprint_interface_initialization_string = "SculptPrint embedded python process {} PID: {} starting pncApp"
printout_trajectory_planner_connection_string = "WEBSOCKET CONNECTED: {} connected to {}"
printout_trajectory_planner_connection_failure_string = "WEBSOCKET CONNECTION FAILED: {} had error {}"
printout_ssh_connection_success_string = "SSH CONNECTION SUCCESS: User {} on {}"
printout_ssh_connection_failure_string = "SSH CONNECTION FAILURE: User {} on {} had error: {}"
printout_ssh_connection_close_string = "DISCONNECTED: {} ended SSH connection for user {}"
printout_interface_socket_connection_string = "PNC INTERFACE SOCKET LAUNCHER: Connected to client {} on address {}"

#Queue operations
queue_wait_timeout = 1

#SculptPrint Formatting
SP_axis_sensor_IDs = [0, 1]
SP_pncApp_time = ['T']
SP_pncApp_machine_axes = ['X','Y','Z','A','B']
SP_CAM_machine_axes = [['X','Z','S','Y','A','B','V','W'], ['X','Z','S','Y','A','B','V','W']]
SP_pncApp_data_auxes = [['BL', 'BPID'], ['']]
SP_data_formats = [['T','X','Z','S','Y','A','B','V','W','BL','BPID'], ['T','X','Z','S','Y','A','B','V','W']]

#SP_clock_data_names = [['RTAPI_CLOCK_TIMES', 'RSH_CLOCK_TIMES'], ['ENCODER_RECEIVED_TIMES']]
SP_main_data_streams = [[('RTAPI_CLOCK_TIMES', 'STEPGEN_FEEDBACK_POSITIONS', (1, 5))], [('SERIAL_RECEIVED_TIMES', 'ENCODER_FEEDBACK_POSITIONS', (1, 5))]]
SP_auxiliary_data_streams = [[('RSH_CLOCK_TIMES', 'HIGHRES_TC_QUEUE_LENGTH', (1, 1)), ('POLYLINE_TRANSMISSION_TIMES', 'NETWORK_PID_DELAYS', (1, 1))], [('', '', (0, 0))]]
SP_auxiliary_data_labels = [[r'Buffer Level', r'Buffer Control PID Delays'], ['']]

#Machine kinematics
machine_number_of_joints = 5
machine_servo_dt = 0.001
machine_limits = [[-1.75, 2.55], [-2.05, 2.95], [-3.45, 0.1], [-5, 95], [-99999, 99999]]
machine_max_joint_velocity = [0.6666, 0.6666, 0.6666, 20, 20]
machine_max_joint_acceleration = [30, 30, 30, 1500, 1500]
machine_max_joint_jerk = [100, 100, 100, 100, 100]
machine_fk = []
machine_ik = []

######################## Directories ########################
dir_parent_folder = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC'
dir_point_sample_folder = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Position Samples'
dir_pncApp_project_path = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\MachineKitOpenCNC\\Interface Application\\pncApp\\'

def updatePath():
    if dir_pncApp_project_path not in sys.path:
        sys.path.append(dir_pncApp_project_path)

######################## Useful Classes ########################
class SculptPrintInterface():
    #MVC for SculptPrint UI
    def __init__(self):
        #super(SculptPrintInterface, self).__init__()
        #self.machine = machine
        self.connected = False

        #Flags for user events
        self.enqueue_moves = False

        #UI Data Fields
        self.start_file = 0
        self.end_file = 0

        #Events
        # self.connect_event = threading.Event()
        # self.enqueue_moves_event = threading.Event()
        # self.moves_queued_event = threading.Event()
        # self.run_motion_event = threading.Event()

class SculptPrintFeedbackState():
    def __init__(self):
        #Data Handling
        self.LF_start_time_index = 0
        self.HF_start_time_index = 0
        self.serial_start_time_index = 0

        #self.last_time_reading = 0
        #FIXME set this up parametrically
        #self.last_position_reading = [0]*5
        #self.last_buffer_level_reading = 0

        #self.last_values_read = [[0]*len(SP_data_formats[k]) for k in range(0,len(SP_data_formats))]
        #self.feedback_indices = [[0]*(len(SP_pncApp_machine_axes)+len(SP_pncApp_data_auxes[k])) for k in range(0,len(SP_data_formats))]
        self.CAM_clock_offsets = dict()
        self.last_values_read = dict()
        self.feedback_indices = dict()

        self.machine_feedback_record_id = 0
        self.encoder_feedback_record_id = 0

    def buildFeedbackIndexDictionary(self):
        pass

class InternalFeedbackState():
    def __init__(self, machine):
        # Feedback State
        self.byte_string = bytearray()
        self.binary_transmission_length = calculateBinaryTransmissionLength(machine)
        self.incoming_transmission_length = None
        self.feedback_encoding = None
        self.feedback_type = None
        self.header_delimiter_index = None
        self.socket_passes = 0

        # Flags
        self.transmission_received = False
        self.terminator_received = False
        self.header_processed = False
        self.header_processing_error = False
        self.feedback_data_processed = False
        self.feedback_data_processing_error = False
        self.rsh_error_check = False
        self.multiple_socket_passes_required = False


class PrintServer(Thread):
    def __init__(self, machine, synchronizer):
        super(PrintServer, self).__init__()
        self.name = "print_server"
        self.machine = machine
        self.synchronizer = synchronizer
        self.startup_event = Event()

    def run(self):
        #printTerminalString(self.machine.thread_launch_string, current_process().name, self.name)
        self.startup_event.set()
        while self.synchronizer.t_run_print_server_event.is_set():
            print(self.synchronizer.q_print_server_message_queue.get(0.5))
            #sys.stdout.flush()

class Synchronizer():
    def __init__(self, manager):
        #Process run events
        self.p_enable_database_event = manager.Event()
        self.p_enable_encoder_event = manager.Event()
        self.p_enable_feedback_handler_event = manager.Event()
        self.p_enable_machine_controller_event = manager.Event()
        self.p_run_database_event = manager.Event()
        self.p_run_encoder_interface_event = manager.Event()
        self.p_run_machine_controller_event = manager.Event()
        self.p_run_feedback_handler_event = manager.Event()
        self.t_run_motion_controller_event = manager.Event()
        self.t_run_feedback_processor_event = manager.Event()
        self.t_run_logging_server_event = manager.Event()
        self.t_run_database_puller_event = manager.Event()
        self.t_run_database_pusher_event = manager.Event()
        self.t_run_machine_state_manipulator_event = manager.Event()
        self.t_run_print_server_event = manager.Event()
        self.t_run_cloud_trajectory_planner_event = manager.Event()
        self.t_run_operating_system_controller = manager.Event()

        #State Switch Events: fb_ for feedback, mc_ for machine_controller, mvc_ for UI, ei_ for encoder_interface
        self.fb_connection_change_event = manager.Event()
        self.fb_link_change_event = manager.Event()
        self.fb_echo_change_event = manager.Event()
        self.fb_estop_change_event = manager.Event()
        self.fb_drive_power_change_event = manager.Event()
        self.fb_status_change_event = manager.Event()
        self.fb_status_change_event.clear()
        self.fb_mode_change_event = manager.Event()
        self.fb_servo_logging_mode_change_event = manager.Event()
        self.fb_buffer_level_feedback_mode_change_event = manager.Event()
        self.fb_comm_mode_change_event = manager.Event()
        self.fb_home_change_event = manager.Event()
        self.fb_all_homed_event = manager.Event()
        self.fb_ping_event = manager.Event()
        self.fb_clock_event = manager.Event()
        self.fb_buffer_level_reception_event = manager.Event()
        self.fb_servo_feedback_reception_event = manager.Event()
        self.fb_feedback_data_initialized_event = manager.Event()

        # #FIXME implement this
        self.fb_position_change_event = manager.Event()

        self.mc_initial_position_set_event = manager.Event()
        self.mc_restore_mode_event = manager.Event()
        self.mc_clock_sync_event = manager.Event()
        self.mc_xenomai_clock_sync_event = manager.Event()
        self.mc_run_motion_event = manager.Event()
        self.mc_motion_complete_event = manager.Event()
        self.mc_rsh_error_event = manager.Event()
        self.mc_socket_connected_event = manager.Event()
        self.ei_encoder_init_event = manager.Event()

        self.mc_startup_event = manager.Event()
        self.fb_startup_event = manager.Event()
        self.ei_startup_event = manager.Event()
        self.db_startup_event = manager.Event()
        self.mc_successful_start_event = manager.Event()
        self.fb_successful_start_event = manager.Event()
        self.ei_successful_start_event = manager.Event()
        self.db_successful_start_event = manager.Event()
        #self.pnc_app_initialized_event = manager.Event()
        self.mc_running_event = manager.Event()
        self.fb_running_event = manager.Event()
        self.ei_running_event = manager.Event()
        self.db_running_event = manager.Event()

        self.os_linuxcnc_running_event = manager.Event()
        self.os_linuxcncrsh_running_event = manager.Event()
        self.os_ssh_connected_event = manager.Event()

        #self.testevent = Event()

        #SculptPrint MVC Events
        self.mvc_pncApp_initialized_event = manager.Event()
        self.mvc_pncApp_started_event = manager.Event()
        self.mvc_connect_event = manager.Event()
        self.mvc_connected_event = manager.Event()
        self.mvc_enqueue_moves_event = manager.Event()
        self.mvc_moves_queued_event = manager.Event()
        self.mvc_execute_motion_event = manager.Event()
        self.mvc_run_feedback_event = manager.Event()
        self.mvc_app_shutdown_event = manager.Event()

        # Queues
        self.q_database_command_queue_proxy = manager.Queue()
        self.q_database_output_queue_proxy = manager.Queue()
        self.q_print_server_message_queue = manager.Queue()
        self.q_machine_controller_command_queue = manager.Queue()

        # Locks
        self.db_data_store_lock = manager.Lock()
        self.db_machine_state_lock = manager.Lock()
        self.db_pull_lock = manager.Lock()
        self.mc_socket_lock = manager.Lock()

        # Process Clock Sync
        self.process_start_signal = manager.Event()

class Move():
    def __init__(self,point_samples,move_type = None, sequence_id = None):
        super(Move, self).__init__()
        self.serial_number = -1
        self.point_samples = point_samples
        self.move_type = move_type
        self.sequence_id = sequence_id

        ## To be populated when move is inserted into MotionController queue
        self.servo_tx_array = -1
        self.polylines = -1
        self.blocklength = -1

        #self.start_points = np.array([],dtype=float)
        self.start_points = point_samples[0, :]
        #self.end_points = np.array([],dtype=float)
        self.end_points = point_samples[-1, :]

class MachineCommand():
    #Class for RSH commands, mode switches, etc
    def __init__(self, command_type, command_data):
        #super(MachineCommand, self).__init__()
        self.command_type = command_type
        self.command_data = command_data

class DatabaseCommand():
    def __init__(self, command_type, records, parameters = None, time = None):
        self.command_type = command_type
        self.data = records
        self.command_parameters = parameters
        self.time = time

class OSCommand():
    def __init__(self, command_type, command_string = None, time = None):
        self.command_type = command_type
        self.command_string = command_string
        self.time = time

# class MVCPacket():
#     def __init__(self, command, data):
#         self.command = command
#         self.data = data

class PNCAppCommand():
    def __init__(self, command, data, connection, connection_type, connection_format):
        self.command = command
        self.data = data
        self.connection = connection
        self.connection_type = connection_type
        self.connection_format = connection_format

class IPCDataWrapper():
    def __init__(self, data):
        self.data = data

class RSHError(BaseException):
    def __init__(self, message, errors):
        super().__init__(message)
        self.errors = errors

class WebsocketError(BaseException):
    def __init__(self, message):
        self.message = message

class MachineControllerError(BaseException):
    def __init__(self, message):
        self.message = message

######################## Multitask Management ########################
def startPrintServer(machine, synchronizer):
    print_server = PrintServer(machine, synchronizer)
    print_server.start()
    print_server.startup_event.wait()
    printTerminalString(machine.thread_launch_string, current_process().name, print_server.name)
    return print_server

def waitForThreadStart(caller, *args):
    started_threads = []
    for thread_class in args:
        thread = thread_class(caller)
        setattr(caller, getattr(thread, 'name'), thread)
        thread.start()
        started_threads.append(thread)

    for thread in started_threads:
        thread.startup_event.wait()

def waitForThreadStop(caller, *args):
    stopped_threads = []
    for thread in args:
        getattr(caller.synchronizer, 't_run_' + thread.name + '_event').clear()
        stopped_threads.append(thread)

    for thread in stopped_threads:
        thread.join()

######################## IPC ########################
# def getEventStatus(event, database_command_queue_proxy, database_output_queue_proxy):
#     database_command_queue_proxy.put('get_event_status')

def sendIPCData(connection_type, data_type, connection, message):
    if connection_type == 'pipe':
        if data_type == 'binary':
            connection.write(pickle.dumps(IPCDataWrapper(message)))
        elif data_type == 'string':
            connection.write((message).encode())
        elif data_type == 'internal':
            pass
    elif connection_type == 'socket':
        if data_type == 'binary':
            connection.send(pickle.dumps(IPCDataWrapper(message)))
        elif data_type == 'string':
            connection.send((str(message)+'\n').encode())
        elif data_type == 'internal':
            pass

def sendIPCAck(connection_type, connection_format, connection, message):
    if connection_format != 'internal':
        sendIPCData(connection_type, 'string', connection, message + '_ACK')

def receiveIPCData(connection_type, connection_format, connection, timeout = socket_wait_timeout, interval = socket_wait_interval):
    received_data = waitForIPCDataTransfer(connection_type, connection, timeout, interval)
    if received_data is None:
        raise ConnectionAbortedError
    else:
        print('pickled size is: ' + str(len(received_data)))
        if connection_format == 'binary':
            try:
                return pickle.loads(received_data)
            except Exception as error:
                print('break in receive, error: ' + str(error))
        elif connection_format == 'string':
            return received_data.decode()

def waitForIPCDataTransfer(connection_type, connection, timeout = None, wait_interval = None):
    if timeout is None:
        timeout = np.inf

    if connection_type == 'pipe':
        start_time = time.clock()
        while (time.clock() - start_time) < timeout and not connection.canread():
            time.sleep(wait_interval or 0)

        if (time.clock() - start_time) > timeout:
            raise TimeoutError
        else:
            return connection.read()

    elif connection_type == 'socket':
        if select.select([connection], [], [], timeout)[0]:
            socket_data = bytearray()
            while select.select([connection], [], [], 0.1)[0]:
                socket_data += connection.recv(4096)
            return socket_data
        else:
            raise TimeoutError

def MVCHandshake(connection_type, connection, message):
    sendIPCData(connection_type, 'string', connection, message)
    received_message = receiveIPCData(connection_type, 'string', connection).strip()
    if not received_message == message + '_ACK':
        print('handshake doesnt match, received: ' + received_message)
    return received_message == message + '_ACK'
    # if connection_format == 'binary':
    #     if not received_message.data == message + '_ACK':
    #         print('handshake doesnt match, received: ' + received_message.data)
    #     return received_message.data == message + '_ACK'
    # elif connection_format == 'string':

def setSynchronizer(pipe, synchronizer):
    pipe.send(synchronizer)

def getSynchronizer(caller, pipe):
    caller.synchronizer = pipe.recv()

######################## App Interface ########################
def updateInterfaceData(update_mode, synchronizer, feedback_state, main_data_streams, auxiliary_data_streams, axis_sensor_IDs):
    time_slices = []
    data_slices = []
    for ID in axis_sensor_IDs:
        streams = main_data_streams[ID] + auxiliary_data_streams[ID]
        clock_stream_names = [name for name in list(map(lambda stream: stream[0], streams)) if name != '']
        data_stream_names = [name for name in list(map(lambda stream: stream[1], streams)) if name != '']
        clock_stream_sizes = [size for size in list(map(lambda stream: stream[2][0], streams)) if size != 0]
        data_stream_sizes = [size for size in list(map(lambda stream: stream[2][1], streams)) if size != 0]
        complete_stream_names = clock_stream_names + data_stream_names
        complete_stream_sizes = clock_stream_sizes + data_stream_sizes
        #complete_stream_names = [name for name in clock_stream_names + data_stream_names if name != '']
        #complete_stream_names = clock_stream_names + data_stream_names
        #complete_stream_sizes = [size for size in clock_stream_sizes + data_stream_sizes if size != 0]
        #complete_stream_sizes = clock_stream_sizes + data_stream_sizes

        DB_query_data = lockedPull(synchronizer, complete_stream_names, getFeedbackIndices(feedback_state, complete_stream_names), len(complete_stream_names) * [None])

        time_slice, data_slice = DB_query_data[1][:len(clock_stream_names)], DB_query_data[1][-len(data_stream_names):]
        updateFeedbackIndices(feedback_state.feedback_indices, complete_stream_names, time_slice + data_slice)
        updateFallbackDataPoints(feedback_state.last_values_read, complete_stream_names, complete_stream_sizes, time_slice + data_slice)

        time_slices.append(time_slice)
        data_slices.append(data_slice)

    if update_mode == 'pull':
        return time_slice, data_slice, data_stream_names, data_stream_sizes
    elif update_mode == 'touch':
        return

def getFeedbackIndices(feedback_state, data_stream_names):
    return list(map(lambda stream: feedback_state.feedback_indices[stream] if stream in feedback_state.feedback_indices else 0, data_stream_names))

def updateFeedbackIndices(feedback_indices, stream_names, streams):
    for stream_name, stream in map(lambda sn, s: (sn, s), stream_names, streams):
        if stream_name in feedback_indices:
            feedback_indices[stream_name] += len(stream)
        else:
            feedback_indices[stream_name] = len(stream)
    #for key, value in feedback_indices.items():

    #return dict(map(lambda stream_name, stream: (stream_name, feedback_indices[stream_name] + len(stream) if stream_name in feedback_indices else len(stream)), stream_names, streams))

def getFallbackDataPoints(feedback_state, stream_names, stream_sizes):
    return list(map(lambda stream_name, stream_size: feedback_state.last_values_read[stream_name] if stream_name in feedback_state.last_values_read else np.zeros(stream_size), stream_names, stream_sizes))

def updateFallbackDataPoints(fallback_values, stream_names, stream_sizes, streams):
    for stream_name, stream_size, stream in map(lambda sn, ss, s: (sn, ss, s), stream_names, stream_sizes, streams):
        if len(stream) > 0:
            fallback_values[stream_name] = stream[-1]
        else:
            if stream_name not in fallback_values:
                fallback_values[stream_name] = np.zeros(stream_size)
        # if stream_name in feedback_indices:
        #     if len(stream) > 0:
        #         fallback_values[stream_name] = stream[-1]
        #     else:
        #
        #     feedback_indices[stream_name] += len(stream)
        # else:
        #     feedback_indices[stream_name] = len(stream)
    #return dict(map(lambda stream_name, stream_size, stream: tuple((stream_name, stream[-1] if len(stream) > 0 else fallback_values[stream_name] if stream_name in fallback_values else np.zeros(stream_size))), stream_names, stream_sizes, streams))

def lockedPull(synchronizer, data_types, start_indices, end_indices):
    with synchronizer.db_pull_lock:
        synchronizer.q_database_command_queue_proxy.put(DatabaseCommand('pull', data_types, (start_indices, end_indices)))
        return synchronizer.q_database_output_queue_proxy.get()

def waitForErrorReset():
    #FIXME implement a spinlock or wait for RSH error to be handled and reset, then return to normal operation
    pass

######################## State Machine ########################
def setTaskRunFlags(synchronizer, state = True):
    for attribute in dir(synchronizer):
        if attribute.startswith("p_") or attribute.startswith("t_"):
            if state:
                getattr(synchronizer, attribute).set()
            else:
                getattr(synchronizer, attribute).clear()

def pushState(machine):
    machine.mode_stack += machine.mode

def popState(machine):
    return machine.mode_stack.pop()
    # return prev_mode

def restoreState():
    print("IMPLEMENT RESTORE STATE IF NEEDED")

def resetRSHError(machine):
    print('before ' + str(machine.rsh_error))
    # super(MachineModel, self).rsh_error = 0
    machine.rsh_error = 0
    print('after ' + str(machine.rsh_error))
    # self.machine.rsh_error.value = 0

def checkOnOff(machine, flag):
    return machine.rsh_feedback_flags.index(flag.strip().upper()) % 2

def isAutoMode(machine, synchronizer):
    if machine.mode.upper() == 'AUTO' and synchronizer.fb_mode_change_event.is_set():
        return True
    else:
        return False

def isManualMode(machine, synchronizer):
    if machine.mode.upper() == 'MANUAL' and synchronizer.fb_mode_change_event.is_set():
        return True
    else:
        return False

def isHomed(machine, synchronizer):
    return (all(machine.axis_home_state) and synchronizer.fb_home_change_event.is_set())
    # if all(self.machine.axis_home_state) and self.machine.home_change_event.isSet():
    #     return True
    # else:
    #     return False

def isIdle(machine, synchronizer):
    if machine.status.upper() == 'IDLE' and synchronizer.fb_status_change_event.is_set():
        return True
    else:
        return False

######################## Clocking ########################
def estimateMachineClock(machine, time_to_estimate=-1):
    if time_to_estimate == -1:
        time_to_estimate = time.time()

    return (time_to_estimate - machine.pncApp_clock_offset)
    #return machine.RT_clock_offset + (time_to_estimate - machine.estimated_network_latency) * machine.clock_resolution

######################## Comms ########################
def calculateBinaryTransmissionLength(machine):
    machine.binary_transmission_length = 2 + (
                machine.servo_log_num_axes * machine.servo_log_buffer_size + machine.servo_log_buffer_size) * machine.size_of_feedback_double + 1
    return machine.binary_transmission_length

def gobbleTerminators(byte_string, terminator):
    if len(byte_string) >= len(terminator):
        if byte_string[0:len(terminator)] == terminator:
            print('detected useless terminators')
            return gobbleTerminators(byte_string[len(terminator):],terminator) or b''
        else:
            return byte_string
    else:
        return byte_string

def countTerminatorsToGobble(byte_string, terminator):
    terminator_count = 0
    if len(byte_string) >= len(terminator):
        for b in range(0,len(byte_string),len(terminator)):
            if byte_string[b:b+len(terminator)] == terminator:
                terminator_count += 1
            else:
                break
    return terminator_count

def socketLockedWrite(machine, synchronizer, data):
    with synchronizer.mc_socket_lock:
        machine.rsh_socket.send(data)

######################## Data Handling ########################
import struct, numpy as np, csv
def convertFloat2Bin(num):
    return struct.pack('!f', num)

def convertBin2Float(bytes):
    return struct.unpack('!f', bytes)

def convertInt2Bin(num):
    return struct.pack('!i',num)

def formatBinaryLine(axisCoords, polyLines, blockLength, positionInFile):
    return #position in file#

# def padAndFormatAxisPoints(points, polylines, blocklength):
#     pad_points = np.lib.pad(points, ((0, blocklength - (np.size(points, 0) % blocklength)), (0, 0)), 'constant',
#                             constant_values=points[-1])
#     shape_points = pad_points.reshape((-1, blocklength), order='C')
#     return np.pad(shape_points, ((0, polylines - (np.size(shape_points, 0) % polylines)), (0, 0)), 'constant',
#                   constant_values=shape_points[-1, -1])

# def importAxisPoints(file, polylines, blocklength):
#     points = np.array(list(csv.reader(open(file, "rt"), delimiter=","))).astype("float")
#     return padAndFormatAxisPoints(points, polylines, blocklength)
#
# def importAxesPoints(file, machine):
#     ##FIXME check for overtravel
#     points = np.array(list(csv.reader(open(file, "rt"), delimiter=" "))).astype("float")[:,:machine.number_of_joints]
#     return points
#
# def formatPoints(points, polylines, block_length):
#     axis_coords = []
#     #FIXME fix if not divisible by polylines*blocklength
#     for axis in range(points.shape[1]):
#         axis_coords.append(padAndFormatAxisPoints(np.asarray([points[:, axis]]).T, polylines, block_length))
#     return np.asarray(axis_coords).transpose(1, 2, 0)

######################## Visualization ########################
import matplotlib.pyplot as plt
def visualizePoints(move_queue):
    points = np.empty((0,5), float)
    for move in move_queue:
        points = np.vstack((points,move.point_samples))
    plt.plot(points)
    plt.show()

######################## Terminal Interaction ########################
def printTerminalString(string, *args):
    print(string.format(*args))

def printStringToTerminalMessageQueue(queue, string, *args):
    queue.put(string.format(*args))