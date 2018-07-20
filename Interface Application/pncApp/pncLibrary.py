import pncTrajectoryGeneration as TP, pncSculptPrintIntegration as SP
import sys, pickle, time, select, socket, wpipe
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
printout_subthread_launch_string = "SUBTHREAD LAUNCHED: {} thread started {}"
printout_thread_terminate_string = "THREAD STOPPED: {} process stopped {}"
printout_subthread_terminate_string = "SUBTHREAD STOPPED: {} thread stopped {}"
printout_device_boot_string = "DEVICE BOOTSTRAPPED: {} on {}"
printout_device_comm_initialization_string = "DEVICE COMMUNICATION INITIALIZED: {} comm running on {} process"
printout_connection_open_string = "CONNECTED to {} on {}"
printout_connection_close_string = "DISCONNECTED from {} on {}"
printout_connection_failed_string = "CONNECTION FAILED on {}, error: "
printout_sculptprint_interface_initialization_string = "{} {} PID: {} starting pncApp"
printout_trajectory_planner_connection_string = "WEBSOCKET CONNECTED: {} connected to {}"
printout_websocket_connection_failure_string = "WEBSOCKET CONNECTION FAILED: {} had error {}"
printout_trajectory_planner_connection_failure_string = "REMOTE TP CONNECTION FAILED: {} did not get response from {}"
printout_waiting_for_first_move_string = "MACHINE CONTROLLER: Waiting for first trajectory from {}..."
printout_trajectory_planner_motion_queues_linked_string = "MOTION QUEUE FEEDER: Primary move queue linked to {} move queue"
printout_subsequence_enqueued_string = "TRAJECTORY PLANNER: {} received and enqueued sequence ID {}"
printout_ssh_connection_success_string = "SSH CONNECTION SUCCESS: User {} connected on address {}"
printout_ssh_connection_failure_string = "SSH CONNECTION FAILURE: User {} on {} had error: {}"
printout_ssh_connection_close_string = "DISCONNECTED: {} ended SSH connection for user {}"
printout_ssh_command_execution_string = "MACHINE OS CONTROLLER: Executing {}"
printout_interface_socket_connection_string = "PNC INTERFACE SOCKET LAUNCHER: Connected to client {} on address {}"
printout_database_field_creation_string = "DATABASE CONTROLLER: Creating record type {} with size {}"
printout_database_object_list_creation_string = "DATABASE CONTROLLER: Creating object list type {} with size {}"
printout_database_flush_to_websocket_string = "DATABASE CONTROLLER: Writing {} to websocket flush queue"

#Queue operations
queue_wait_timeout = 1
move_queue_wait_timeout = 1
queue_database_command_queue_wait_timeout = 0.05

#SculptPrint Formatting
SP_axis_sensor_IDs = [0, 1]
SP_pncApp_time = ['T']
SP_pncApp_machine_axes = ['X','Y','Z','A','B']
SP_CAM_machine_axes = [['X','Z','S','Y','A','B','V','W'], ['X','Z','S','Y','A','B','V','W']]
SP_pncApp_data_auxes = [['BL', 'BPID'], ['']]
SP_data_formats = [['T','X','Z','S','Y','A','B','V','W','BL','BPID'], ['T','X','Z','S','Y','A','B','V','W']]

#SP_clock_data_names = [['RTAPI_CLOCK_TIMES', 'RSH_CLOCK_TIMES'], ['ENCODER_RECEIVED_TIMES']]
#SP_main_data_streams = [[('RTAPI_CLOCK_TIMES', 'STEPGEN_FEEDBACK_POSITIONS', (1, 5))], [('SERIAL_RECEIVED_TIMES', 'ENCODER_FEEDBACK_POSITIONS', (1, 5))]]
SP_main_data_streams = [[('RTAPI_CLOCK_TIMES', 'STEPGEN_FEEDBACK_POSITIONS', (1, 5))], [('INTERPOLATED_POLYLINE_TRANSMISSION_TIMES', 'COMMANDED_SERVO_POSITIONS', (1, 5))]]
SP_auxiliary_data_streams = [[('RSH_CLOCK_TIMES', 'HIGHRES_TC_QUEUE_LENGTH', (1, 1)), ('POLYLINE_TRANSMISSION_TIMES', 'NETWORK_PID_DELAYS', (1, 1))], [('', '', (0, 0))]]
SP_auxiliary_data_labels = [[r'Buffer Level', r'Buffer Control PID Delays'], ['']]

#Machine kinematics
machine_number_of_joints = 5
machine_servo_dt = 0.001
#machine_limits = [[-1.75, 2.55], [-2.05, 2.95], [-3.45, 0.1], [-5, 95], [-99999, 99999]]
machine_table_center_axis_travel_limits = [[-1.75, -2.05, -3.45, -5, -99999], [2.55, 2.95, 0.1, 95, 99999]]
machine_max_joint_velocity = [0.6666, 0.6666, 0.6666, 20, 20]
machine_max_joint_acceleration = [30, 30, 30, 1500, 1500]
machine_max_joint_jerk = [100, 100, 100, 100, 100]
machine_fk = []
machine_ik = []

#Machine Comm Parameters
machine_ping_delay_time = 0.05

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
        #self.LF_start_time_index = 0
        #self.HF_start_time_index = 0
        #self.serial_start_time_index = 0

        #self.last_time_reading = 0
        #FIXME set this up parametrically
        #self.last_position_reading = [0]*5
        #self.last_buffer_level_reading = 0

        #self.last_values_read = [[0]*len(SP_data_formats[k]) for k in range(0,len(SP_data_formats))]
        #self.feedback_indices = [[0]*(len(SP_pncApp_machine_axes)+len(SP_pncApp_data_auxes[k])) for k in range(0,len(SP_data_formats))]
        self.CAM_clock_offsets = dict()
        self.last_values_read = dict()
        self.feedback_indices = dict()
        self.clock_offsets = dict()

        #self.machine_feedback_record_id = 0
        #self.encoder_feedback_record_id = 0

    def buildFeedbackIndexDictionary(self):
        pass

class CloudTrajectoryPlannerState():
    def __init__(self):
        self.websocket_connected_event = Event()
        self.tp_connected_event = Event()
        self.send_next_block_event = Event()
        self.point_id_ack_event = Event()
        self.matching_sequence_received_event = Event()

        self.point_ack_id = 0
        self.enqueued_sequence_id = 0
        self.current_requested_sequence_id = 0
        self.current_received_sequence_id = 0

class PNCAppConnection():
    def __init__(self, connection_type, command_format, feedback_format):
        self.connection = None
        self.connection_type = connection_type
        self.command_format = command_format
        self.feedback_format = feedback_format

        self.app_connection_event = Event()
        self.app_feedback_synchronization_event = Event()

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
        self.t_run_motion_queue_feeder_event = manager.Event()
        self.t_run_feedback_processor_event = manager.Event()
        self.t_run_logging_server_event = manager.Event()
        self.t_run_database_puller_event = manager.Event()
        self.t_run_database_pusher_event = manager.Event()
        self.t_run_machine_state_manipulator_event = manager.Event()
        self.t_run_print_server_event = manager.Event()
        self.t_run_cloud_trajectory_planner_event = manager.Event()
        self.t_run_cloud_trajectory_planner_receiver_event = manager.Event()
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

        self.mc_initial_stepgen_position_set_event = manager.Event()
        self.mc_initial_buffer_level_set_event = manager.Event()
        self.mc_restore_mode_event = manager.Event()
        self.mc_clock_sync_event = manager.Event()
        self.mc_xenomai_clock_sync_event = manager.Event()
        self.mc_run_motion_event = manager.Event()
        self.mc_motion_complete_event = manager.Event()
        self.mc_rsh_error_event = manager.Event()
        self.mc_socket_connected_event = manager.Event()
        self.ei_encoder_comm_init_event = manager.Event()
        self.ei_encoder_init_event = manager.Event()
        self.ei_initial_encoder_position_set_event = manager.Event()
        self.tp_plan_motion_event = manager.Event()

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
        self.q_trajectory_planner_data_return_queue = manager.Queue()
        self.q_trajectory_planner_planned_move_queue = manager.Queue()

        # Locks
        self.db_data_store_lock = manager.Lock()
        self.db_machine_state_lock = manager.Lock()
        self.db_pull_lock = manager.Lock()
        self.mc_socket_lock = manager.Lock()

        # Process Clock Sync
        self.process_start_signal = manager.Event()

        # Position state machine initialization events
        #self.state_streams = ['STEPGEN_FEEDBACK_POSITIONS', 'HIGHRES_TC_QUEUE_LENGTH', 'ENCODER_FEEDBACK_POSITIONS']
        #self.machine_state_events = ['mc_initial_position_set_event', 'ei_initial_encoder_position_set_event', 'mc_initial_buffer_level_set_event']
        #self.machine_state_events = [self.mc_initial_position_set_event, self.ei_initial_encoder_position_set_event, self.mc_initial_buffer_level_set_event]

        self.profile_event = manager.Event()

######################## Commands ########################
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
    def __init__(self, command, command_data, connection, connection_type, connection_format):
        self.command = command
        self.command_data = command_data
        self.connection = connection
        self.connection_type = connection_type
        self.connection_format = connection_format

class IPCDataWrapper():
    def __init__(self, data):
        self.data = data

class RSHError(Exception):
    def __init__(self, message, errors):
        super().__init__(message)
        self.errors = errors

class WebsocketError(Exception):
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
def initializeInterfaceIPC(app_connector):
    if app_connector.connection_type == 'pipe':
        try:
            print("SCULPTPRINT INTERFACE: Acquiring pipe %s..." % socket_sculptprint_ipc_pipe_name)
            sculptprint_pipe = wpipe.Client(socket_sculptprint_ipc_pipe_name, wpipe.Mode.Master, maxmessagesz=4096)
            app_connector.app_connection_event.set()
            print('SCULPTPRINT INTERFACE: Pipe %s opened successfully' % socket_sculptprint_ipc_pipe_name)
            return sculptprint_pipe
        except:
            raise ConnectionRefusedError
    elif app_connector.connection_type == 'socket':
        try:
            sculptprint_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sculptprint_socket.connect(('localhost', socket_interface_socket_port))
            app_connector.app_connection_event.set()
            print('SCULPTPRINT INTERFACE: Socket connected successfully to %s' % 'localhost')
            return sculptprint_socket
        except:
            raise ConnectionRefusedError

def sendIPCData(connection_type, data_type, connection, message):
    if connection_type == 'pipe':
        if data_type == 'binary':
            connection.write(pickle.dumps(IPCDataWrapper(message)))
        elif data_type == 'text':
            connection.write((message).encode())
        elif data_type == 'internal':
            pass
    elif connection_type == 'socket':
        if data_type == 'binary':
            start_time = time.clock()
            connection.send(pickle.dumps(IPCDataWrapper(message)))
            #print('send time is ' + str(time.clock()-start_time))
        elif data_type == 'text':
            connection.send((str(message)+'\n').encode())
        elif data_type == 'internal':
            pass

def sendIPCAck(connection_type, connection_format, connection, message):
    if connection_format != 'internal':
        sendIPCData(connection_type, connection_format, connection, message + '_ACK')

def receiveIPCData(connection_type, connection_format, connection, timeout = socket_wait_timeout, interval = socket_wait_interval):
    received_data = waitForIPCDataTransfer(connection_type, connection, timeout, interval)
    if received_data is None:
        raise ConnectionAbortedError
    else:
        print('pickled size is: ' + str(len(received_data)))
        if str(len(received_data)) == 55:
            print('break')
        if connection_format == 'binary':
            try:
                return pickle.loads(received_data)
            except Exception as error:
                print('break in receive, error: ' + str(error))
        elif connection_format == 'text':
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
            start_time = time.clock()
            socket_data = bytearray()
            while select.select([connection], [], [], 0.001)[0]:
                socket_data += connection.recv(65536)
            #print('receive time is ' + str(time.clock()-start_time))
            return socket_data
        else:
            raise TimeoutError

def MVCHandshake(app_connector, message):
    sendIPCData(app_connector.connection_type, 'text', app_connector.connection, message)
    received_message = receiveIPCData(app_connector.connection_type, 'text', app_connector.connection).strip()
    if not received_message == message + '_ACK':
        print('handshake doesnt match, received: ' + received_message)
    return received_message == message + '_ACK'
    # if connection_format == 'binary':
    #     if not received_message.data == message + '_ACK':
    #         print('handshake doesnt match, received: ' + received_message.data)
    #     return received_message.data == message + '_ACK'
    # elif connection_format == 'text':

def safelyHandleSocketData(app_connector, message, expected_data_type, fallback_data):
    if not app_connector.app_connection_event.is_set():
        print('pncApp not connected')
        return fallback_data

    try:
        sendIPCData(app_connector.connection_type, app_connector.command_format, app_connector.connection, message)
        timeout = socket_wait_timeout_initial

        received_packet = receiveIPCData(app_connector.connection_type, app_connector.feedback_format, app_connector.connection, timeout)
        app_connector.app_feedback_synchronization_event.set()
        if type(received_packet.data) is not expected_data_type:
            raise TypeError

        # if received_packet.data == True:
        #     print('break')
        return received_packet.data
    except ConnectionResetError:
        print('pncApp closed connection during ' + str(message))
        app_connector.app_connection_event.clear()
        return fallback_data
    except TimeoutError:
        print('timed out on call to: ' + str(message))
        return fallback_data
    except TypeError:
        print('SculptPrint socket out of sync on call to: ' + str(message))
        return fallback_data
    except EOFError:
        print('socket read error on call to: ' + str(message))
        return fallback_data

def closeMVCConnection(pncApp_connector):
    if pncApp_connector.connection_type == 'pipe':
        pncApp_connector.connection.close()
    elif pncApp_connector.connection_type == 'socket':
        #pncApp_connector.connection.send('CLOSESOCKET'.encode())
        if 'CLOSESOCKET' in safelyHandleSocketData(pncApp_connector, 'CLOSESOCKET', str, ''):
            pncApp_connector.connection.shutdown(socket.SHUT_RDWR)
            pncApp_connector.connection.close()
        else:
            print('socket close error')

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

        DB_query_data = synchronousPull(synchronizer, complete_stream_names, getFeedbackIndices(feedback_state.feedback_indices, complete_stream_names), len(complete_stream_names) * [None])
        time_slice, data_slice = DB_query_data[1][:len(clock_stream_names)], DB_query_data[1][-len(data_stream_names):]

        clock_offsets = getInterfaceClockOffsets(feedback_state.clock_offsets, clock_stream_names)
        #print('interface clock offsets are: ' + str(clock_offsets))
        updateFeedbackIndices(feedback_state.feedback_indices, complete_stream_names, time_slice + data_slice)
        updateFallbackDataPoints(feedback_state.last_values_read, complete_stream_names, complete_stream_sizes, time_slice + data_slice)

        time_slices.append([time_slice[k] - clock_offsets[k] for k in range(0,len(clock_stream_names))])
        data_slices.append(data_slice)

    if update_mode == 'pull':
        return time_slices, data_slices, data_stream_names, data_stream_sizes
    elif update_mode == 'touch':
        return

def getFeedbackIndices(feedback_indices, data_stream_names):
    return list(map(lambda stream: feedback_indices[stream] if stream in feedback_indices else 0, data_stream_names))

def updateFeedbackIndices(feedback_indices, stream_names, streams):
    for stream_name, stream in map(lambda sn, s: (sn, s), stream_names, streams):
        if stream_name in feedback_indices:
            feedback_indices[stream_name] += len(stream)
        else:
            feedback_indices[stream_name] = len(stream)
    #for key, value in feedback_indices.items():

    #return dict(map(lambda stream_name, stream: (stream_name, feedback_indices[stream_name] + len(stream) if stream_name in feedback_indices else len(stream)), stream_names, streams))

def getFallbackDataPoints(fallback_values, stream_names, stream_sizes):
    return list(map(lambda stream_name, stream_size: fallback_values[stream_name] if stream_name in fallback_values else np.zeros(stream_size), stream_names, stream_sizes))

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

def getInterfaceClockOffsets(clock_offsets, data_stream_names):
    return list(map(lambda stream: clock_offsets[stream] if stream in clock_offsets else 0, data_stream_names))

def updateMotionControllerClockOffset(machine, clock_offset, app_clock_offset):
    #clock = time.time()-app_clock_offset
    setattr(machine, clock_offset, time.time()-app_clock_offset)

# def updateInterfaceClockOffsets(fallback_values, clock_offsets):
#     for key, value in fallback_values.items():
#         if 'TIME' in key:
#             clock_offsets[key] = value

def updateInterfaceClockOffsets(machine, clock_offsets):
    clock_offsets['SERIAL_RECEIVED_TIMES'] = estimateMachineClock(machine)#time.time() - machine.pncApp_clock_offset
    #clock_offsets['RTAPI_CLOCK_TIMES'] = time.time() - machine.xenomai_clock_offset
    clock_offsets['RTAPI_CLOCK_TIMES'] = estimateMachineClock(machine)
    clock_offsets['RSH_CLOCK_TIMES'] = time.time() - machine.pncApp_clock_offset
    clock_offsets['POLYLINE_TRANSMISSION_TIMES'] = time.time() - machine.pncApp_clock_offset
    clock_offsets['INTERPOLATED_POLYLINE_TRANSMISSION_TIMES'] = time.time() - machine.pncApp_clock_offset

######################## Database Interaction ########################
def synchronousPull(synchronizer, data_types, start_indices, end_indices):
    with synchronizer.db_pull_lock:
        synchronizer.q_database_command_queue_proxy.put(DatabaseCommand('pull', data_types, (start_indices, end_indices)))
        return synchronizer.q_database_output_queue_proxy.get()

def asynchronousPush(synchronizer, records):
    if type(records) is not list:
        records = [records]
    synchronizer.q_database_command_queue_proxy.put(DatabaseCommand('push', records))

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

    #return (time_to_estimate - machine.pncApp_clock_offset)
    return (time_to_estimate - machine.RT_clock_sync_pncApp_time)
    #return machine.RT_clock_offset + (time_to_estimate - machine.current_current_estimated_network_latency) * machine.clock_resolution

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