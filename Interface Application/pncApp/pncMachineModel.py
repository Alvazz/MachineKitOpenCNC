import numpy as np
from multiprocessing.managers import NamespaceProxy

class MachineModelStatics():
    def __init__(self):
        self.name = None#Jokes
        self.machine_name = "PocketVMC"
        self.simulator_name = "the_danke$t"

        #Terminal printouts
        self.connection_string = "CONNECTED to {} remote shell at {}:{}"
        self.failed_connection_string = "Could not connect to {0} remote shell at {1}:{2}. Is {0} ON?"
        self.manager_launch_string = "PROCESS LAUNCHED: {} PID: {}"
        self.process_launch_string = "PROCESS LAUNCHED: {} PID: {} from master {} PID: {}"
        self.process_terminate_string = "PROCESS STOPPED: {} PID: {}"
        self.pncApp_launch_string = "SUCCESS: pncApp launched on {} CPUs..."
        self.pncApp_terminate_string = "SUCCESS: pncApp terminated without mess"
        self.thread_launch_string = "THREAD LAUNCHED: {} process started {}"
        self.thread_terminate_string = "THREAD STOPPED: {} process stopped {}"
        self.device_boot_string = "DEVICE BOOTSTRAPPED: {} on {}"
        self.connection_open_string = "CONNECTED to {} on {}"
        self.connection_close_string = "DISCONNECTED from {} on {}"

        #Machine kinematics
        self.number_of_joints = 5
        self.servo_dt = 0.001
        self.table_center_axis_travel_limits = [[-1.75, 2.55], [-2.05, 2.95], [-3.45, 0.1], [-5, 95], [-99999, 99999]]
        self.max_joint_velocity = [0.6666, 0.6666, 0.6666, 20, 20]
        self.max_joint_acceleration = [30, 30, 30, 1500, 1500]
        self.max_joint_jerk = [100, 100, 100, 100, 100]
        self.fk = []
        self.ik = []


class MachineModelState():
    def __init__(self):
        self.name = None
#         # Init states
#         self.local_epoch = 0
#         self.mode = self.modes[0]
#         self.mode_stack = []
#         self.status = self.statuses[2]
#
#         self.estop = 0
#         self.drive_power = 0
#         self.echo = 1
#         self.comm_mode = 0
#         self.logging_mode = 0
#         self.units = 'inch'
#
#         # self.rsh_error = 0
#         self.rsh_error = 0
#         self.linked = 0
#         self.connected = 0
#         self.axis_home_state = [0] * self.number_of_joints
#         self.current_move_serial_number = -1
#
#         self.ping_tx_time = 0
#         self.ping_rx_time = 0
#         self.current_estimated_network_latency = 0
#         self.OS_clock_offset = 0
#         self.RT_clock_offset = 0
#         self.pncApp_clock_offset = 0
#         self.last_unix_time = 0
#         self.clock_sync_received_time = 0
#
#         # Servo log parameters
#         self.servo_log_num_axes = 5
#         self.servo_log_sub_sample_rate = 10
#         self.servo_log_buffer_size = 50
#         self.buffer_level_feedback_mode = 0
#         self.buffer_level_feedback_period_us = 1e6
#
#         self.polylines_per_tx = 1
#         self.points_per_polyline = 25
#         self.buffer_level_setpoint = 1000

class MachineModel():
    def __init__(self):
        self.name = None

        #Thread/Process Parameters
        self.thread_queue_wait_timeout = 0.1
        self.queue_move_queue_wait_timeout = 1
        self.process_queue_wait_timeout = 0.1
        self.event_wait_timeout = 5
        self.encoder_event_wait_timeout = 30
        self.join_timeout = 2
        self.socket_timeout = 1
        self.max_mvc_startup_wait_time = 10

        #Jokes
        self.machine_name = "PocketVMC"
        self.simulator_name = "the_danke$t"

        #Terminal printouts
        self.connection_string = "CONNECTED to {} remote shell at {}:{}"
        self.failed_connection_string = "CONNECTION FAILED: Could not connect to {0} remote shell at {1}:{2}. Is {0} ON?"
        self.manager_launch_string = "PROCESS LAUNCHED: {} PID: {}"
        self.process_launch_string = "PROCESS LAUNCHED: {} PID: {} from master {} PID: {}"
        self.process_terminate_string = "PROCESS STOPPED: {} PID: {}"
        self.process_self_terminate_string = "PROCESS STOPPED: {} PID: {} shut itself down"
        self.process_force_terminate_string = "PROCESS FORCE STOPPED: {} PID: {}"
        self.process_error_string = "PROCESS ERROR: {} encountered error: {}"
        self.pncApp_launch_string = "SUCCESS: pncApp launched on {} CPUs..."
        self.pncApp_launch_failure_string = "FAILURE: pncApp cannot launch, cleaning up..."
        self.pncApp_terminate_string = "SUCCESS: pncApp terminated without mess"
        self.thread_launch_string = "THREAD LAUNCHED: {} process started {}"
        self.thread_terminate_string = "THREAD STOPPED: {} process stopped {}"
        self.device_boot_string = "DEVICE BOOTSTRAPPED: {} on {}"
        self.device_comm_initialization_string = "DEVICE COMMUNICATION INITIALIZED: {} comm running on {} process"
        self.connection_open_string = "CONNECTED to {} on {}"
        self.connection_close_string = "DISCONNECTED from {} on {}"
        self.connection_failed_string = "CONNECTION FAILED on {}, error: "
        self.sculptprint_interface_initialization_string = "SculptPrint embedded python process {} PID: {} starting pncApp"
        self.trajectory_planner_connection_string = "WEBSOCKET CONNECTED: {} connected to {}"
        self.trajectory_planner_connection_failure_string = "WEBSOCKET CONNECTION FAILED: {} had error {}"
        self.ssh_connection_success_string = "SSH CONNECTION SUCCESS: User {} on {}"
        self.ssh_connection_failure_string = "SSH CONNECTION FAILURE: User {} on {}"
        self.ssh_connection_close_string = "DISCONNECTED: {} ended SSH connection for user {}"

        # self.feedback_listener_thread_handle = None
        # self.machine_controller_thread_handle = None
        # self.encoder_thread_handle = None
        # self.data_store_manager_thread_handle = None
        # self.system_controller_thread_handle = None
        self.sculptprint_interface = None

        #State variables
        self.modes = ['MANUAL', 'MDI', 'AUTO']
        self.statuses = ['IDLE', 'RUNNING', 'PAUSED']

        #RSH Comm
        self.hello_string = 'hello EMC robby 1'
        self.rsh_feedback_strings = ['*', 'bL=', 'bT=', 'PROGRAM_STATUS', 'MODE', 'ON', 'SERVO_LOG_PARAMS', 'MACHINE', 'ECHO', 'HELLO', 'ENABLE', 'ESTOP', 'JOINT_HOMED', 'PING', 'TIME', 'NAK']
        self.rsh_error_string = 'NAK'
        self.rsh_echo_strings = ['SET', 'GET', 'DM']
        self.ascii_rsh_feedback_strings = ['*', 'BTBL', 'PROGRAM_STATUS', 'MODE', 'SERVO_LOG_PARAMS', 'MACHINE', 'ECHO', 'HELLO', 'ENABLE', 'ESTOP', 'JOINT_HOMED', 'PING', 'TIME', 'COMM_MODE', 'BUFFER_LEVEL_FEEDBACK', 'SET_TIMEOUT', 'NAK']
        self.binary_rsh_feedback_strings = ['SF', 'BL']
        self.servo_log_types = ['COMMANDED', 'MEASURED']
        self.minimum_header_length = 3
        self.ascii_header_delimiter = ' '
        self.ascii_line_terminator = '\r\n'
        self.ascii_servo_feedback_terminator = '|'

        # Header is two characters plus a NULL plus a uint32
        #self.binary_header_length = 3 + 4
        self.binary_direct_mode_header = 'DM '
        self.binary_header_delimiter = b' \x00'
        self.binary_line_terminator = b'\x7f'
        self.ascii_header_delimiter_bytes = self.ascii_header_delimiter.encode('utf-8')

        self.rsh_feedback_flags = ['OFF', 'ON', 'NO', 'YES', 'ASCII', 'BINARY', '0', '1']
        self.axes = ['X','Y','Z','A','B']

        #Stepgen calibration
        self.axis_offsets = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.table_zero = [0.0, 0.0, 0.0, 0.0, 0.0]

        #Encoder calibration
        self.number_of_encoders = 6
        #self.machine_table_center_zero = [-1.75, -2.05, 0.1, -5.0, 0.0]
        self.machine_table_center_zero = [-1.75, -2.05, 0.1, 0.0, 0.0]
        self.machine_table_center_zero = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.encoder_read_buffer_size = 10
        self.encoder_init = 1000000
        self.encoder_offset = 5*[1e8]
        #self.encoder_scale = [1/5/8000, 1/5/8000, 1/5/8000, 1/35.5368/8000, 1/35.5555/8000]
        #self.encoder_scale = [.096 / 8000, .096 / 8000, .096 / 8000, 1.0 / 172, -1.0 / 167]
        self.encoder_scale = [.096 / 8000, .096 / 8000, .096 / 8000, 1.0 / (8000 * 2 / 90), -1.0 / (8000 * 2 / 90)]
        self.max_encoder_transmission_length = 1+4*6+1
        self.encoder_command_strings = ['S', 'G', 'R', 'B']
        self.encoder_ack_strings = ['INIT\r\n', 'S&\r\n', 'G&\r\n', 'R&\r\n', 'B&\r\n', '&']
        self.encoder_nak_strings = ['F&\r\n']

        #Spindle parameters
        self.spindle_axis = 'axis1'
        self.spindle_current_feed_forward = 1
        self.spindle_velocity_gain = 0.0012
        self.spindle_position_gain = 100
        self.spindle_motor_current_limit = 20

        self.spindle_encoder_resolution = 8192
        self.spindle_max_rpm = 3000

        #Machine kinematics
        self.number_of_joints = 5
        self.servo_dt = 0.002
        #self.table_center_axis_travel_limits = [[-1.75, 2.55], [-2.05, 2.95], [-3.45, 0.1], [-5, 95], [-99999, 99999]]
        self.table_center_axis_travel_limits = [[-1.75, -2.05, -3.45, -5, -99999], [2.55, 2.95, 0.1, 95, 99999]]
        #self.absolute_axis_travel_limits = [4.25, 4.55, -3.55, 100, 99999]
        #self.absolute_axis_travel_limits = [[0, 0, -3.55, -5, -99999], [4.3, 5, 0, 95, 99999]]
        self.absolute_axis_travel_limits = [[-1.75, -2.05, -3.45, -5, -99999], [2.55, 2.95, 0.1, 95, 99999]]
        self.max_joint_velocity = [0.6666, 0.6666, 0.6666, 20, 20]
        self.tp_max_joint_velocity = [0.6666, 0.6666, 0.6666, 20 * np.pi / 180, 20 * np.pi / 180]
        self.max_joint_acceleration = [30, 30, 30, 1500, 1500]
        self.tp_max_joint_acceleration = [30, 30, 30, 1500 * np.pi / 180, 1500 * np.pi / 180]
        self.max_joint_jerk = [100, 100, 100, 100, 100]
        self.fk = []
        self.ik = []

        #Init states
        self.local_epoch = 0
        self.mode = self.modes[0]
        self.mode_stack = []
        self.status = self.statuses[2]

        self.estop = 0
        self.drive_power = 0
        self.echo = 1
        self.comm_mode = 0
        self.servo_feedback_mode = 0
        self.buffer_level_feedback_mode = 0
        self.buffer_level_feedback_period_us = 1e6/20
        self.units = 'inch'
        self.emc_timeout = 0

        #self.rsh_error = 0
        self.rsh_error = 0
        self.linked = 0
        self.connected = 0
        self.axis_home_state = [0]*self.number_of_joints
        self.current_move_serial_number = -1

        #Timing parameters
        self.clock_resolution = 1e6
        self.ping_tx_time = 0
        self.ping_rx_time = 0
        self.current_estimated_network_latency = 0
        self.mean_estimated_network_latency = 0
        self.OS_clock_offset = 0
        self.RT_clock_offset = 0
        self.RT_clock_sync_pncApp_time = 0
        self.pncApp_clock_offset = 0
        self.motion_controller_clock_offset = 0
        self.last_unix_time = 0
        self.clock_sync_received_time = 0
        self.motion_start_time = -1

        # Servo log parameters
        self.servo_log_type = self.servo_log_types.index('COMMANDED')
        self.servo_log_type = self.servo_log_types.index('MEASURED')
        self.servo_log_num_axes = 5
        self.servo_log_sub_sample_rate = 2
        #self.servo_log_buffer_size = 50
        self.servo_log_buffer_size = 50

        #Comm parameters
        self.rsh_socket = None
        self.endianness = 'little'
        self.size_of_feedback_double = 8
        self.size_of_feedback_int = 4
        self.bytes_to_receive = 65536
        self.binary_transmission_length = (self.servo_log_num_axes * self.servo_log_buffer_size + self.servo_log_buffer_size) * self.size_of_feedback_double + 1

        self.tcp_port = 5007
        self.ip_address = '129.1.15.5'
        self.simulator_ip_address = '127.0.0.1'
        #self.ip_address = '127.0.0.1'
        #self.ip_address = '129.1.15.69'
        #self.udp_port = 515
        #self.listen_ip = '0.0.0.0'
        self.comm_port = 'COM3'
        self.initial_baudrate = 115200
        self.target_baudrate = 250000
        self.serial_read_timeout = 0.5
        self.websocket_timeout = 5
        #self.websocket_client_GUID = '7ab22c19-3454-42ee-a68d-74c2789c4530'
        #self.websocket_server_GUID = '1ce49dd2-042c-4bec-95bd-790f0d0ece54'
        self.websocket_client_GUID = '7e93e9a1-ab35-4a28-bb08-2daf7823620c'
        self.websocket_server_GUID = '9b1b3197-8c81-48cb-a1e1-c0e5b8bce3b8'
        self.websocket_block_length = 20000
        self.ssh_port = 22
        self.ssh_credentials = ('pocketnc', 'pocketnc')
        self.ssh_opts = '-X'
        self.ssh_hosts_path = 'E:\\SculptPrint\\PocketNC\\OpenCNC\\Interface Application\\pncApp\\Support Files\\known_hosts'

        #TCP Control Parameters
        self.polylines_per_tx = 1
        self.points_per_polyline = 20
        self.max_motion_block_size = 1000
        self.buffer_level_setpoint = 500
        self.max_buffer_level = 2000
        self.motion_control_data_prebuffer_size = 20

        #Motion State Machine
        self.current_stepgen_position = np.asarray([0.0]*self.number_of_joints)
        self.current_encoder_position = [0.0]*self.number_of_joints
        self.current_velocity = [0.0]*self.number_of_joints
        self.current_acceleration = [0.0]*self.number_of_joints
        self.current_jerk = [0.0]*self.number_of_joints
        self.current_buffer_level = 0
        #self.motion_states = [self.current_stepgen_position, self.current_encoder_position, self.current_buffer_level]
        #self.motion_states = ['current_stepgen_position', 'current_encoder_position', 'current_buffer_level']
        self.motion_states = ['current_stepgen_position']#, 'current_encoder_position']
        #self.state_streams = ['STEPGEN_FEEDBACK_POSITIONS', 'ENCODER_FEEDBACK_POSITIONS', 'HIGHRES_TC_QUEUE_LENGTH']
        self.state_streams = ['STEPGEN_FEEDBACK_POSITIONS']#, 'ENCODER_FEEDBACK_POSITIONS']
        self.state_initialization_events = ['mc_initial_stepgen_position_set_event', 'ei_initial_encoder_position_set_event', 'mc_initial_buffer_level_set_event']

        #File Handling
        #self.point_files_path = 'E:\\SculptPrint\\PocketNC\\Position Sampling\\Diva Head\\Longest Path Yet\\'
        self.point_files_path = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Position Samples\\Longest Path Yet\\RA Points\\'
        self.raw_point_files_path = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Position Samples\\Longest Path Yet\\Raw\\'
        self.raw_point_files_path = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Position Samples\\Diva\\Raw\\'
        self.point_file_prefix = 'opt_code'
        self.log_file_output_directory = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Logs\\'
        self.database_output_directory = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Logs\\'
        self.database_file_name = 'database_output'

class MachineModelProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')

class MachineModelStateProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')

class MachineModelStaticsProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')
