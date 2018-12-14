import numpy as np
from pncLibrary import CloudTrajectoryPlannerState
from multiprocessing.managers import NamespaceProxy

# class MachineModelStatics():
#     def __init__(self):
#         self.name = None#Jokes
#         self.machine_name = "PocketVMC"
#         self.simulator_name = "the_danke$t"
#
#         #Terminal printouts
#         self.connection_string = "CONNECTED to {} remote shell at {}:{}"
#         self.failed_connection_string = "Could not connect to {0} remote shell at {1}:{2}. Is {0} ON?"
#         self.manager_launch_string = "PROCESS LAUNCHED: {} PID: {}"
#         self.process_launch_string = "PROCESS LAUNCHED: {} PID: {} from master {} PID: {}"
#         self.process_terminate_string = "PROCESS STOPPED: {} PID: {}"
#         self.pncApp_launch_string = "SUCCESS: pncApp launched on {} CPUs..."
#         self.pncApp_terminate_string = "SUCCESS: pncApp terminated without mess"
#         self.thread_launch_string = "THREAD LAUNCHED: {} process started {}"
#         self.thread_terminate_string = "THREAD STOPPED: {} process stopped {}"
#         self.device_boot_string = "DEVICE BOOTSTRAPPED: {} on {}"
#         self.connection_open_string = "CONNECTED to {} on {}"
#         self.connection_close_string = "DISCONNECTED from {} on {}"
#
#         #Machine kinematics
#         self.number_of_joints = 5
#         self.servo_dt = 0.001
#         self.table_center_axis_travel_limits = [[-1.75, 2.55], [-2.05, 2.95], [-3.45, 0.1], [-5, 95], [-99999, 99999]]
#         self.max_joint_velocity = [0.6666, 0.6666, 0.6666, 20, 20]
#         self.max_joint_acceleration = [30, 30, 30, 1500, 1500]
#         self.max_joint_jerk = [100, 100, 100, 100, 100]
#         self.fk = []
#         self.ik = []


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

        # File Handling
        self.work_transformation_file = 'machine_tableToPart.txt'
        self.tool_transformation_file = 'machine_toolToHolder.txt'
        #self.point_files_path = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Position Samples\\Longest Path Yet\\RA Points\\'
        #self.raw_point_files_path = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Position Samples\\Diva\\Raw\\'
        #self.log_file_output_directory = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Logs\\'
        #self.database_output_directory = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Logs\\'
        self.raw_point_files_path = 'E:\\SculptPrint\\PocketNC\\Position Samples\\Diva\\Raw\\'
        #self.database_output_directory = 'E:\\SculptPrint\\PocketNC\\Logs\\'
        self.log_file_output_directory = 'E:\\SculptPrint\\PocketNC\\Logs\\'
        self.database_file_name = 'database_output'

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

        #self.sculptprint_interface = None

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
        self.encoder_scale = [.096 / 8000, .096 / 8000, .096 / 8000, -1.0 / (8000 * 2 / 90), 1.0 / (8000 * 2 / 90)]
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
        self.servo_dt = 0.001
        self.position_epsilon = 0.001
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
        #self.fk = []
        #self.ik = []
        #self.kinematic_translation = np.array([0, 0, 0])
        #self.translation_vector = np.array([[0, 0, 0, 1]]).T

        # self.work_transformation_matrix = np.loadtxt(
        #     self.raw_point_files_path + self.work_transformation_file, skiprows=2).T
        # self.tool_transformation_matrix = np.loadtxt(
        #     self.raw_point_files_path + self.tool_transformation_file, skiprows=2).T
        # self.tool_translation_vector = np.dot(self.tool_transformation_matrix, self.translation_vector)[:-1]
        # self.workpiece_translation_vector = np.dot(self.work_transformation_matrix, self.translation_vector)[:-1]

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
        self.motion_start_time = 0

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
        self.ip_address = None
        self.machine_ip_address = '129.1.15.5'
        self.simulator_ip_address = '127.0.0.1'
        #self.ip_address = '127.0.0.1'
        #self.ip_address = '129.1.15.69'
        #self.udp_port = 515
        #self.listen_ip = '0.0.0.0'
        self.comm_port = 'COM12'
        self.initial_baudrate = 115200
        self.target_baudrate = 250000
        self.serial_read_timeout = 0.5
        self.websocket_timeout = 5
        #self.websocket_client_GUID = '7ab22c19-3454-42ee-a68d-74c2789c4530'
        #self.websocket_server_GUID = '1ce49dd2-042c-4bec-95bd-790f0d0ece54'
        self.websocket_type = "achex"
        self.nodered_websocket_url = "ws://gw14.iotfm.org:1880/ws/example"
        self.websocket_username = "remote_tp_client"
        self.websocket_tp_name = "remote_tp_server"
        self.websocket_client_GUID = '7e93e9a1-ab35-4a28-bb08-2daf7823620c'
        self.websocket_server_GUID = '9b1b3197-8c81-48cb-a1e1-c0e5b8bce3b8'
        self.websocket_block_length = 2000
        self.toolpath_point_buffer_length = 10
        self.ssh_port = 22
        self.ssh_credentials = None
        self.machine_ssh_credentials = ('pocketnc', 'pocketnc')
        self.simulator_ssh_credentials = ('machinekit', 'password')
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
        self.currently_executing_sequence_id = 4*[-1]
        self.currently_executing_move_type = 'NULL'
        #self.current_executing_CAM_sequence_id = -1
        #self.current_executing_rapid_sequence_id = -1
        self.motion_states = ['current_stepgen_position']#, 'current_encoder_position']
        #self.state_streams = ['STEPGEN_FEEDBACK_POSITIONS', 'ENCODER_FEEDBACK_POSITIONS', 'HIGHRES_TC_QUEUE_LENGTH']
        self.state_streams = ['STEPGEN_FEEDBACK_POSITIONS']#, 'ENCODER_FEEDBACK_POSITIONS']
        self.state_initialization_events = ['mc_initial_stepgen_position_set_event', 'ei_initial_encoder_position_set_event', 'mc_initial_buffer_level_set_event']

        self.tp_state = CloudTrajectoryPlannerState()
        self.toolpath_data = None

        self.tp_state_rapid_sequence_id = 0
        self.tp_state_enqueued_sequence_id = -1

        # First element is number of subsequences in outbound transmission, second is number in inbound transmission
        self.tp_state_number_of_subsequences = [-1, -1]
        # First element is SP_trajectory requested ID, second is rapid requested ID
        self.tp_state_sequence_ack_id = [-1, -1]
        self.tp_state_subsequence_ack_id = [-1, -1]
        self.tp_state_current_received_sequence_id = [-1, -1]
        self.tp_state_current_received_subsequence_id = [-1, -1]
        self.tp_state_current_requested_sequence_id = [-1, -1]
        self.tp_state_current_requested_subsequence_id = [-1, -1]

        self.tp_state_last_CAM_sequence_end_points = np.empty((6, 0))
        self.tp_state_last_CAM_sequence_tool_end_points = np.empty((5, 0))
        self.tp_state_last_CAM_sequence_end_volumes = np.empty((1,0))
        self.tp_state_flag_set_count = 0

    # def loadTransformations(self):
    #     self.work_transformation_matrix = np.loadtxt(
    #         self.raw_point_files_path + self.work_transformation_file, skiprows=2).T
    #     self.tool_transformation_matrix = np.loadtxt(
    #         self.raw_point_files_path + self.tool_transformation_file, skiprows=2).T

    # Take a point xin, yin, zin expressed in local coordinates attached to
    # cutting tool holder to tool space coordinates in world frame
    def FK(self, X, Y, Z, A, B, S, xin, yin, zin, translation):
        X, Y, Z, A, B, S, xin, yin, zin = tuple(np.atleast_1d(X, Y, Z, A, B, S, xin, yin, zin))
        return (1.0 * X * np.sin(B) - 1.0 * Y * np.sin(A) * np.cos(B) + 1.0 * Z * np.cos(A) * np.cos(B) + 1.0 * xin * np.sin(A) * np.sin(
            S) * np.cos(B) -
                1.0 * xin * np.sin(B) * np.cos(S) + 1.0 * yin * np.sin(A) * np.cos(B) * np.cos(S) + 1.0 * yin * np.sin(B) * np.sin(
            S) - 1.0 * zin * np.cos(A) * np.cos(B)
                - 0.000135 * np.sin(A) * np.cos(B) - 0.000291 * np.sin(B) +
                4.069679 * np.cos(A) * np.cos(B) - 1.002354 + translation[0],

                1.0 * X * np.cos(B) + 1.0 * Y * np.sin(A) * np.sin(B) - 1.0 * Z * np.sin(B) * np.cos(A) - 1.0 * xin * np.sin(A) * np.sin(
                    B) * np.sin(S) -
                1.0 * xin * np.cos(B) * np.cos(S) - 1.0 * yin * np.sin(A) * np.sin(B) * np.cos(S) + 1.0 * yin * np.sin(S) * np.cos(B) +
                1.0 * zin * np.sin(B) * np.cos(A) + 0.000135 * np.sin(A) * np.sin(B) -
                4.069679 * np.sin(B) * np.cos(A) - 0.000291 * np.cos(B) + 1.0e-6 +
                translation[1],

                1.0 * Y * np.cos(A) + 1.0 * Z * np.sin(A) - 1.0 * xin * np.sin(S) * np.cos(A) - 1.0 * yin * np.cos(A) * np.cos(
                    S) - 1.0 * zin * np.sin(A) +
                4.069679 * np.sin(A) + 0.000135 * np.cos(A) + 0.372 + translation[2],

                -B,

                np.pi / 2 - A)

    # Take a point having local coordinates local, and world coordinates world
    # (both in tool space) to joint space
    def IK(self, world, local, theta, phi, S, translation):
        A = np.pi / 2 - phi
        B = -theta
        IK_matrix = np.array(
            [[np.sin(B), np.cos(B), np.zeros_like(B)],
             [-np.sin(A) * np.cos(B), np.sin(A) * np.sin(B), np.cos(A)],
             [np.cos(A) * np.cos(B), -np.sin(B) * np.cos(A), np.sin(A)]]).transpose()

        xin, yin, zin = local
        b_vector = np.array(
            [world[0, :] - translation[0] - (
                        1.0 * xin * np.sin(A) * np.sin(S) * np.cos(B) - 1.0 * xin * np.sin(B) * np.cos(S) + 1.0 * yin * np.sin(
                    A) * np.cos(B) * np.cos(S) + 1.0 * yin * np.sin(B) * np.sin(S) -
                        1.0 * zin * np.cos(A) * np.cos(B) - 0.000135 * np.sin(A) * np.cos(B) - 0.000291 * np.sin(
                    B) + 4.069679 * np.cos(A) * np.cos(B) - 1.002354),
             world[1, :] - translation[1] - (
                         - 1.0 * xin * np.sin(A) * np.sin(B) * np.sin(S) - 1.0 * xin * np.cos(B) * np.cos(S) - 1.0 * yin * np.sin(
                     A) * np.sin(B) * np.cos(S) + 1.0 * yin * np.sin(S) * np.cos(B) +
                         1.0 * zin * np.sin(B) * np.cos(A) + 0.000135 * np.sin(A) * np.sin(B) - 4.069679 * np.sin(B) * np.cos(
                     A) - 0.000291 * np.cos(B) + 1.0e-6),
             world[2, :] - translation[2] - (
                         - 1.0 * xin * np.sin(S) * np.cos(A) - 1.0 * yin * np.cos(A) * np.cos(S) - 1.0 * zin * np.sin(
                     A) + 4.069679 * np.sin(A) + 0.000135 * np.cos(A) + 0.372)]
        ).transpose()
        X_ik = np.zeros_like(b_vector)
        for index in np.arange(IK_matrix.shape[0]):
            X_ik[index] = IK_matrix[index].T.dot(b_vector[index])
        X_ik = X_ik.transpose()
        return np.array([X_ik[0], X_ik[1], X_ik[2], A, B, S]).T



class MachineModelProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')

class MachineModelStateProxy(NamespaceProxy):
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')

# class MachineModelStaticsProxy(NamespaceProxy):
#     _exposed_ = ('__getattribute__', '__setattr__', '__delattr__')
