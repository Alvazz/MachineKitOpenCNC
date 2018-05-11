#from pncMachineControl import MachineController
import threading
import queue

class MachineModel():
    global machine_controller

    def __init__(self):
        #State variables
        self.modes = ['MANUAL', 'MDI', 'AUTO']
        self.statuses = ['IDLE', 'RUNNING', 'PAUSED']
        self.rsh_feedback_strings = ['*', 'bL=', 'bT=', 'PROGRAM_STATUS', 'MODE', 'ON', 'SERVO_LOG_PARAMS', 'MACHINE', 'ECHO', 'HELLO', 'EMCTOO', 'ESTOP', 'NAK']
        self.rsh_feedback_flags = ['OFF', 'ON']
        self.axes = ['X','Y','Z','A','B']

        #Stepgen calibration
        self.axis_offsets = [-0.00085, 2.5, .0013, .114, -.002]
        self.axis_offsets = [-2.5, -2.5, -0.1, 0.0, 0.0]
        self.axis_offsets = [0.0, 0.0, 0.0, 0.0, 0.0]
        #self.table_zero = [-2.55023, 0.00002, -0.0003, -0.03703, 360.03]
        #self.table_zero = [0.00087, 0.00002, -0.00234, 0.227, 360.003]
        self.table_zero = [0.0, 0.0, 0.0, 0.0, 0.0]

        #Encoder calibration
        self.machine_zero = [-1.75, -2.05, 0.1, -5.0, 0.0]
        self.encoder_init = 1000000
        self.encoder_offset = [155836, 180838, 2283, 9121, 0]
        #self.encoder_scale = [1/5/8000, 1/5/8000, 1/5/8000, 1/35.5368/8000, 1/35.5555/8000]
        self.encoder_scale = [.096 / 8000, .096 / 8000, .096 / 8000, -1.0 / 172, -1.0 / 167]

        #Machine kinematics
        self.num_joints = 5
        self.servo_dt = 0.001
        self.limits = [[-1.75, 2.55], [-2.05, 2.95], [-3.45, 0.1], [-5, 95], [-99999, 99999]]
        self.max_joint_velocity = [0.6666, 0.6666, 0.6666, 20, 20]
        self.max_joint_acceleration = [30, 30, 30, 1500, 1500]
        self.max_joint_jerk = [100, 100, 100, 100, 100]
        self.fk = []
        self.ik = []

        #Init states
        self.mode = self.modes[0]
        self.mode_stack = []
        self.status = self.statuses[2]

        self.estop = 0
        self.drive_power = 0
        self.echo = 1
        self.binary_mode = 0
        self.logging_mode = 0
        self.units = 'inch'

        self.rsh_error = 0
        self.linked = 0
        self.connected = 0
        self.homed = [0, 0, 0, 0, 0]
        self.current_move_serial_number = -1

        #State Switch Events
        self.connection_change_event = threading.Event()
        self.link_change_event = threading.Event()
        self.echo_change_event = threading.Event()
        self.estop_change_event = threading.Event()
        self.drive_power_change_event = threading.Event()
        self.status_change_event = threading.Event()
        self.status_change_event.clear()
        self.mode_change_event = threading.Event()
        self.logging_mode_change_event = threading.Event()

        #Comm parameters
        self.tcp_port = 5007
        self.ip_address = '129.1.15.5'
        #self.ip_address = '129.1.15.69'
        self.udp_port = 515
        self.listen_ip = '0.0.0.0'
        self.comm_port = 'COM12'
        self.ssh_opts = '-X'
        self.ssh_credentials = 'pocketnc@' + self.ip_address

        #Servo log parameters
        self.servo_log_num_axes = 5
        self.servo_log_sub_sample_rate = 10
        self.servo_log_buffer_size = 50

        #TCP Control Parameters
        self.polylines_per_tx = 1
        self.points_per_polyline = 25
        self.buffer_level_setpoint = 1000
        self.max_buffer_level = 2000

        #Motion State Machine
        self.current_position = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_velocity = [0, 0, 0, 0, 0]
        self.current_acceleration = [0, 0, 0, 0, 0]
        self.current_jerk = [0, 0, 0, 0, 0]

        #State stack
        self.prev_mode = self.modes[0]

        #SculptPrint Data Format
        #self.SP_data_format = ['X','Z','S','Y','A','B','V','W']

    ######################## State Machine ########################
    def pushState(self):
        #save machine state
        #self.prev_mode = self.mode
        self.mode_stack += self.mode
        #return #saved state structure

    def popState(self):
        #prev_mode = self.mode_stack[-1]
        #mode_stack = self.mode_stack[0:-1]
        return self.mode_stack.pop()
        #return prev_mode

    def restoreState(self, stateStruct):
        machine_controller.modeSwitchWait(self.prev_mode)
        #Restore state to prev state after op
        return

    ###################################################################
    # def setLoggingMode(self, mode):
    #     self.logging_mode = mode
    #     self.logging_mode_changed_callback(mode)

    def isAutoMode(self):
        if self.mode.upper() == 'AUTO' and self.mode_change_event.isSet():
            return True
        else:
            return False

    def isIdle(self):
        if self.status.upper() == 'IDLE' and self.status_change_event.isSet():
            return True
        else:
            return False