import pncTrajectoryGeneration as tp
import sys
from threading import Thread
from multiprocessing import Event, current_process

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

        self.last_time_reading = 0
        self.last_position_reading = 0
        self.last_buffer_level_reading = 0

        self.machine_feedback_record_id = 0
        self.encoder_feedback_record_id = 0

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
            sys.stdout.flush()

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
        self.t_run_logger_event = manager.Event()
        self.t_run_puller_event = manager.Event()
        self.t_run_pusher_event = manager.Event()
        self.t_run_state_manipulator_event = manager.Event()
        self.t_run_print_server_event = manager.Event()
        self.t_run_cloud_trajectory_planner = manager.Event()

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
    def __init__(self,point_samples,move_type = None, filename = None):
        super(Move, self).__init__()
        self.serial_number = -1
        self.filename = ''
        self.point_samples = point_samples
        self.move_type = move_type

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

        # self.control_function = control_function
        # self.control_function_parameters = control_function_parameters
        # self.ack_function = ack_function
        # if ack_function_parameters != None:
        #     self.ack_function_parameters = ack_function_parameters

class DatabaseCommand():
    def __init__(self, command_type, records, parameters = None, time = None):
        self.command_type = command_type
        self.data = records
        self.command_parameters = parameters
        self.time = time

class RSHError(Exception):
    def __init__(self, message, errors):
        super().__init__(message)
        self.errors = errors

class WebsocketError(Exception):
    def __init__(self, message):
        self.message = message

######################## Multitask Management ########################
def startPrintServer(machine, synchronizer):
    print_server = PrintServer(machine, synchronizer)
    print_server.start()
    print_server.startup_event.wait()
    printTerminalString(machine.thread_launch_string, current_process().name, print_server.name)
    return print_server

def waitForThreadStart(args):
    # FIXME implement this
    pass

######################## IPC ########################
def getEventStatus(event, database_command_queue_proxy, database_output_queue_proxy):
    database_command_queue_proxy.put('get_event_status')

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

def restoreState(machine):
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
import time
def estimateMachineClock(machine, time_to_estimate=-1):
    # if not machine.clock_event.isSet():
    #     print('WARNING: missing clock synchronization flag')

    if time_to_estimate == -1:
        time_to_estimate = time.clock()

    return machine.RT_clock_offset + (time_to_estimate - machine.estimated_network_latency) * machine.clock_resolution

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

def padAndFormatAxisPoints(points, polylines, blocklength):
    pad_points = np.lib.pad(points, ((0, blocklength - (np.size(points, 0) % blocklength)), (0, 0)), 'constant',
                            constant_values=points[-1])
    shape_points = pad_points.reshape((-1, blocklength), order='C')
    return np.pad(shape_points, ((0, polylines - (np.size(shape_points, 0) % polylines)), (0, 0)), 'constant',
                  constant_values=shape_points[-1, -1])

def importAxisPoints(file, polylines, blocklength):
    points = np.array(list(csv.reader(open(file, "rt"), delimiter=","))).astype("float")
    return padAndFormatAxisPoints(points, polylines, blocklength)

def importAxesPoints(file, machine):
    ##FIXME check for overtravel
    points = np.array(list(csv.reader(open(file, "rt"), delimiter=" "))).astype("float")[:,:machine.number_of_joints]
    return points

def formatPoints(points, polylines, block_length):
    axis_coords = []
    #FIXME fix if not divisible by polylines*blocklength
    for axis in range(points.shape[1]):
        axis_coords.append(padAndFormatAxisPoints(np.asarray([points[:, axis]]).T, polylines, block_length))
    return np.asarray(axis_coords).transpose(1, 2, 0)

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