import pncTrajectoryGeneration as tp

######################## Useful Classes ########################
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
    def __init__(self, control_function, control_function_parameters, ack_function, ack_function_parameters = None):
        super(Command, self).__init__()
        self.control_function = control_function
        self.control_function_parameters = control_function_parameters
        self.ack_function = ack_function
        if ack_function_parameters != None:
            self.ack_function_parameters = ack_function_parameters

######################## IPC ########################
def getEventStatus(event, database_command_queue_proxy, database_output_queue_proxy):
    database_command_queue_proxy.put('get_event_status')

######################## State Machine ########################
def pushState(machine):
    # save machine state
    # self.prev_mode = self.mode
    machine.mode_stack += machine.mode
    # return #saved state structure

def popState(machine):
    # prev_mode = self.machine.mode_stack[-1]
    # mode_stack = self.machine.mode_stack[0:-1]
    return machine.mode_stack.pop()
    # return prev_mode

def resetRSHError(machine):
    print('before ' + str(machine.rsh_error))
    # super(MachineModel, self).rsh_error = 0
    machine.rsh_error = 0
    print('after ' + str(machine.rsh_error))
    # self.machine.rsh_error.value = 0

def checkOnOff(machine, flag):
    return machine.rsh_feedback_flags.index(flag.strip().upper()) % 2

def isAutoMode(machine):
    if machine.mode.upper() == 'AUTO' and machine.mode_change_event.isSet():
        return True
    else:
        return False

def isManualMode(machine):
    if machine.mode.upper() == 'MANUAL' and machine.mode_change_event.isSet():
        return True
    else:
        return False

def isHomed(machine):
    return (all(machine.axis_home_state) and machine.home_change_event.isSet())
    # if all(self.machine.axis_home_state) and self.machine.home_change_event.isSet():
    #     return True
    # else:
    #     return False

def isIdle(machine):
    if machine.status.upper() == 'IDLE' and machine.status_change_event.isSet():
        return True
    else:
        return False

######################## Clocking ########################
import time
def estimateMachineClock(machine, time_to_estimate=-1):
    if not machine.clock_event.isSet():
        print('WARNING: missing clock synchronization flag')

    if time_to_estimate == -1:
        time_to_estimate = time.clock()

    return machine.RT_clock_offset + (time_to_estimate - machine.estimated_network_latency) * machine.clock_resolution

######################## Comms ########################
def calculateBinaryTransmissionLength(machine):
    machine.binary_transmission_length = 2 + (
                machine.servo_log_num_axes * machine.servo_log_buffer_size + machine.servo_log_buffer_size) * machine.size_of_feedback_double + 1
    return machine.binary_transmission_length

######################## Data Handling ########################
import struct, numpy as np, csv
def convertFloat2Bin(num):
    return struct.pack('!f', num)

def convertBin2Float(bytes):
    return struct.unpack('!f', bytes)

def convertInt2Bin(num):
    return struct.pack('!i',num)

def formatBinaryLine(self,axisCoords,polyLines,blockLength,positionInFile):
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

def formatPoints(self, points, polylines, block_length):
    axis_coords = []
    #FIXME fix if not divisible by polylines*blocklength
    for axis in range(points.shape[1]):
        axis_coords.append(self.padAndFormatAxisPoints(np.asarray([points[:, axis]]).T, polylines, block_length))
    return np.asarray(axis_coords).transpose(1, 2, 0)

######################## Visualization ########################
import matplotlib.pyplot as plt
def visualizePoints(self, move_queue):
    points = np.empty((0,5), float)
    for move in move_queue:
        points = np.vstack((points,move.point_samples))
    plt.plot(points)
    plt.show()