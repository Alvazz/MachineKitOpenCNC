######### SCULPTPRINT INTEGRATION CODE #########
#import pnc
#import pncApp, time
#from pnc.pncApp import *

from pncApp import appInit, appClose
from pncDataStore import DataStore
import numpy as np
import threading
import copy
import threading, time

machine_feedback_record_id = 0
encoder_feedback_record_id = 0
LF_start_time_index, HF_start_time_index = (0, 0)

#There are two set of axis sensors: stepgens (0) and encoders (1)
axis_sensor_id = [0, 1]
SP_data_format = ['T','X','Z','S','Y','A','B','V','W','BL']

#from pncApp import machine_controller, data_store, feedback_listener
global feedback_listener, machine_controller, encoder_interface, data_store
#global machine

############################# Setup Functions #############################

def monitoredMachineCount():
    return 2

def setupMachineAuxilary():
    return setupAuxiliary()

def setupAuxiliary():
    #stringArray = [r'C', r'sinx', r'fx', r'gx', r'fx+gx', r'cos+cos', r'cos*sin', r'stepf']
    stringArray = [r'Buffer Level']
    return stringArray

def setupMachineAuxilary(nMachine):
    return setupMachineAuxiliary(nMachine)

def setupMachineAuxiliary(nMachine):
    if nMachine == 0:
        stringArray = [r'Buffer Level']
    else:
        stringArray = ['']
    return stringArray

def setupMachineDescriptors(nMachine):
    columnTypeArray = []
    if nMachine == 0:
        #Monitoring from BBB gives stepgen position and buffer level
        columnTypeArray = [0, 1, 1, 1, 1, 1, 1, 1, 1, 2]
    else:
        #Encoder positions are only time and axis position
        columnTypeArray = [0, 1, 1, 1, 1, 1, 1, 1, 1]
    return columnTypeArray

def setupUserDataNames():
    stringArray = [r'Start File', r'End File', r'my data 3', r'my data 4', r'my data 5']
    return stringArray

# Returns an array of user defined function names that are displayed on the function buttons in the feature UI. The array affects
# feature UI functionality only.  The array must be sized on the interval [1,3]. Defining this method is optional.
def setupUserFunctionNames():
    stringArray = [r'Enqueue Movements',r'Begin Motion Execution',r'my function 3']
    return stringArray

######################## Operation ########################
def start():
    #global machine_controller
    global machine, feedback_listener, machine_controller, encoder_interface, data_store, data_store_lock
    #commInit()
    print('initializing')
    #pnc.pncApp.appInit()
    machine, feedback_listener, machine_controller, encoder_interface, data_store = appInit()
	
    #Log machine feedback start point
    while machine_controller == []:
        print('busy waiting')
        pass

    #Log feedback start point
    machine_controller.machine.machine_feedback_written_record_id = machine_controller.data_store.machine_feedback_num_records
    machine_controller.machine.encoder_feedback_written_record_id = machine_controller.data_store.encoder_feedback_num_records

    #Configure SP data format in machine model
    machine_controller.machine.SP_data_format = SP_data_format

    #time.sleep(1)
    #print('logging in')

    #machine_controller.connectAndLink()
    machine.sculptprint_interface.connect_event.set()

    #machine_controller.setLogging(1)
    #machine_controller.waitForSet(machine_controller.setLogging,1,machine_controller.getLogging)

    # We are setup -- start commanding points using the motion controller
    #motion_controller.start()
    #time.sleep(3)
    #machine_controller.motion_controller.testMachine()
    #machine_controller.testMachine(1,1,1,1,1)
    #machine_controller.motion_controller._running_motion = True
    return True

# def read():
#     global feedback_listener, machine_controller, encoder_interface, data_store
#     #If appInit was successful
#     if not machine_controller == []:
#         #print('machine controller is ' + machine_controller)
#         #return feedbackData[-1]
#         #return [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]]
#
#         #Format data from data store in format requested by SP
#         #fb_data = [machine_controller.data_store.stepgen_feedback_positions[-1][np.array([0,2])].tolist(),[-90.], machine_controller.data_store.stepgen_feedback_positions[-1][np.array([1,3,4])].tolist(),[0.,0.]]
#         fb_data = [machine_controller.data_store.ENCODER_FEEDBACK_POSITIONS[-1][np.array([0, 2])].tolist(), [-90.],
#                    machine_controller.data_store.ENCODER_FEEDBACK_POSITIONS[-1][np.array([1, 3, 4])].tolist(), [0., 0.],
#                    [float(machine_controller.data_store.HIGHRES_TC_QUEUE_LENGTH[-1])]]
#         print(machine_controller.data_store.encoder_feedback_num_records)
#         print([[item for sublist in fb_data for item in sublist]])
#         print(machine_controller.data_store.ENCODER_FEEDBACK_POSITIONS[-1])
#         print('encoder feedback id is ' + str(machine_controller.machine.encoder_feedback_written_record_id))
#
#         #Return only new data and don't skip any due to thread sync
#         ##FIXME need mutex?
#         #if machine_controller.data_store.machine_feedback_num_records - machine_feedback_record_id:
#         current_record = machine_controller.data_store.encoder_feedback_num_records
#         if current_record - machine_controller.machine.encoder_feedback_written_record_id:
#             print('returning values')
#             machine_controller.machine.encoder_feedback_written_record_id = current_record
#             return [[data for sublist in fb_data for data in sublist]]
#         else:
#             print('returning no values')
#             print(str(machine_controller.data_store.encoder_feedback_num_records))
#             return []
#     return []

############################# Data Handling #############################

def findNextClosestTimeIndex(sample_time,data_time_array):
    if len(data_time_array) == 0:
        return -1

    delta_T = data_time_array - sample_time
    future_delta_T = delta_T[np.where(delta_T >= 0)]
    if len(future_delta_T) == 0:
        #Requested time is out of bounds for given time vector
        return -2

    time_index = future_delta_T.argmin() + len(delta_T[delta_T < 0])
    return time_index


def formatFeedbackDataForSP(time, positions, aux):
    #Expects a feedback vector in XYZAB format
    SP_formatted_data = [0]*len(SP_data_format)
    for label in range(0,len(SP_data_format)):
        label_text = SP_data_format[label]
        if label_text == 'X':
            SP_formatted_data[label] = float(positions[0])
        elif label_text == 'Y':
            SP_formatted_data[label] = float(positions[1])
        elif label_text == 'Z':
            SP_formatted_data[label] = float(positions[2])
        elif label_text == 'A':
            SP_formatted_data[label] = float(positions[3])
        elif label_text == 'B':
            SP_formatted_data[label] = float(positions[4])
        elif label_text == 'T':
            SP_formatted_data[label] = float((time-machine_controller.machine.RT_clock_offset)/machine.clock_resolution)
        elif label_text == 'BL':
            SP_formatted_data[label] = float(aux)
        elif label_text == 'S':
            SP_formatted_data[label] = -90.0
        else:
            SP_formatted_data[label] = 0.0
    return SP_formatted_data

def mergeSortByIndex(times_LF, times_HF, data_LF, data_HF):
    LF_index, HF_index = (0, 0)
    output_data = []
    while LF_index != times_LF.size and HF_index != times_HF.size:
        if times_LF[LF_index] < times_HF[HF_index]:
            data_time = times_LF[LF_index]
            data_positions = data_LF[LF_index]
            if HF_index-1 < 0:
                data_aux = 0
            else:
                data_aux = data_HF[HF_index-1]
            LF_index += 1
        elif times_LF[LF_index] == times_HF[HF_index]:
            data_time = times_LF[LF_index]
            data_positions = data_LF[LF_index]
            data_aux = data_HF[HF_index]
            LF_index += 1
            HF_index += 1
        else:
            #output_index_list.append((0, index2))
            data_time = times_HF[HF_index]
            if LF_index - 1 < 0:
                data_positions = machine_controller.machine.current_position
            else:
                data_positions = data_LF[LF_index-1]
            data_aux = data_HF[HF_index]
            HF_index += 1
        data_point = formatFeedbackDataForSP(data_time, data_positions, data_aux)
        output_data.append(data_point)

    while LF_index != times_LF.size:
        #output_index_list.append((0,index1))
        data_time = times_LF[LF_index]
        data_positions = data_LF[LF_index]
        if HF_index - 1 < 0:
            data_aux = 0
        else:
            data_aux = data_HF[HF_index - 1]
        LF_index += 1
        data_point = formatFeedbackDataForSP(data_time, data_positions, data_aux)
        output_data.append(data_point)

    while HF_index != times_HF.size:
        #output_index_list.append((1,index2))
        data_time = times_HF[HF_index]
        if LF_index - 1 < 0:
            data_positions = machine_controller.machine.current_position
        else:
            data_positions = data_LF[LF_index - 1]
        data_aux = data_HF[HF_index]
        HF_index += 1
        data_point = formatFeedbackDataForSP(data_time, data_positions, data_aux)
        output_data.append(data_point)

    if len(output_data):
        return output_data, LF_index, HF_index
    return None, 0, 0

def readMachine(axis_sensor_id):
    global feedback_listener, machine_controller, encoder_interface, data_store
    global LF_start_time_index, HF_start_time_index, highfreq_rx_time, lowfreq_rx_time
    #print('reading machine')
    ## FIXME should be incorporated below
    if not machine_controller.machine.logging_mode:
        print('logging not enabled, not returning data')
        return []

    if axis_sensor_id == 0:
        ## FIXME check that logging is enabled and that threads are set up right
        #Snapshot data store

        machine_controller.data_store.data_store_lock.acquire()
        data_store_snapshot = DataStore()
        data_store_snapshot.RSH_CLOCK_TIMES = np.copy(data_store.rsh_clock_times)
        data_store_snapshot.RTAPI_CLOCK_TIMES = np.copy(data_store.RTAPI_clock_times)
        data_store_snapshot.HIGHRES_TC_QUEUE_LENGTH = np.copy(data_store.highres_tc_queue_length)
        data_store_snapshot.STEPGEN_FEEDBACK_POSITIONS = np.copy(data_store.stepgen_feedback_positions)
        data_store_snapshot.LOWFREQ_ETHERNET_RECEIVED_TIMES = np.copy(data_store.lowfreq_ethernet_received_times)
        data_store_snapshot.HIGHFREQ_ETHERNET_RECEIVED_TIMES = np.copy(data_store.highfreq_ethernet_received_times)
        machine_controller.data_store.data_store_lock.release()
        #data_store_snapshot = copy.deepcopy(machine_controller.data_store)

        #Pull relevant section of data from data_store_snapshot
        #Serial_time_index = LF_time_index = findNextClosestTimeIndex(data_sample_time,data_store_snapshot.SERIAL_RECEIVED_TIMES)
        #LF_ethernet_time_slice = data_store_snapshot.lowfreq_ethernet_received_times[LF_start_time_index:]-machine_controller.machine.pncApp_clock_offset
        LF_ethernet_time_slice = data_store_snapshot.RTAPI_CLOCK_TIMES[LF_start_time_index:]# - machine_controller.machine.RT_clock_offset
        LF_ethernet_data_slice = data_store_snapshot.STEPGEN_FEEDBACK_POSITIONS[LF_start_time_index:]
        #HF_ethernet_time_slice = data_store_snapshot.highfreq_ethernet_received_times[HF_start_time_index:]-machine_controller.machine.RT_clock_offset
        HF_ethernet_time_slice = data_store_snapshot.RSH_CLOCK_TIMES[HF_start_time_index:]# - machine_controller.machine.RT_clock_offset
        HF_ethernet_data_slice = data_store_snapshot.HIGHRES_TC_QUEUE_LENGTH[HF_start_time_index:]


        BBB_feedback, LF_start_time_index_increment, HF_start_time_index_increment = mergeSortByIndex(LF_ethernet_time_slice,HF_ethernet_time_slice,LF_ethernet_data_slice,HF_ethernet_data_slice)
        LF_start_time_index += LF_start_time_index_increment
        HF_start_time_index += HF_start_time_index_increment
        #print(LF_start_time_index)
        if BBB_feedback is None:
            return []
        else:
            return BBB_feedback
    else:
        return np.asarray([[0, 1, 1, 1, 1, 1, 1, 1, 1]], dtype=float).tolist()


# Returns true if monitoring is currently happening.
def isMonitoring():
    return bool(machine_controller.is_alive() & machine_controller.machine.logging_mode)

############################# User Functions #############################

def userPythonFunction1(arg0, arg1, arg2, arg3, arg4):
    global machine
    print('execute enqueueMoves from ' + str(arg0) +' to ' + str(arg1))#(' + str(arg0) + ',' + str(arg1) + ',' + str(arg2) + ',' + str(arg3) + ',' + str(arg4) + ')\n')
    #machine_controller.testMachine(1,1,1,1,1)
    #machine_controller.motion_controller._running_motion = True
    machine.sculptprint_interface.start_file = arg0
    machine.sculptprint_interface.end_file = arg1
    machine.sculptprint_interface.enqueue_moves_event.set()
    return True;

def userPythonFunction2(arg0, arg1, arg2, arg3, arg4):
    print('execute userPythonFunction2(' + str(arg0) + ',' + str(arg1) + ',' + str(arg2) + ',' + str(arg3) + ',' + str(arg4) + ')\n')
    machine.sculptprint_interface.run_motion_event.set()
    return True;

def userPythonFunction3(arg0, arg1, arg2, arg3, arg4):
    print('execute userPythonFunction3(' + str(arg0) + ',' + str(arg1) + ',' + str(arg2) + ',' + str(arg3) + ',' + str(arg4) + ')\n')
    return True;

# Called to stop monitoring the machine.
# Will execute when the stop button is pressed in the Monitor Machine feature.
def stop():
    print('closing')
    appClose()
    return True

def testMonitoring():
    while True:
        z = readMachine(0)
        if z == []:
            print('returning nothing')
        else:
            print('returning good data')
        print('read machine')

start()

#machine.sculptprint_interface.enqueue_moves_event.set()
#machine.sculptprint_interface.run_motion_event.set()

#time.sleep(1)
#readMachine(0)
#
# while True:
#     readMachine(0)