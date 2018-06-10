import sys, numpy as np
print('Current PYTHONPATH is ' + str(sys.path))
project_path = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\MachineKitOpenCNC\\Interface Application\\pncApp\\'
if project_path not in sys.path:
    sys.path.append('C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\MachineKitOpenCNC\\Interface Application\\pncApp\\')
from pncApp import appInit, appStart, appStop, appClose
from pncDatabase import DatabaseServer
import multiprocessing, inspect
from pncMachineModel import MachineModelProxy, MachineModel
from threading import current_thread
import os, sys
import pncLibrary

import pncMachineModel

print(appInit)

machine_feedback_record_id = 0
encoder_feedback_record_id = 0
LF_start_time_index, HF_start_time_index = (0, 0)

#There are two set of axis sensors: stepgens (0) and encoders (1)
axis_sensor_id = [0, 1]
SP_data_formats = [['T','X','Z','S','Y','A','B','V','W','BL'], ['T','X','Z','S','Y','A','B','V','W']]
machine = 0
#from pncApp import machine_controller, data_store, feedback_listener
#global feedback_listener, machine_controller, encoder_interface, data_store
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

class PNCAppManager(multiprocessing.managers.SyncManager): pass

def registerProxy(name, cls, proxy, manager):
    for attr in dir(cls):
        if inspect.ismethod(getattr(cls, attr)) and not attr.startswith("__"):
            proxy._exposed_ += (attr,)
            setattr(proxy, attr,
                    lambda s: object.__getattribute__(s, '_callmethod')(attr))
    manager.register(name, cls, proxy)

######################## Operation ########################
def start():
    #global machine_controller
    #global machine#, feedback_listener, machine_controller, encoder_interface, data_store, data_store_lock
    #commInit()
    #print('initializing')
    #pnc.pncApp.appInit()
    #global pnc_app_manager
    pnc_app_manager, machine, synchronizer = appInit()
    pncLibrary.startPrintServer(machine, synchronizer)

    main_process_name = str(multiprocessing.current_process().name)
    main_process_pid = str(multiprocessing.current_process().pid)

    pncLibrary.printTerminalString(machine.sculptprint_interface_initialization_string, main_process_name, main_process_pid)

    database, encoder_interface, feedback_handler, machine_controller = appStart('pocketnc', pnc_app_manager, machine, synchronizer)

    # print('Current process: ' + str(multiprocessing.current_process().pid) + ', current thread: ' + str(
    #     current_thread().name))
    # registerProxy('MachineModel', MachineModel, MachineModelProxy, PNCAppManager)
    # # registerProxy('Synchronizers', Synchronizers, SynchronizersProxy, PNCAppManager)
    # # registerProxy('DatabaseServer', DatabaseServer, DatabaseServerProxy)
    #
    # print('name is: ' + str(__name__))
    #
    # # FIXME ignore SIGINT
    print('starting new process')
    # pnc_app_manager = PNCAppManager()
    # pnc_app_manager.name = 'pnc_app_manager'
    # pnc_app_manager.start()

    #database = DatabaseServer(0, 0)
    #database.start()

    pnc_app_manager, machine, database, encoder_interface, machine_controller, feedback_handler, synchronizer = appInit('pocketnc')
	
    #Log machine feedback start point
    # while machine_controller == []:
    #     print('busy waiting')
    #     pass

    #Log feedback start point
    #machine.machine_feedback_written_record_id = machine_controller.data_store.machine_feedback_num_records
    #machine.encoder_feedback_written_record_id = machine_controller.data_store.encoder_feedback_num_records

    #Configure SP data format in machine model
    #machine.SP_data_format = SP_data_formats

    #machine.sculptprint_interface.connect_event.set()

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


def formatFeedbackDataForSP(sensor_type, times, positions, auxes=[]):
    #SP_formatted_data = len(times) * [[0] * len(SP_data_format)]
    SP_formatted_data = []
    for data_point_index in range(0,len(times)):
        if len(auxes):
            time, position, aux = (times[data_point_index], positions[data_point_index], auxes[data_point_index])
        else:
            time, position = (times[data_point_index], positions[data_point_index])
        data_point = [0] * len(SP_data_formats[sensor_type])
        for label in range(0,len(SP_data_formats[sensor_type])):
            label_text = SP_data_formats[sensor_type][label]
            if label_text == 'X':
                data_point[label] = float(position[0])
            elif label_text == 'Y':
                data_point[label] = float(position[1])
            elif label_text == 'Z':
                data_point[label] = float(position[2])
            elif label_text == 'A':
                data_point[label] = float(position[3])
            elif label_text == 'B':
                data_point[label] = float(position[4])
            elif label_text == 'T':
                data_point[label] = float((time-machine.RT_clock_offset)/machine.clock_resolution)
            elif label_text == 'BL':
                data_point[label] = float(aux)
            elif label_text == 'S':
                data_point[label] = -90.0
            else:
                data_point[label] = 0.0
        SP_formatted_data.append(data_point)
    return SP_formatted_data

def mergeSortByIndex(times_LF, times_HF, data_LF, data_HF):
    LF_index, HF_index = (0, 0)
    #output_data = []
    output_times, output_positions, output_auxes = [], [], []
    while LF_index != times_LF.size and HF_index != times_HF.size:
        if times_LF[LF_index] < times_HF[HF_index]:
            data_time = times_LF[LF_index]
            data_position = data_LF[LF_index]
            if HF_index-1 < 0:
                #data_aux = 0
                #data_aux = machine.rsh_buffer_level
                data_aux = machine.sculptprint_interface.last_buffer_level_reading
            else:
                data_aux = data_HF[HF_index-1]
            LF_index += 1
        elif times_LF[LF_index] == times_HF[HF_index]:
            data_time = times_LF[LF_index]
            data_position = data_LF[LF_index]
            data_aux = data_HF[HF_index]
            LF_index += 1
            HF_index += 1
        else:
            #output_index_list.append((0, index2))
            data_time = times_HF[HF_index]
            if LF_index - 1 < 0:
                #data_position = machine.current_position
                data_position = machine.sculptprint_interface.last_position_reading
            else:
                data_position = data_LF[LF_index-1]
            data_aux = data_HF[HF_index]
            HF_index += 1
        #data_point = formatFeedbackDataForSP(data_time, data_positions, data_aux)
        output_times.append(data_time)
        output_positions.append(data_position)
        output_auxes.append(data_aux)
        #output_data.append(data_point)

    while LF_index != times_LF.size:
        #output_index_list.append((0,index1))
        data_time = times_LF[LF_index]
        data_position = data_LF[LF_index]
        if HF_index - 1 < 0:
            #data_aux = machine.rsh_buffer_level
            data_aux = machine.sculptprint_interface.last_buffer_level_reading
        else:
            data_aux = data_HF[HF_index - 1]
        LF_index += 1
        #data_point = formatFeedbackDataForSP(data_time, data_positions, data_aux)
        output_times.append(data_time)
        output_positions.append(data_position)
        output_auxes.append(data_aux)
        #output_data.append(data_point)

    while HF_index != times_HF.size:
        #output_index_list.append((1,index2))
        data_time = times_HF[HF_index]
        if LF_index - 1 < 0:
            #data_position = machine.current_position
            data_position = machine.sculptprint_interface.last_position_reading
        else:
            data_position = data_LF[LF_index - 1]
        data_aux = data_HF[HF_index]
        HF_index += 1
        #FIXME do this in readMachine instead
        #data_point = formatFeedbackDataForSP(data_time, data_positions, data_aux)
        output_times.append(data_time)
        output_positions.append(data_position)
        output_auxes.append(data_aux)
        #output_data.append(data_point)

    if len(output_times) > 0:
        #return output_data, LF_index, HF_index
        return output_times, output_positions, output_auxes, LF_index, HF_index
    return None, None, None, 0, 0

def readMachine(axis_sensor_id):
    global machine
    #global feedback_listener, machine_controller, encoder_interface, data_store
    global LF_start_time_index, HF_start_time_index#, highfreq_rx_time, lowfreq_rx_time
    #print('reading machine')
    ## FIXME should be incorporated below, can this check be removed?
    #if not machine.servo_feedback_mode:
        #print('logging not enabled, not returning data')
        #return []
    return []
    if axis_sensor_id == 0:
        ## FIXME check that logging is enabled and that threads are set up right
        #Snapshot data store

        #DB_query_data = pncLibrary.lockedPull

        # DB_query_data = machine.data_store_manager_thread_handle.pull(
        #     ['RSH_CLOCK_TIMES','HIGHRES_TC_QUEUE_LENGTH','RTAPI_CLOCK_TIMES','STEPGEN_FEEDBACK_POSITIONS'],
        #     [HF_start_time_index,HF_start_time_index,LF_start_time_index,LF_start_time_index],
        #     [None,None,None,None])

        HF_ethernet_time_slice, HF_ethernet_data_slice, LF_ethernet_time_slice, LF_ethernet_data_slice = DB_query_data[1]

        if len(HF_ethernet_data_slice) == 0 and len(LF_ethernet_data_slice) == 0:
            return []

        #BBB_feedback, LF_start_time_index_increment, HF_start_time_index_increment = mergeSortByIndex(LF_ethernet_time_slice,HF_ethernet_time_slice,LF_ethernet_data_slice,HF_ethernet_data_slice)
        times, positions, auxes, LF_start_time_index_increment, HF_start_time_index_increment = mergeSortByIndex(
            LF_ethernet_time_slice, HF_ethernet_time_slice, LF_ethernet_data_slice, HF_ethernet_data_slice)

        LF_start_time_index += LF_start_time_index_increment
        HF_start_time_index += HF_start_time_index_increment
        #print(LF_start_time_index)
        # if any([d is None for d in [times, positions, auxes]]):
        #     return []
        # else:
        machine.sculptprint_interface.last_time_reading = times[-1]
        machine.sculptprint_interface.last_position_reading = positions[-1]#.tolist()
        machine.sculptprint_interface.last_buffer_level_reading = auxes[-1]#.item()
        #BBB_feedback = formatFeedbackDataForSP(axis_sensor_id, times, positions, auxes)
        print('HF ethernet data slice is ' + str(HF_ethernet_data_slice))

        return formatFeedbackDataForSP(axis_sensor_id, times, positions, auxes)
        # if BBB_feedback is None:
        #     return []
        # else:
        #     return BBB_feedback

    elif axis_sensor_id == 1:
        DB_query_data = machine.data_store_manager_thread_handle.pull(
            ['SERIAL_RECEIVED_TIMES', 'ENCODER_FEEDBACK_POSITIONS'],
            2*[machine.sculptprint_interface.serial_start_time_index],
            2*[None])

        serial_time_slice, serial_data_slice = DB_query_data[1]
        machine.sculptprint_interface.serial_start_time_index += len(serial_data_slice)

        # if len(serial_time_slice) == 0:
        #     return 0

        #encoder_feedback = formatFeedbackDataForSP(axis_sensor_id, serial_time_slice, serial_data_slice)
        return formatFeedbackDataForSP(axis_sensor_id, serial_time_slice, serial_data_slice)
        # if encoder_feedback is None:
        #     return []
        # else:
        #     return encoder_feedback

    else:
        return np.asarray([[0, 1, 1, 1, 1, 1, 1, 1, 1]], dtype=float).tolist()

# Returns true if monitoring is currently happening.
def isMonitoring():
    return 1
    return bool(machine.machine_controller_thread_handle.is_alive() & machine.servo_feedback_mode)

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

# if False:
#     read_data0 = []
#     time_to_read0 = []
#     read_data1 = []
#     start()
#     machine.sculptprint_interface.enqueue_moves_event.set()
#     machine.sculptprint_interface.run_motion_event.set()
#     while True:
#         start = time.clock()
#         data0 = readMachine(0)
#         time_to_read0.append(time.clock()-start)
#         data1 = readMachine(1)
#         #print(data)
#         if data0 != []:
#             print('appending data0')
#             read_data0.append(data0)
#         if data1 != []:
#             print('appending data1')
#             read_data1.append(data1)
    #else:
        #print('not appending data')

#time.sleep(1)
#readMachine(0)
#
# while True:
#     readMachine(0)
# print('here')
# if __name__ == '__main__':
#     start()
multiprocessing.set_executable(os.path.join(sys.exec_prefix, 'pythonw.exe'))
print('main name is ' + str(__name__))
if __name__ == '__main__':
    multiprocessing.freeze_support()
    start()