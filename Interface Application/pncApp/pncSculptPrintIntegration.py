import pncLibrary
import multiprocessing, os, sys, time, logging, numpy as np
#from pncCamUserInterface import CAM_MVC

#There are two set of axis sensors: stepgens (0) and encoders (1)
#axis_sensor_id = [0, 1]
#SP_data_formats = [['T','X','Z','S','Y','A','B','V','W','BL'], ['T','X','Z','S','Y','A','B','V','W']]

############################# Setup Functions #############################

# def monitoredMachineCount():
#     return 2
#
# def setupMachineAuxilary():
#     return setupAuxiliary()
#
# def setupAuxiliary():
#     #stringArray = [r'C', r'sinx', r'fx', r'gx', r'fx+gx', r'cos+cos', r'cos*sin', r'stepf']
#     stringArray = [r'Buffer Level']
#     return stringArray
#
# def setupMachineAuxilary(nMachine):
#     return setupMachineAuxiliary(nMachine)
#
# def setupMachineAuxiliary(nMachine):
#     if nMachine == 0:
#         stringArray = [r'Buffer Level']
#     else:
#         stringArray = ['']
#     return stringArray
#
# def setupMachineDescriptors(nMachine):
#     #columnTypeArray = []
#     if nMachine == 0:
#         #Monitoring from BBB gives stepgen position and buffer level
#         columnTypeArray = [0, 1, 1, 1, 1, 1, 1, 1, 1, 2]
#     else:
#         #Encoder positions are only time and axis position
#         columnTypeArray = [0, 1, 1, 1, 1, 1, 1, 1, 1]
#     return columnTypeArray
#
# def setupUserDataNames():
#     stringArray = [r'Start File', r'End File', r'my data 3', r'my data 4', r'my data 5']
#     return stringArray
#
# # Returns an array of user defined function names that are displayed on the function buttons in the feature UI. The array affects
# # feature UI functionality only.  The array must be sized on the interval [1,3]. Defining this method is optional.
# def setupUserFunctionNames():
#     stringArray = [r'Initialize Control',r'Enqueue Movements',r'Execute Motion']
#     return stringArray

############################# Data Handling #############################

# def findNextClosestTimeIndex(sample_time,data_time_array):
#     if len(data_time_array) == 0:
#         return -1
#
#     delta_T = data_time_array - sample_time
#     future_delta_T = delta_T[np.where(delta_T >= 0)]
#     if len(future_delta_T) == 0:
#         #Requested time is out of bounds for given time vector
#         return -2
#
#     time_index = future_delta_T.argmin() + len(delta_T[delta_T < 0])
#     return time_index


def formatFeedbackData(sensor_type, data_format, data):
    SP_formatted_data = []
    if sensor_type == 1:
        print('sending number of encoder records: ' + str(len(data)))
    for data_point_index in range(0,len(data)):
        try:
            raw_data_point = np.hstack((data[data_point_index][k].flatten() for k in range(0, len(data[data_point_index]))))
        except Exception as error:
            print('break')
        output_data_point = [0.0] * len(pncLibrary.SP_data_formats[sensor_type])
        for label in range(0,len(pncLibrary.SP_data_formats[sensor_type])):
            label_text = pncLibrary.SP_data_formats[sensor_type][label]
            if label_text == 'S':
                output_data_point[label] = -90.0
            elif label_text not in data_format:
                output_data_point[label] = 0.0
            else:
                output_data_point[label] = raw_data_point[data_format.index(label_text)]
        SP_formatted_data.append(output_data_point)
    return SP_formatted_data

def formatFeedbackDataForSP(machine, sensor_type, times, positions, auxes=[]):
    SP_formatted_data = []
    if sensor_type == 1:
        print('sending number of encoder records: ' + str(len(times)))
    for data_point_index in range(0, len(times)):

        if len(auxes):
            time, position, aux = (times[data_point_index], positions[data_point_index], auxes[data_point_index])
        else:
            time, position = (times[data_point_index], positions[data_point_index])
        data_point = [0] * len(pncLibrary.SP_data_formats[sensor_type])
        for label in range(0, len(pncLibrary.SP_data_formats[sensor_type])):
            label_text = pncLibrary.SP_data_formats[sensor_type][label]
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
                data_point[label] = float(time)
            elif label_text == 'BL':
                data_point[label] = float(aux)
            elif label_text == 'S':
                data_point[label] = -90.0
            else:
                data_point[label] = 0.0
        SP_formatted_data.append(data_point)
    return SP_formatted_data

def mergeSort(feedback_state, time_arrays, data_arrays, fallback_data_samples, output_time_array = [], output_data_array = []):
    number_of_arrays = len(time_arrays)
    array_indices = [0]*len(time_arrays)
    while all([array_indices[k] != time_arrays[k].size for k in range(0, len(time_arrays)) if time_arrays[k].size != 0]) and not all([time_array.size == 0 for time_array in time_arrays]):
    #while all([array_indices[k] != time_arrays[k].size for k in range(0,len(time_arrays)) if time_arrays[k].size != 0]):
        time_sample, merged_data = None, number_of_arrays * [None]
        for k in range(0, len(time_arrays)):
            if all([time_arrays[k][array_indices[k]] < time_arrays[j][array_indices[j]] if time_arrays[k].size != 0 and time_arrays[j].size != 0 else False for j in range(0, len(time_arrays))]):
                if j == k:
                    continue
                time_sample = time_arrays[k][array_indices[k]]
                merged_data[k] = data_arrays[k][array_indices[k]]
                for j in range(0,number_of_arrays):
                    if j == k:
                        continue
                    if array_indices[j]-1 < 0:
                        merged_data[j] = fallback_data_samples[j]
                    else:
                        merged_data[j] = data_arrays[j][array_indices[j]-1]
                array_indices[k] += 1
                break
            elif any([time_arrays[k][array_indices[k]] == time_arrays[j][array_indices[j]] if j != k and time_arrays[k].size != 0 and time_arrays[j].size != 0 else False for j in range(0, len(time_arrays)) if j != k]):
                time_sample = time_arrays[k][array_indices[k]]
                merged_data[k] = data_arrays[k][array_indices[k]]
                for j in range(0,number_of_arrays):
                    if j == k:
                        continue
                    if time_arrays[k][array_indices[k]] != time_arrays[j][array_indices[j]]:
                        if array_indices[j]-1 < 0:
                            merged_data[j] = fallback_data_samples[j]
                        else:
                            merged_data[j] = data_arrays[j][array_indices[j]-1]
                    else:
                        merged_data[j] = data_arrays[j][array_indices[j]]
                        array_indices[j] += 1
                array_indices[k] += 1
                break
            else:
                dt = [time_arrays[j][array_indices[j]] - time_arrays[k][array_indices[k]] if time_arrays[k].size != 0 and time_arrays[j].size != 0 else np.inf for j in range(0, number_of_arrays)]
                kk = dt.index(min(dt))
                if not all([dt[j] == np.inf for j in range(0,number_of_arrays)]):
                    time_sample = time_arrays[kk][array_indices[kk]]
                    merged_data[kk] = data_arrays[kk][array_indices[kk]]
                    for j in range(0,number_of_arrays):
                        if j == kk:
                            continue
                        if array_indices[j]-1 < 0:
                            merged_data[j] = fallback_data_samples[j]
                        else:
                            merged_data[j] = data_arrays[j][array_indices[j]-1]
                    array_indices[kk] += 1
                    break
        if time_sample is not None:
            output_time_array.append(time_sample)
            output_data_array.append(merged_data)

    next_time_arrays = [time_arrays[k][array_indices[k]:] for k in range(0, number_of_arrays)]
    next_data_arrays = [data_arrays[k][array_indices[k]:] for k in range(0, number_of_arrays)]
    next_fallback_points = [fallback_data_samples[k] if time_arrays[k].size == 0 else np.array([data_arrays[k][array_indices[k]-1]]) for k in range(0, number_of_arrays)]

    if all([len(time_array) == 0 for time_array in next_time_arrays]):
        return output_time_array, output_data_array
    else:
        return mergeSort(feedback_state, next_time_arrays, next_data_arrays, next_fallback_points, output_time_array, output_data_array)

def mergeSortByIndex(machine, feedback_state, times_LF, times_HF, data_LF, data_HF):
    LF_index, HF_index = (0, 0)
    #output_data = []
    output_times, output_positions, output_auxes = [], [], []
    while LF_index != times_LF.size and HF_index != times_HF.size:
        if times_LF[LF_index] < times_HF[HF_index]:
            data_time = times_LF[LF_index]
            data_position = data_LF[LF_index]
            if HF_index-1 < 0:
                #data_aux = 0
                #data_aux = machine.current_buffer_level
                data_aux = feedback_state.last_buffer_level_reading
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
                #data_position = machine.current_stepgen_position
                data_position = feedback_state.last_position_reading
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
            #data_aux = machine.current_buffer_level
            data_aux = feedback_state.last_buffer_level_reading
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
            #data_position = machine.current_stepgen_position
            #data_position = machine.sculptprint_interface.last_position_reading
            data_position = feedback_state.last_position_reading
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

    return output_times, output_positions, output_auxes, LF_index, HF_index

# def updateInterfaceData(synchronizer, feedback_state, axis_sensor_id):
#     clock_stream_names = list(map(lambda stream: stream[0], pncLibrary.SP_main_data_streams[axis_sensor_id] + pncLibrary.SP_auxiliary_data_streams[axis_sensor_id]))
#     data_stream_names = list(map(lambda stream: stream[1], pncLibrary.SP_main_data_streams[axis_sensor_id] + pncLibrary.SP_auxiliary_data_streams[axis_sensor_id]))
#     clock_stream_sizes = list(map(lambda stream: stream[2][0], pncLibrary.SP_main_data_streams[axis_sensor_id] + pncLibrary.SP_auxiliary_data_streams[axis_sensor_id]))
#     data_stream_sizes = list(map(lambda stream: stream[2][1], pncLibrary.SP_main_data_streams[axis_sensor_id] + pncLibrary.SP_auxiliary_data_streams[axis_sensor_id]))
#     complete_stream_names = clock_stream_names + data_stream_names
#     complete_stream_sizes = clock_stream_sizes + data_stream_sizes
#
#     DB_query_data = pncLibrary.lockedPull(synchronizer, complete_stream_names, getFeedbackIndices(feedback_state, complete_stream_names), len(complete_stream_names) * [None])
#
#     time_slice, data_slice = DB_query_data[1][:len(clock_stream_names)], DB_query_data[1][-len(data_stream_names):]
#     feedback_state.feedback_indices = updateFeedbackIndices(feedback_state.feedback_indices, complete_stream_names, time_slice + data_slice)
#     feedback_state.last_values_read = updateFallbackDataPoints(feedback_state.last_values_read, complete_stream_names, complete_stream_sizes, time_slice + data_slice)

def readMachine(synchronizer, feedback_state, axis_sensor_id):
    data_format = pncLibrary.SP_pncApp_time + pncLibrary.SP_pncApp_machine_axes + pncLibrary.SP_pncApp_data_auxes[axis_sensor_id]
    if synchronizer.mvc_run_feedback_event.is_set():
        #FIXME add clock offset
        #if axis_sensor_id == 0 or 1:
        time_slice, data_slice, data_stream_names, data_stream_sizes = pncLibrary.updateInterfaceData('pull', synchronizer, feedback_state, pncLibrary.SP_main_data_streams, pncLibrary.SP_auxiliary_data_streams, [axis_sensor_id])
        merged_data = mergeSort(feedback_state, time_slice[0], data_slice[0], pncLibrary.getFallbackDataPoints(feedback_state.last_values_read, data_stream_names, data_stream_sizes), [], [])

        #FIXME this iteration here is probably not good
        return formatFeedbackData(axis_sensor_id, data_format, [[merged_data[0][k]] + merged_data[1][k] for k in range(0,len(merged_data[0]))])

        # elif axis_sensor_id == 1:
        #     DB_query_data = pncLibrary.lockedPull(synchronizer, ['SERIAL_RECEIVED_TIMES', 'ENCODER_FEEDBACK_POSITIONS'],
        #                                           2*[feedback_state.serial_start_time_index],
        #                                           2*[None])
        #
        #     serial_time_slice, serial_data_slice = DB_query_data[1]
        #     feedback_state.serial_start_time_index += len(serial_data_slice)
        #
        #     return formatFeedbackDataForSP(machine, axis_sensor_id, serial_time_slice, serial_data_slice)
    else:
        return []

# def getFeedbackIndices(feedback_state, data_stream_names):
#     return list(map(lambda stream: feedback_state.feedback_indices[stream] if stream in feedback_state.feedback_indices else 0, data_stream_names))
#     #return list(map(lambda indices, stream: indices[stream] if stream in indices else 0, feedback_state.feedback_indices, data_stream_names))
#
# def updateFeedbackIndices(feedback_indices, stream_names, streams):
#     return dict(map(lambda stream_name, stream: (stream_name, feedback_indices[stream_name] + len(stream) if stream_name in feedback_indices else len(stream)), stream_names, streams))
#     # for stream_name, stream in stream_names, streams:
#     #     feedback_state[stream_name] += len(stream)
#
# def getFallbackDataPoints(feedback_state, stream_names, stream_sizes):
#     return list(map(lambda stream_name, stream_size: feedback_state.last_values_read[stream_name] if stream_name in feedback_state.last_values_read else np.zeros(stream_size), stream_names, stream_sizes))
#
# def updateFallbackDataPoints(fallback_values, stream_names, stream_sizes, streams):
#     return dict(map(lambda stream_name, stream_size, stream: tuple((stream_name, stream[-1] if len(stream) > 0 else fallback_values[stream_name] if stream_name in fallback_values else np.zeros(stream_size))), stream_names, stream_sizes, streams))

    # for stream_name, stream in stream_names, streams:
    #     feedback_state.last_values_read[stream_name] = stream[-1] if len(stream) > 0 else feedback_state.last_values_read[stream_name]
    # feedback_state.last_values_read = [streams[k][-1] if len(streams[k]) > 0 else feedback_state.last_values_read]
    #
    # data_slice[k][-1] if len(data_slice[k]) > 0 else feedback_state.last_values_read[k] for k in range(0, len(data_slice))
    # feedback_state.last_values_read = dict(map(lambda stream, fallback_value: (stream, fallback_value), stream_names, fallback_values))

# Returns true if monitoring is currently happening.
def isMonitoring(synchronizer):
    try:
        monitoring_flag = synchronizer.process_start_signal.is_set() and synchronizer.mvc_run_feedback_event.is_set()
    except NameError:
        print('Synchronizer not set up yet')
        monitoring_flag = False
    return monitoring_flag
    #return synchronizer.process_start_signal.is_set()
    #return bool(machine.machine_controller_thread_handle.is_alive() & machine.servo_feedback_mode)

############################# User Functions #############################
def userPythonFunction1(arg0, arg1, arg2, arg3, arg4):
    sculptprint_MVC.command_queue.put('CONNECT')


# def userPythonFunction1(arg0, arg1, arg2, arg3, arg4):
#     #global machine
#     print('execute enqueueMoves from ' + str(arg0) +' to ' + str(arg1))#(' + str(arg0) + ',' + str(arg1) + ',' + str(arg2) + ',' + str(arg3) + ',' + str(arg4) + ')\n')
#     #machine_controller.testMachine(1,1,1,1,1)
#     #machine_controller.motion_controller._running_motion = True
#     machine.sculptprint_interface.start_file = arg0
#     machine.sculptprint_interface.end_file = arg1
#     machine.sculptprint_interface.enqueue_moves_event.set()
#     return True;

def userPythonFunction2(arg0, arg1, arg2, arg3, arg4):
    print('execute userPythonFunction2(' + str(arg0) + ',' + str(arg1) + ',' + str(arg2) + ',' + str(arg3) + ',' + str(arg4) + ')\n')
    sculptprint_MVC.command_queue.put('ENQUEUE ' + str(arg0) + ' ' + str(arg1))
    return True;

def userPythonFunction3(arg0, arg1, arg2, arg3, arg4):
    print('execute userPythonFunction3(' + str(arg0) + ',' + str(arg1) + ',' + str(arg2) + ',' + str(arg3) + ',' + str(arg4) + ')\n')
    sculptprint_MVC.command_queue.put('EXECUTE')
    return True;

# def connectToMachine():
#     sculptprint_MVC.command_queue.put('CONNECT')

# Called to stop monitoring the machine.
# Will execute when the stop button is pressed in the Monitor Machine feature.
def stop(synchronizer):
    #print('closing')
    #appClose()
    sculptprint_MVC.command_queue.put('CLOSE')
    synchronizer.mvc_app_shutdown_event.wait()
    return True

# def testMonitoring():
#     while True:
#         z = readMachine(0)
#         if z == []:
#             print('returning nothing')
#         else:
#             print('returning good data')
#         print('read machine')

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

#multiprocessing.set_executable(os.path.join(sys.exec_prefix, 'pythonw.exe'))
#multiprocessing.set_start_method('spawn')
#print('main name is ' + str(__name__))
