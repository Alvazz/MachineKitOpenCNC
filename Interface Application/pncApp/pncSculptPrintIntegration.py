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

############################# Data Handling #############################
NUMAXES = 8
TOOLPATHPOINTSIZE = 11
# DATASOURCEFORMATS = [(r'Planned Trajectory', 'COMMANDED_SERVO_POSITIONS', NUMAXES + 1),
#                    (r'Actual Trajectory', 'ENCODER_FEEDBACK_POSITIONS', NUMAXES + 1),
#                    (r'Estimated Trajectory', 'STEPGEN_FEEDBACK_POSITIONS', NUMAXES + 1)]
AUXILIARYDATASOURCEFORMATS = [(r'Servo Buffer Level', 'HIGHRES_TC_QUEUE_LENGTH', 2), (r'Network PID Delays', 'NETWORK_PID_DELAYS', 2)]

def buildAxisDataSourceArray():
    DATASOURCEFORMATS = []
    for axis_data_source in pncLibrary.SP_main_data_streams:
        data_name = axis_data_source['SP_format'][0]
        data_size = axis_data_source['SP_format'][1]
        axis_data = axis_data_source['data_name']
        DATASOURCEFORMATS.append((data_name, axis_data, data_size))
    return DATASOURCEFORMATS

def buildAuxiliaryDataSourceArray():
    AUXILIARYDATASOURCEFORMATS = []
    for axis_data_source in pncLibrary.SP_auxiliary_data_streams:
        # data_name = axis_data_source[-1][0]
        # axis_data = axis_data_source[1]
        # data_size = axis_data_source[-1][1]
        data_name = axis_data_source['SP_format'][0]
        data_size = axis_data_source['SP_format'][1]
        axis_data = axis_data_source['data_name']
        AUXILIARYDATASOURCEFORMATS.append((data_name, axis_data, data_size))
    return AUXILIARYDATASOURCEFORMATS

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

def formatFullFeedbackData(axis_data_source_format, auxiliary_data_source_format, axis_data_sample_format, auxiliary_data_sample_format, database_format, time_slice, data_slice, data_stream_names):
    output_data_object = pncLibrary.SculptPrintFeedbackData()

    for k in range(0, len(axis_data_source_format)):
        axis_data_stream = data_slice[0][k]
        axis_data_stream_label = axis_data_source_format[k][1]
        if axis_data_stream.size != 0:
            data_stream_length = np.shape(axis_data_stream)[0]
            output_data_array = np.empty((data_stream_length, axis_data_source_format[k][-1]))
            for label in range(0, len(axis_data_sample_format)):
                label_text = axis_data_sample_format[label]
                if label_text == 'S':
                    output_data_array[:, label] = -90 + np.zeros((data_stream_length,))
                elif label_text == 'T':
                    try:
                        output_data_array[:, label] = time_slice[0][k][:,0]
                    except Exception as r:
                        print('break')
                elif label_text not in database_format:
                    output_data_array[:, label] = np.zeros((data_stream_length))
                else:
                    output_data_array[:, label] = axis_data_stream[:,database_format.index(label_text)]
        else:
            output_data_array = np.empty((0,0))

        #Remove duplicate time values here
        if output_data_array.shape[0] > 0:
            unique_values, unique_indices = np.unique(output_data_array[:, 0], return_index=True)
            print('removing ' + str(output_data_array.shape[0] - unique_indices.shape[0]) + ' duplicate time values')
            output_data_array = output_data_array[unique_indices, :]

        setattr(output_data_object, axis_data_stream_label, output_data_array.tolist())

    for k in range(0, len(auxiliary_data_source_format)):
        auxiliary_data_stream = data_slice[0][k+len(axis_data_source_format)]
        auxiliary_data_stream_label = auxiliary_data_source_format[k][1]
        if auxiliary_data_stream.size != 0:
            data_stream_length = np.shape(auxiliary_data_stream)[0]
            output_data_array = np.empty((data_stream_length, len(auxiliary_data_source_format)))
            for label in range(0, len(auxiliary_data_sample_format)):
                label_text = auxiliary_data_sample_format[label]
                if label_text == 'T':
                    try:
                        output_data_array[:, label] = time_slice[0][k+len(axis_data_source_format)][:,0]
                    except Exception as r:
                        print('break')
                else:
                    output_data_array[:, label] = auxiliary_data_stream[:,0]
        else:
            output_data_array = np.empty((0, 0))

        setattr(output_data_object, auxiliary_data_stream_label, output_data_array.tolist())

    return output_data_object




    # for data_point_index in range(0, len(data)):
    #     try:
    #         raw_data_point = np.hstack(
    #             (data[data_point_index][k].flatten() for k in range(0, len(data[data_point_index]))))
    #     except Exception as error:
    #         print('break')
    #     output_data_point = [0.0] * len(pncLibrary.SP_data_formats[sensor_type])
    #     for label in range(0, len(pncLibrary.SP_data_formats[sensor_type])):
    #         label_text = pncLibrary.SP_data_formats[sensor_type][label]
    #         if label_text == 'S':
    #             output_data_point[label] = -90.0
    #         elif label_text not in data_format:
    #             output_data_point[label] = 0.0
    #         else:
    #             output_data_point[label] = raw_data_point[data_format.index(label_text)]
    #     SP_formatted_data.append(output_data_point)
    # return SP_formatted_data

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
    #time_start = time.clock()
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
                    if k == 0 and time_arrays[0].size == 0:
                        print('break mergesort')
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
        #time_end = time.clock()-time_start
        #print('mergesort time is: ' + str(time_end))
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

def readMachine(synchronizer, feedback_state, axis_sensor_id):
    data_format = pncLibrary.SP_pncApp_time + pncLibrary.SP_pncApp_machine_axes + pncLibrary.SP_pncApp_data_auxes[axis_sensor_id]
    if axis_sensor_id == 1:
        print('break')
    if synchronizer.mvc_run_feedback_event.is_set():
        if axis_sensor_id == 1:
            print('breakpull')
        start_time = time.clock()
        time_slice, data_slice, data_stream_names, data_stream_sizes = pncLibrary.updateInterfaceData('pull', synchronizer, feedback_state, pncLibrary.SP_main_data_streams, pncLibrary.SP_auxiliary_data_streams, [axis_sensor_id])
        #print('pull took ' + str(time.clock()-start_time))
        start_time = time.clock()
        merged_data = mergeSort(feedback_state, time_slice[0], data_slice[0], pncLibrary.getFallbackDataPoints(feedback_state.last_values_read, data_stream_names, data_stream_sizes), [], [])
        #print('MS ' + str(time.clock() - start_time))
        #FIXME this iteration here is probably not good
        return formatFeedbackData(axis_sensor_id, data_format, [[merged_data[0][k]] + merged_data[1][k] for k in range(0,len(merged_data[0]))])

def read(synchronizer, feedback_state):
    #axis_data_sample_format = pncLibrary.SP_pncApp_time + pncLibrary.SP_pncApp_machine_axes# + pncLibrary.SP_pncApp_data_auxes
    # if axis_sensor_id == 1:
    #     print('break')
    if synchronizer.mvc_run_feedback_event.is_set():
        # if axis_sensor_id == 1:
        #     print('breakpull')
        #start_time = time.clock()
        time_slice, data_slice, data_stream_names, data_stream_sizes = pncLibrary.updateFullInterfaceData('pull',
                                                                                                      synchronizer,
                                                                                                      feedback_state,
                                                                                                      pncLibrary.SP_main_data_streams,
                                                                                                      pncLibrary.SP_auxiliary_data_streams)
        return formatFullFeedbackData(feedback_state.SP_axis_data_source_format, feedback_state.SP_auxiliary_data_source_format,
                                       pncLibrary.SP_axis_data_sample_format, pncLibrary.SP_auxiliary_data_sample_format,
                                       pncLibrary.SP_pncApp_machine_axes, time_slice, data_slice, data_stream_names)
    else:
        return pncLibrary.SculptPrintFeedbackData()

def getPointRequest(synchronizer):
    if synchronizer.tp_need_points_event.is_set():
        return (True, 1)
    else:
        return (False, 0)

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

def getDrivePowerState(machine):
    return bool(machine.drive_power)

def getTPConnectionState(synchronizer):
    return synchronizer.tp_connected_event.is_set()

def getMotionStatus(synchronizer):
    return synchronizer.mc_run_motion_event.is_set()

def getCurrentlyPlanningSequenceID(machine):
    return int(machine.tp_state_current_requested_sequence_id[0])

def getCurrentlyExecutingMoveType(machine):
    try:
        move_type = pncLibrary.SP_move_type_strings[pncLibrary.tp_move_types.index(machine.currently_executing_move_type)]
    except ValueError as error:
        move_type = 'NULL'
    return move_type

def getCurrentlyExecutingSequenceID(machine):
    return int(machine.currently_executing_sequence_id[0])

def setBufferLevelSetpoint(machine, scale):
    machine.buffer_level_setpoint = scale*machine.max_buffer_level
    return True

############################# User Functions #############################

