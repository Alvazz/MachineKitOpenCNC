######### SCULPTPRINT INTEGRATION CODE #########
#import pnc
import pncApp, time
from pnc.pncApp import *
import numpy as np
import copy

machine_feedback_record_id = 0
encoder_feedback_record_id = 0

highfreq_rx_time = 0
lowfreq_rx_time = 0
data_sample_time = 0
minimum_integration_dt = 1e-6

RTAPI_rx_integration_time = 0
RSH_rx_integration_time = 0


#There are two set of axis sensors: stepgens (0) and encoders (1)
axis_sensor_id = [0, 1]

SP_data_format = ['T','X','Z','S','Y','A','B','V','W','BL']

#from pncApp import machine_controller, data_store, feedback_listener
global feedback_listener, machine_controller, encoder_interface, data_store

######################## Setup ########################
def monitoredMachineCount():
    return 2

def setupMachineAuxilary():
    return setupAuxiliary()

def setupAuxiliary():
    #stringArray = [r'C', r'sinx', r'fx', r'gx', r'fx+gx', r'cos+cos', r'cos*sin', r'stepf']
    stringArray = [r'Buffer Level']
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
    stringArray = [r'Deez Nut 1', r'my data 2', r'my data 3', r'my data 4', r'my data 5']
    return stringArray

# Returns an array of user defined function names that are displayed on the function buttons in the feature UI. The array affects
# feature UI functionality only.  The array must be sized on the interval [1,3]. Defining this method is optional.
def setupUserFunctionNames():
    stringArray = [r'Deez Nut''s',r'my function 2',r'my function 3']
    return stringArray

######################## Operation ########################
def start():
    #global machine_controller
    global feedback_listener, machine_controller, encoder_interface, data_store
    #commInit()
    print('initializing')
    #pnc.pncApp.appInit()
    feedback_listener, machine_controller, encoder_interface, data_store = pncApp.appInit()
	
    #Log machine feedback start point
    print('preparing to busy wait')
    while machine_controller == []:
        print('busy waiting')
        pass

    #Log feedback start point
    machine_controller.machine.machine_feedback_written_record_id = machine_controller.data_store.machine_feedback_num_records
    machine_controller.machine.encoder_feedback_written_record_id = machine_controller.data_store.encoder_feedback_num_records

    #Configure SP data format in machine model
    machine_controller.machine.SP_data_format = SP_data_format

    time.sleep(1)
    print('logging in')
    machine_controller.login()
    machine_controller.setLogging(1)

    # We are setup -- start commanding points using the motion controller
    #motion_controller.start()
    #time.sleep(3)
    #machine_controller.motion_controller.testMachine()

    return True

def read():
    global feedback_listener, machine_controller, encoder_interface, data_store
    #If appInit was successful
    if not machine_controller == []:
        #print('machine controller is ' + machine_controller)
        #return feedbackData[-1]
        #return [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]]

        #Format data from data store in format requested by SP
        #fb_data = [machine_controller.data_store.stepgen_feedback_positions[-1][np.array([0,2])].tolist(),[-90.], machine_controller.data_store.stepgen_feedback_positions[-1][np.array([1,3,4])].tolist(),[0.,0.]]
        fb_data = [machine_controller.data_store.encoder_feedback_positions[-1][np.array([0, 2])].tolist(), [-90.],
                   machine_controller.data_store.encoder_feedback_positions[-1][np.array([1, 3, 4])].tolist(), [0., 0.],
                   [float(machine_controller.data_store.highres_tc_queue_length[-1])]]
        print(machine_controller.data_store.encoder_feedback_num_records)
        print([[item for sublist in fb_data for item in sublist]])
        print(machine_controller.data_store.encoder_feedback_positions[-1])
        print('encoder feedback id is ' + str(machine_controller.machine.encoder_feedback_written_record_id))

        #Return only new data and don't skip any due to thread sync
        ##FIXME need mutex?
        #if machine_controller.data_store.machine_feedback_num_records - machine_feedback_record_id:
        current_record = machine_controller.data_store.encoder_feedback_num_records
        if current_record - machine_controller.machine.encoder_feedback_written_record_id:
            print('returning values')
            machine_controller.machine.encoder_feedback_written_record_id = current_record
            return [[data for sublist in fb_data for data in sublist]]
        else:
            print('returning no values')
            print(str(machine_controller.data_store.encoder_feedback_num_records))
            return []
    return []

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
            SP_formatted_data[label] = float(time)
        elif label_text == 'BL':
            SP_formatted_data[label] = aux
        elif label_text == 'S':
            SP_formatted_data[label] = -90.0
        else:
            SP_formatted_data[label] = 0.0
    return SP_formatted_data

def readMachine(axis_sensor_id):
    global feedback_listener, machine_controller, encoder_interface, data_store
    global data_sample_time, highfreq_rx_time, lowfreq_rx_time

    ## FIXME should be incorporated below
    if not machine_controller.machine.logging_mode:
        print('logging not enabled, not returning data')
        return []

    if axis_sensor_id == 0:
        BBB_feedback_lists = []
        #machine_feedback_vector = np.array([])
        LF_sample_time = 0
        HF_sample_time = 0

        data_output_flag = 0

        ## FIXME check that logging is enabled and that threads are set up right
        #machine_feedback_array = np.array(())
        #Return stepgen data
        #for sample in range(machine_feedback_record_id,machine_controller.data_store.stepgen_feedback_positions.shape[0]-1):
        # dt = np.min((highfreq_ethernet_time-dt,lowfreq_ethernet_time-dt))
        #Integrate the time value forward until we have reached the current time in the data_store. First pick the "integration variable" as the lower of either the lowfreq or highfreq received times
        #while highfreq_rx_time < machine_controller.data_store.highfreq_ethernet_received_times[-1] or lowfreq_rx_time < machine_controller.data_store.lowfreq_ethernet_received_times[-1]:

        #Snapshot data store
        data_store_snapshot = copy.deepcopy(machine_controller.data_store)

        #Copy data set to local variable for thread sync
        #local_lowfreq_ethernet_times = machine_controller.data_store.lowfreq_ethernet_received_times
        #local_highfreq_ethernet_times = machine_controller.data_store.highfreq_ethernet_received_times

        #Handle empty data store time vector
        if len(data_store_snapshot.highfreq_ethernet_received_times) != 0:
            highfreq_inquiry_time = float(data_store_snapshot.highfreq_ethernet_received_times[-1])
            #print('setting highfreq inquiry time to ' + str(highfreq_inquiry_time))
        else:
            highfreq_inquiry_time = 0

        if len(data_store_snapshot.lowfreq_ethernet_received_times) != 0:
            lowfreq_inquiry_time = float(data_store_snapshot.lowfreq_ethernet_received_times[-1])
            #print('setting lowfreq inquiry time to ' + str(lowfreq_inquiry_time))
        else:
            lowfreq_inquiry_time = 0

        ## FIXME only process a slice of the FB data, not the entire thing!
        #Pull relevant section of data from data_store_snapshot
        LF_start_time_index = findNextClosestTimeIndex(data_sample_time, data_store_snapshot.lowfreq_ethernet_received_times)
        HF_start_time_index = findNextClosestTimeIndex(data_sample_time, data_store_snapshot.highfreq_ethernet_received_times)
        #Serial_time_index = LF_time_index = findNextClosestTimeIndex(data_sample_time,data_store_snapshot.serial_received_times)
        LF_ethernet_time_slice = data_store_snapshot.lowfreq_ethernet_received_times[LF_start_time_index:]
        LF_ethernet_data_slice = data_store_snapshot.stepgen_feedback_positions[LF_start_time_index:]
        HF_ethernet_time_slice = data_store_snapshot.highfreq_ethernet_received_times[HF_start_time_index:]
        HF_ethernet_data_slice = data_store_snapshot.highres_tc_queue_length[HF_start_time_index:]

        if len(LF_ethernet_time_slice) > 0:
            LF_data_end_time = float(LF_ethernet_time_slice[-1])
        else:
            LF_data_end_time = 0

        if len(HF_ethernet_time_slice) > 0:
            HF_data_end_time = float(HF_ethernet_time_slice[-1])
        else:
            HF_data_end_time = 0

        #while data_sample_time < highfreq_inquiry_time or data_sample_time < lowfreq_inquiry_time:
        itct = 0
        while data_sample_time < LF_data_end_time or data_sample_time < HF_data_end_time:
            itct += 1
            #print('LF_data_end_time is ' + str(LF_data_end_time) + ' and HF is ' + str(HF_data_end_time))
            data_output_flag = 0
            #First find the nearest POSITIVE time delta
            #dt_HF = np.abs(machine_controller.data_store.highfreq_ethernet_received_times-highfreq_ethernet_time)
            #print('high frequency rx time is ' + str(highfreq_rx_time))
            #print('low frequency rx time is ' + str(lowfreq_rx_time))

            #Find next time index for each data channel
            LF_time_index = findNextClosestTimeIndex(data_sample_time,
                                                     LF_ethernet_time_slice)
            HF_time_index = findNextClosestTimeIndex(data_sample_time,
                                                     HF_ethernet_time_slice)

            if LF_time_index == len(LF_ethernet_time_slice-1):
                print('bad')

            ## FIXME hadle case where dt_HF or dt_LF has a zero
            if LF_time_index == -1 and HF_time_index == -1:
                #Both arrays are empty, return nothing
                machine_feedback_time = 0
                machine_feedback_positions = machine_controller.machine.current_position
                machine_feedback_aux = 0
                break
            elif LF_time_index == -1 and HF_time_index != -1:
                #No data on RTAPI channel
                #LF_time_index = np.array([np.inf])
                HF_sample_time = HF_ethernet_time_slice.item(HF_time_index)
                LF_sample_time = np.array([np.inf])
            elif LF_time_index != -1 and HF_time_index == -1:
                #No data on RSH channel
                #HF_time_index = np.array([np.inf])
                HF_sample_time = np.array([np.inf])
                LF_sample_time = LF_ethernet_time_slice.item(LF_time_index)
            else:
                #Good data on both channels
                HF_sample_time = HF_ethernet_time_slice.item(HF_time_index)
                LF_sample_time = LF_ethernet_time_slice.item(LF_time_index)
                #pass

            #
            #
            # dt_HF = machine_controller.data_store.highfreq_ethernet_received_times - data_sample_time
            # # dt_LF = np.abs(machine_controller.data_store.lowfreq_ethernet_received_times-lowfreq_ethernet_time)
            # dt_LF = machine_controller.data_store.lowfreq_ethernet_received_times - data_sample_time
            #
            #
            # #Discard values from the past
            # future_dt_HF = dt_HF[np.where(dt_HF > 0)]
            # future_dt_LF = dt_LF[np.where(dt_LF > 0)]
            #
            # #if len(dt_HF) == 0:
            #
            # #HF_sample_ndx = dt_HF.argmin()+len(dt_HF[np.where(dt_HF<0)])
            # #LF_sample_ndx = dt_LF.argmin()+len(dt_LF[np.where(dt_LF<0)])
            # if len(future_dt_HF) == 0 and len(future_dt_LF) != 0:
            #     # No data from RSH
            #     LF_sample_ndx = future_dt_LF.argmin() + len(dt_LF[np.where(dt_LF < 0)])
            #     HF_sample_ndx = -1
            #     #Hack hack
            #     future_dt_HF = np.array([np.inf])
            # elif len(future_dt_LF) == 0 and len(future_dt_HF) != 0:
            #     # No data from RTAPI
            #     HF_sample_ndx = future_dt_HF.argmin() + len(dt_HF[np.where(dt_HF < 0)])
            #     LF_sample_ndx = -1
            #     future_dt_LF = np.array([np.inf])
            # elif len(future_dt_HF) == 0 and len(future_dt_LF) == 0:
            #     # No data at all
            #     print('no data to return')
            #     return []
            # else:
            #     # We have data for both low and high frequency channels
            #     print('seems to be good data on both channels')
            #     HF_sample_ndx = future_dt_HF.argmin() + len(dt_HF[np.where(dt_HF < 0)])
            #     LF_sample_ndx = future_dt_LF.argmin() + len(dt_LF[np.where(dt_LF < 0)])

            #if HF_sample_time < LF_sample_time:
            if (HF_sample_time-data_sample_time) < (LF_sample_time-data_sample_time):
                #Next future record comes from RSH, integration variable is closer to high frequency time vector value from RSH
                print('next future record comes from RSH')
                # Advance integration time variable
                data_sample_time = HF_sample_time

                machine_feedback_time = HF_ethernet_time_slice[HF_time_index]
                machine_feedback_aux = HF_ethernet_data_slice[HF_time_index]

                #Handle case where only RSH is logging and we have no data from RTAPI
                if LF_time_index > 0:
                    #Use the last stepgen feedback position
                    machine_feedback_positions = machine_controller.data_store.stepgen_feedback_positions[LF_time_index - 1]
                else:
                    #Signal from above that no RTAPI data has been accumulated
                    #else:
                    print('no stepgen data')
                    ## FIXME this should use last element from data_store stepgen feedback? Maybe that is the same....
                    machine_feedback_positions = machine_controller.machine.current_position
                #else:
                # We have a problem. Can't return complete data set, so wait

                #There is no stepgen feedback data to report, so write out RSH buffer level only
                # machine_feedback_time = machine_controller.data_store.highfreq_ethernet_received_times[HF_sample_ndx]
                # machine_feedback_positions = machine_controller.machine.current_position
                # machine_feedback_aux = machine_controller.data_store.highres_tc_queue_length[HF_sample_ndx]


                #highfreq_rx_time = machine_controller.data_store.highfreq_ethernet_received_times[HF_sample_ndx]
                #data_sample_time = highfreq_rx_time
                data_output_flag = 1

            elif (LF_sample_time-data_sample_time) < (HF_sample_time-data_sample_time):
                #Next future record comes from RTAPI, integration variable is closer to low frequency time vector value from RTAPI
                #print('next future record comes from RTAPI')
                # Advance integration time variable
                data_sample_time = LF_sample_time

                #machine_feedback_time = machine_controller.data_store.lowfreq_ethernet_received_times[LF_time_index]
                machine_feedback_time = LF_ethernet_time_slice[LF_time_index]
                #machine_feedback_positions = machine_controller.data_store.stepgen_feedback_positions[LF_time_index]
                machine_feedback_positions = LF_ethernet_data_slice[LF_time_index]
                if HF_time_index > 0:
                    #machine_feedback_aux = machine_controller.data_store.highres_tc_queue_length[HF_time_index - 1]
                    machine_feedback_aux = HF_ethernet_data_slice[HF_time_index - 1]
                else:
                    #print('no buffer level feedback')
                    machine_feedback_aux = 0

                data_output_flag = 1

            else:
                #Times overlap, this one is simple, but will probably never happen
                print('times overlap')
                #Set integration time variable to either LF or HF, shouldn't matter
                data_sample_time = LF_sample_time

                ## FIXME use data slice instead
                machine_feedback_time = machine_controller.data_store.highfreq_ethernet_received_times[HF_time_index]
                machine_feedback_positions = machine_controller.data_store.stepgen_feedback_positions[LF_time_index]
                machine_feedback_aux = machine_controller.data_store.highres_tc_queue_length[HF_time_index]

                lowfreq_rx_time = machine_controller.data_store.highfreq_ethernet_received_times[LF_time_index]
                highfreq_rx_time = machine_controller.data_store.highfreq_ethernet_received_times[HF_time_index]

                if lowfreq_rx_time != highfreq_rx_time:
                    print('major problem')

                data_output_flag = 1

            #print('the integration variable is now ' + str(data_sample_time))
            # Advance data_sample_time by a tiny bit, hack hack
            data_sample_time = np.nextafter(data_sample_time,np.inf)

            #machine_feedback_list = [value for element in machine_feedback_vector.tolist() for value in element]
            #FIXME check that we are actually writing out good data
            if data_output_flag:
                # Now build the array
                machine_feedback_list = formatFeedbackDataForSP(machine_feedback_time, machine_feedback_positions, machine_feedback_aux)
                BBB_feedback_lists.append(machine_feedback_list)

            #print('data sample time is ' + str(data_sample_time) + ' and lowfreq inquiry time is ' + str(lowfreq_inquiry_time))
            # next_dt = np.min(np.abs(machine_controller.data_store.highfreq_ethernet_received_times-highfreq_ethernet_time).argmin()
            #
            # highfreq_rx_time = machine_controller.data_store.highfreq_ethernet_received_times[sample]
            # lowfreq_rx_time = machine_controller.data_store.highfreq_ethernet_received_times[sample]
            #
            # # Loop through rows of stepgen_feedback_positions
            # buffer_Level = machine_controller.data_store.highres_tc_queue_length[sample]
            # stepgen_position = np.hstack([machine_controller.data_store.stepgen_feedback_positions[sample][np.array([0, 2])],-90.,
            #                                machine_controller.data_store.encoder_feedback_positions[sample][np.array([1, 3, 4])],0.,0.])
            # ## FIXME use machine time here
            # stepgen_position = np.hstack((machine_controller.data_store.lowfreq_ethernet_received_times[sample], stepgen_position))
            # #Add auxiliary data
            # ## FIXME this is asynchronous......
            #
            # [float(machine_controller.data_store.highres_tc_queue_length[-1])]]
            #print('the feedback array is ')
            #print(BBB_feedback_array)
        print('done reading data with itct ' + str(itct))
        ## FIXME return empty array if no data?
        return BBB_feedback_lists


# Returns true if monitoring is currently happening.
def isMonitoring():
    #global feed_thread
    #return feedback.is_alive()
    #return True
    # FIXME do something with this
    return machine_controller.is_alive() & machine_controller.machine.logging_mode


def userPythonFunction1(arg0, arg1, arg2, arg3, arg4):
    print('execute Deez Nut''s(' + str(arg0) + ',' + str(arg1) + ',' + str(arg2) + ',' + str(arg3) + ',' + str(arg4) + ')\n')
    return True;

def userPythonFunction2(arg0, arg1, arg2, arg3, arg4):
    print('execute userPythonFunction2(' + str(arg0) + ',' + str(arg1) + ',' + str(arg2) + ',' + str(arg3) + ',' + str(arg4) + ')\n')
    return True;

def userPythonFunction3(arg0, arg1, arg2, arg3, arg4):
    print('execute userPythonFunction3(' + str(arg0) + ',' + str(arg1) + ',' + str(arg2) + ',' + str(arg3) + ',' + str(arg4) + ')\n')
    return True;

# Called to stop monitoring the machine.
# Will execute when the stop button is pressed in the Monitor Machine feature.
def stop():
    #global feed_thread
    #if feedback.is_alive():
    #    print('Feed thread is still alive')
    #else:
    #    print('Feed thread is not alive')
    #feedback.deactivate()
    #feedback.join()
    #feedback.close()
    #print('Buffer file was closed.\n')
	#pnc.pncApp.appClose()
	print('closing')
	pncApp.appClose()
	return True

start()
time.sleep(0.5)
while True:
    readMachine(0)