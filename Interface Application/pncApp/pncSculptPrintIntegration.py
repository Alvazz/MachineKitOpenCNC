######### SCULPTPRINT INTEGRATION CODE #########
#import pnc
import pncApp, time
from pnc.pncApp import *
import numpy as np

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

def formatFeedbackDataForSP(time, positions, aux):
    #Expects a feedback vector in XYZAB format
    SP_formatted_data = [0]*len(SP_data_format)
    for label in range(0,len(SP_data_format)):
        label_text = SP_data_format[label]
        if label_text == 'X':
            SP_formatted_data[label] = positions[0]
        if label_text == 'Y':
            SP_formatted_data[label] = positions[1]
        if label_text == 'Z':
            SP_formatted_data[label] = positions[2]
        if label_text == 'A':
            SP_formatted_data[label] = positions[3]
        if label_text == 'B':
            SP_formatted_data[label] = positions[4]
        if label_text == 'T':
            SP_formatted_data[label] = time
        if label_text == 'BL':
            SP_formatted_data[label] = aux
        if label_text == 'S':
            SP_formatted_data[label] = -90.0
        else:
            SP_formatted_data[label] = 0.0

def readMachine(axis_sensor_id):
    global feedback_listener, machine_controller, encoder_interface, data_store
    global data_sample_time, highfreq_rx_time, lowfreq_rx_time
    if axis_sensor_id == 0:
        BBB_feedback_array = []
        machine_feedback_vector = np.array([])
        ## FIXME check that logging is enabled and that threads are set up right
        #machine_feedback_array = np.array(())
        #Return stepgen data
        #for sample in range(machine_feedback_record_id,machine_controller.data_store.stepgen_feedback_positions.shape[0]-1):
        # dt = np.min((highfreq_ethernet_time-dt,lowfreq_ethernet_time-dt))
        #Integrate the time value forward until we have reached the current time in the data_store. First pick the "integration variable" as the lower of either the lowfreq or highfreq received times
        #while highfreq_rx_time < machine_controller.data_store.highfreq_ethernet_received_times[-1] or lowfreq_rx_time < machine_controller.data_store.lowfreq_ethernet_received_times[-1]:
        highfreq_inquiry_time = machine_controller.data_store.highfreq_ethernet_received_times[-1]
        lowfreq_inquiry_time = machine_controller.data_store.lowfreq_ethernet_received_times[-1]
        while data_sample_time < highfreq_inquiry_time or data_sample_time < lowfreq_inquiry_time:
            #First find the nearest POSITIVE time delta
            #dt_HF = np.abs(machine_controller.data_store.highfreq_ethernet_received_times-highfreq_ethernet_time)
            print('high frequency rx time is ' + str(highfreq_rx_time))
            print('low frequency rx time is ' + str(lowfreq_rx_time))

            dt_HF = machine_controller.data_store.highfreq_ethernet_received_times - data_sample_time
            # dt_LF = np.abs(machine_controller.data_store.lowfreq_ethernet_received_times-lowfreq_ethernet_time)
            dt_LF = machine_controller.data_store.lowfreq_ethernet_received_times - data_sample_time
            ## FIXME hadle case where dt_HF or dt_LF has a zero
            #Discard values from the past
            dt_HF = dt_HF[np.where(dt_HF > 0)]
            dt_LF = dt_LF[np.where(dt_LF > 0)]
            #if len(dt_HF) == 0:

                

            #HF_sample_ndx = dt_HF.argmin()+len(dt_HF[np.where(dt_HF<0)])
            #LF_sample_ndx = dt_LF.argmin()+len(dt_LF[np.where(dt_LF<0)])
            if len(dt_HF) == 0 and len(dt_LF) != 0:
                # No data from RSH
                LF_sample_ndx = dt_LF.argmin() + len(dt_LF[np.where(dt_LF < 0)])
                HF_sample_ndx = -1
                #Hack hack
                dt_HF = np.array([np.inf])
            elif len(dt_LF) == 0 and len(dt_HF) != 0:
                # No data from RTAPI
                HF_sample_ndx = dt_HF.argmin() + len(dt_HF[np.where(dt_HF < 0)])
                LF_sample_ndx = -1
                dt_LF = np.array([np.inf])
            elif len(dt_HF) == 0 and len(dt_LF) == 0:
                # No data at all
                print('no data to return')
                return []
            else:
                # We have data for both low and high frequency channels
                print('seems to be good data on both channels')
                HF_sample_ndx = dt_HF.argmin() + len(dt_HF[np.where(dt_HF < 0)])
                LF_sample_ndx = dt_LF.argmin() + len(dt_LF[np.where(dt_LF < 0)])

            if dt_HF.min() < dt_LF.min():
                #Next future record comes from RSH
                print('next future record comes from RSH')
                #Integration variable is closer to high frequency time vector value from RSH
                #if LF_sample_ndx != 0:
                #Use current HF time feedback as integration variable, and use last stepgen feedback position

                #machine_feedback_vector = machine_controller.data_store.highfreq_ethernet_received_times[HF_sample_ndx]
                #machine_feedback_vector = np.hstack((machine_feedback_vector,machine_controller.data_store.stepgen_feedback_positions[LF_sample_ndx-1]))
                #machine_feedback_vector = np.hstack((machine_feedback_vector,machine_controller.data_store.highres_tc_queue_length[HF_sample_ndx]))
                machine_feedback_time = machine_controller.data_store.highfreq_ethernet_received_times[HF_sample_ndx]
                machine_feedback_aux = machine_controller.data_store.highres_tc_queue_length[HF_sample_ndx]
                if LF_sample_ndx > 0:
                    machine_feedback_positions = machine_controller.data_store.stepgen_feedback_positions[LF_sample_ndx - 1]
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

                # Advance integration time variable
                highfreq_rx_time = machine_controller.data_store.highfreq_ethernet_received_times[HF_sample_ndx]
                data_sample_time = highfreq_rx_time

            elif dt_HF.min() > dt_LF.min():
                #Next future record comes from RTAPI
                print('next future record comes from RTAPI')
                #if HF_sample_ndx != 0:
                #Use current LF time feedback as integration variable, and use current stepgen feedback position
                machine_feedback_time = machine_controller.data_store.lowfreq_ethernet_received_times[LF_sample_ndx]
                machine_feedback_positions = machine_controller.data_store.stepgen_feedback_positions[LF_sample_ndx]
                if HF_sample_ndx > 0:
                    machine_feedback_aux = machine_controller.data_store.highres_tc_queue_length[HF_sample_ndx-1]
                else:
                    print('no buffer level feedback')
                    machine_feedback_aux = 0

                    #machine_feedback_vector = machine_controller.data_store.highfreq_ethernet_received_times[HF_sample_ndx-1]
                    #machine_feedback_vector = np.hstack((machine_feedback_vector,machine_controller.data_store.stepgen_feedback_positions[LF_sample_ndx]))
                    #machine_feedback_vector = np.hstack((machine_feedback_vector, machine_controller.data_store.highres_tc_queue_length[HF_sample_ndx-1]))
                    #else:
                    # We have a problem. Can't return complete data set, so wait

                    #machine_feedback_time = machine_controller.data_store.highfreq_ethernet_received_times[LF_sample_ndx]
                    #machine_feedback_positions = machine_controller.machine.current_position
                    #machine_feedback_aux = machine_controller.data_store.highres_tc_queue_length[HF_sample_ndx]
                    #pass
                # Advance integration time variable
                print('the time sample index is ' + str(LF_sample_ndx))
                lowfreq_rx_time = machine_controller.data_store.lowfreq_ethernet_received_times[LF_sample_ndx]
                print('setting data_sample_time to ' + str(lowfreq_rx_time))
                print('future times are ')
                print(dt_LF)
                data_sample_time = lowfreq_rx_time
            else:
                #Times overlap, this one is simple, but will probably never happen
                print('times overlap')

                machine_feedback_time = machine_controller.data_store.highfreq_ethernet_received_times[HF_sample_ndx]
                machine_feedback_positions = machine_controller.data_store.stepgen_feedback_positions[LF_sample_ndx]
                machine_feedback_aux = machine_controller.data_store.highres_tc_queue_length[HF_sample_ndx]

                # machine_feedback_vector = machine_controller.data_store.highfreq_ethernet_received_times[HF_sample_ndx]
                # machine_feedback_vector = np.hstack((machine_feedback_vector, machine_controller.data_store.stepgen_feedback_positions[LF_sample_ndx]))

                lowfreq_rx_time = machine_controller.data_store.highfreq_ethernet_received_times[LF_sample_ndx]
                highfreq_rx_time = machine_controller.data_store.highfreq_ethernet_received_times[HF_sample_ndx]


                #data_sample_time = lowfreq_rx_time+minimum_integration_dt
                #data_sample_time += minimum_integration_dt
                print('the integration variable is now ' + str(data_sample_time))
                if lowfreq_rx_time != highfreq_rx_time:
                    print('major problem')

            # Advance data_sample_time by a tiny bit
            data_sample_time += minimum_integration_dt

            #machine_feedback_list = [value for element in machine_feedback_vector.tolist() for value in element]
            machine_feedback_list = formatFeedbackDataForSP(machine_feedback_time, machine_feedback_positions, machine_feedback_aux)

            #Now build the array
            BBB_feedback_array.append(machine_feedback_list)
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
        return BBB_feedback_array


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

