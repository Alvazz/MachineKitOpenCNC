######### SCULPTPRINT INTEGRATION CODE #########
#import pnc
import pncApp, time
from pnc.pncApp import *
import numpy as np

machine_feedback_record_id = 0
encoder_feedback_record_id = 0

#from pncApp import machine_controller, data_store, feedback_listener
global feedback_listener, machine_controller, encoder_interface, data_store

def start():
    #global machine_controller
    global feedback_listener, machine_controller, encoder_interface, data_store
    #commInit()
    print('initializing')
    #pnc.pncApp.appInit()
    feedback_listener, machine_controller, motion_controller, encoder_interface, data_store = pncApp.appInit()
	
    #Log machine feedback start point
    while machine_controller == []:
        print('busy waiting')
        pass

    #Log feedback start point
    machine_controller.machine.machine_feedback_written_record_id = machine_controller.data_store.machine_feedback_num_records
    machine_controller.machine.encoder_feedback_written_record_id = machine_controller.data_store.encoder_feedback_num_records
    time.sleep(1)
    machine_controller.login()
    # We are setup -- start commanding points using the motion controller
    motion_controller.start()

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

# Returns true if monitoring is currently happening.
def isMonitoring():
    #global feed_thread
    #return feedback.is_alive()
    return True

def setupAuxilary():
    return setupAuxiliary()

def setupAuxiliary():
    #stringArray = [r'C', r'sinx', r'fx', r'gx', r'fx+gx', r'cos+cos', r'cos*sin', r'stepf']
    stringArray = [r'Buffer Level']
    return stringArray

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

