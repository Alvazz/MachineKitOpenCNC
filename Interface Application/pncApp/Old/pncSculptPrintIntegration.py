######### SCULPTPRINT INTEGRATION CODE #########
import pnc
from pnc.pncApp import *
import numpy as np

def start():
    #commInit()
    print('initializing')
    pnc.pncApp.appInit()
    return True

def read():
	global data_source, machine_controller
	print(machine_controller)
	#return feedbackData[-1]
    #return [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]]
	
	#	print([machine_controller.data_store.stepgen_feedback_positions[-1]])
	#print([machine_controller.data_store.stepgen_feedback_positions[-1][np.array([0,2])].tolist(),[-90], machine_controller.data_store.stepgen_feedback_positions[-1][np.array([1,3,4])].tolist(),[0,0]])
	fb_data = [machine_controller.data_store.stepgen_feedback_positions[-1][np.array([0,2])].tolist(),[-90.], machine_controller.data_store.stepgen_feedback_positions[-1][np.array([1,3,4])].tolist(),[0.,0.]]
	#print([[item for sublist in fb_data for item in sublist]])
	return [[data for sublist in fb_data for data in sublist]]
	#return [machine_controller.data_store.stepgen_feedback_positions[-1][np.array([0,2])],-90, machine_controller.data_store.stepgen_feedback_positions[-1][np.array([1,3,4])],0,0]
	#return [machine_controller.data_store.stepgen_feedback_positions[-1]]

# Returns true if monitoring is currently happening.
def isMonitoring():
    #global feed_thread
    #return feedback.is_alive()
    return True

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
	pnc.pncApp.appClose()
	return True

