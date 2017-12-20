######### SCULPTPRINT INTEGRATION CODE #########
import pncApp.pncApp

def start():
    #commInit()
    pncApp.appInit()
    return True

def read():
    #return feedbackData[-1]
    return [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]]

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
    return True

