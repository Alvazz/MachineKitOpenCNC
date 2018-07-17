import sys
dir_pncApp_project_path = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\MachineKitOpenCNC\\Interface Application\\pncApp\\'
if dir_pncApp_project_path not in sys.path:
    sys.path.append(dir_pncApp_project_path)

import pncLibrary
from multiprocessing import Event
from threading import Thread
import os, time, logging, socket, pickle, numpy as np

#pncLibrary.updatePath()

#Globals
pncApp_connector = pncLibrary.PNCAppConnection('socket', 'text', 'binary')
# pncApp_connection_event = Event()
# pncApp_feedback_synchronization_event = Event()
# mvc_connection_type = 'socket'
# mvc_command_format = 'text'
# mvc_feedback_format = 'binary'


#mvc_connection_format = 'binary'

#There are two set of axis sensors: stepgens (0) and encoders (1)
# pncLibrary.SP_axis_sensor_id = pncLibrary.SP_pncLibrary.SP_axis_sensor_id
# pncLibrary.SP_machine_axes = pncLibrary.pncLibrary.SP_machine_axes#[['X','Z','S','Y','A','B','V','W'], ['X','Z','S','Y','A','B','V','W']]
# pncLibrary.SP_data_formats = pncLibrary.pncLibrary.SP_data_formats[['T','X','Z','S','Y','A','B','V','W','BL'], ['T','X','Z','S','Y','A','B','V','W']]
# pncLibrary.auxiliary_data_labels = [[r'Buffer Level', r'Buffer Control PID Delays', r'Polyline Send Times'], ['']]
# pncLibrary.auxiliary_data_names = [['HIGHRES_TC_QUEUE_LENGTH', 'BUFFER_PID_DELAYS', 'POLYLINE_SEND_TIMES'], ['']]

# class PNCAppConnector(Thread):
#     def __init__(self):
#         super(PNCAppConnector, self).__init__()

############################# Setup Functions #############################
def monitoredMachineCount():
    return 1

def setupAuxilary():
    return setupAuxiliary()

def setupAuxiliary():
    stringArray = [r'Buffer Level']
    return stringArray

def setupMachineAuxilary(nMachine):
    return setupMachineAuxiliary(nMachine)

def setupMachineAuxiliary(nMachine):
    return pncLibrary.SP_auxiliary_data_labels[nMachine]
    # if nMachine == 0:
    #     stringArray = [r'Buffer Level', r'Buffer Control PID Delays', r'Polyline Send Times']
    # else:
    #     stringArray = ['']
    # return stringArray

def setupMachineDescriptors(nMachine):
    return [0] + [1 for k in range(0,len(pncLibrary.SP_CAM_machine_axes[nMachine]))] + [2 for k in range(0,len(pncLibrary.SP_auxiliary_data_labels[nMachine])) if pncLibrary.SP_auxiliary_data_labels[nMachine][k] != '']
    # if nMachine == 0:
    #     #Monitoring from BBB gives stepgen position and buffer level
    #     columnTypeArray = [0, 1, 1, 1, 1, 1, 1, 1, 1, 2]
    #     [0] + [1 for k in range(0,len(pncLibrary.SP_machine_axes[nMachine]))] + [2 for k in range(0,len(pncLibrary.auxiliary_data_labels[nMachine]))]
    # else:
    #     #Encoder positions are only time and axis position
    #     columnTypeArray = [0, 1, 1, 1, 1, 1, 1, 1, 1]
    # return columnTypeArray

def setupUserDataNames():
    stringArray = [r'Start File', r'End File', r'my data 3', r'my data 4', r'my data 5']
    return stringArray

# Returns an array of user defined function names that are displayed on the function buttons in the feature UI. The array affects
# feature UI functionality only.  The array must be sized on the interval [1,3]. Defining this method is optional.
def setupUserFunctionNames():
    stringArray = [r'Load and Enqueue Voxelized Points',r'Load and Enqueue Trapezoid Points',r'Execute Motion']
    return stringArray

def start():
    #global mvc_connection
    try:
        pncApp_connector.connection = pncLibrary.initializeInterfaceIPC(pncApp_connector)
        if pncLibrary.MVCHandshake(pncApp_connector, 'HELLO_SCULPTPRINT_BINARY'):
            print("SUCCESS: Connected to pncApp")
            return True

    except ConnectionRefusedError:
        print("SCULPTPRINT INTERFACE: Could not connect to pncApp")
        return False
    except TimeoutError:
        print("SCULPTPRINT INTERFACE: pncApp handshake failed")
        return False

# def checkAppStart(connection_type):
#     pncLibrary.sendIPCData(connection_type, mvc_connection, 'CHECKSTATUS')
#     return pncLibrary.receiveIPCData(connection_type, mvc_connection)

def isMonitoring():
    return pncLibrary.safelyHandleSocketData(pncApp_connector, 'ISMONITORING', bool, False)

def readMachine(axis_sensor_id):
    return pncLibrary.safelyHandleSocketData(pncApp_connector, 'READ_' + str(pncLibrary.SP_axis_sensor_IDs[axis_sensor_id]), list, [])

############################# User Functions #############################
# def userPythonFunction1(arg0, arg1, arg2, arg3, arg4):
#     if arg1 == 0:
#         arg1 = 10
#     return 'LOAD' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'LOAD_' + str(int(arg0)) + '_' + str(int(arg1)), str, '')
    #return 'CONNECT' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'CONNECT', str, '')

def userPythonFunction1(arg0, arg1, arg2, arg3, arg4):
    if arg1 == 0:
        arg1 = 10
    return 'VOXELENQUEUE' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'VOXELENQUEUE_' + str(int(arg0)) + '_' + str(int(arg1)), str, '')

def userPythonFunction2(arg0, arg1, arg2, arg3, arg4):
    #print('execute userPythonFunction2(' + str(arg0) + ',' + str(arg1) + ',' + str(arg2) + ',' + str(arg3) + ',' + str(arg4) + ')\n')
    return 'TRAPENQUEUE' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'TRAPENQUEUE_' + str(int(arg0)), str, '')
    # if 'ENQUQUE' in ack:
    #     return True
    # else:
    #     return False
    #sculptprint_MVC.command_queue.put('ENQUEUE ' + str(arg0) + ' ' + str(arg1))
    #return True;

def userPythonFunction3(arg0, arg1, arg2, arg3, arg4):
    return 'EXECUTE' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'EXECUTE', str, '')
    # print('execute userPythonFunction3(' + str(arg0) + ',' + str(arg1) + ',' + str(arg2) + ',' + str(arg3) + ',' + str(arg4) + ')\n')
    # sculptprint_MVC.command_queue.put('EXECUTE')
    # return True;

# def connectToMachine():
#     sculptprint_MVC.command_queue.put('CONNECT')

# Called to stop monitoring the machine.
# Will execute when the stop button is pressed in the Monitor Machine feature.


def stop():
    if pncApp_connector.app_connection_event.is_set():
        pncLibrary.closeMVCConnection(pncApp_connector)
    return True

#initializeInterfaceIPC()

#print(setupMachineDescriptors(0))
#print(setupMachineDescriptors(1))
if __name__ != 'machinemonitor':
    start()
    #pncLibrary.sendIPCData(mvc_connection_type, mvc_command_format, mvc_connection, 'FASTFORWARD_0_1')
    time.sleep(1)

if __name__ != 'machinemonitor':
    # userPythonFunction1(0, 0, 0, 0, 0)
    # time.sleep(1)
    # userPythonFunction3(0, 0, 0, 0, 0)

    z = isMonitoring()
    print(z)
    yy = readMachine(0)
    #print(yy[0])
    # time.sleep(0.1)
    print('encoder data is: ')
    yy = readMachine(1)
    #print(yy[0])
    userPythonFunction1(0, 0, 0, 0, 0)
    time.sleep(1)
    userPythonFunction3(0,0,0,0,0)

    #zz = readMachine(1)
    while 1:
        z = isMonitoring()
        print(z)
        yy = readMachine(0)
        try:
            print(yy[-1])
        except:
            pass
        #time.sleep(0.1)
        print('encoder data is: ')
        yy = readMachine(1)
        try:
            print(yy[-1])
        except:
            pass
            #print("print break")

        #userPythonFunction2(5, 7, 0, 0, 0)
        #userPythonFunction3(0,0,0,0,0)
        #stop()
        #break

    #time.sleep(0.1)
    #stop()
    #time.sleep(2)
# z = isMonitoring()
# zz = readMachine(1)
# print(zz)
#stop()
