import sys
dir_pncApp_project_path = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\MachineKitOpenCNC\\Interface Application\\pncApp\\'
if dir_pncApp_project_path not in sys.path:
    sys.path.append(dir_pncApp_project_path)

import pncLibrary
from multiprocessing import Event
from threading import Thread
import os, time, logging, socket, pickle, wpipe, numpy as np

#pncLibrary.updatePath()

#Globals
pncApp_connection_event = Event()
pncApp_feedback_synchronization_event = Event()
mvc_connection_type = 'socket'
mvc_command_format = 'string'
mvc_feedback_format = 'binary'


#mvc_connection_format = 'binary'

#There are two set of axis sensors: stepgens (0) and encoders (1)
# pncLibrary.SP_axis_sensor_id = pncLibrary.SP_pncLibrary.SP_axis_sensor_id
# pncLibrary.SP_machine_axes = pncLibrary.pncLibrary.SP_machine_axes#[['X','Z','S','Y','A','B','V','W'], ['X','Z','S','Y','A','B','V','W']]
# pncLibrary.SP_data_formats = pncLibrary.pncLibrary.SP_data_formats[['T','X','Z','S','Y','A','B','V','W','BL'], ['T','X','Z','S','Y','A','B','V','W']]
# pncLibrary.auxiliary_data_labels = [[r'Buffer Level', r'Buffer Control PID Delays', r'Polyline Send Times'], ['']]
# pncLibrary.auxiliary_data_names = [['HIGHRES_TC_QUEUE_LENGTH', 'BUFFER_PID_DELAYS', 'POLYLINE_SEND_TIMES'], ['']]

class PNCAppConnector(Thread):
    def __init__(self):
        super(PNCAppConnector, self).__init__()

############################# Setup Functions #############################
def monitoredMachineCount():
    return 2

def setupAuxilary():
    return setupAuxiliary()

def setupAuxiliary():
    stringArray = [r'Buffer Level']
    return stringArray

def setupMachineAuxilary(nMachine):
    return setupMachineAuxiliary(nMachine)

def setupMachineAuxiliary(nMachine):
    return pncLibrary.auxiliary_data_labels[nMachine]
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
    stringArray = [r'Initialize Control',r'Enqueue Movements',r'Execute Motion']
    return stringArray


######################## Operation ########################
def initializeComms(connection_type):
    if connection_type == 'pipe':
        try:
            print("SCULPTPRINT INTERFACE: Acquiring pipe %s..." % pncLibrary.socket_sculptprint_ipc_pipe_name)
            sculptprint_pipe = wpipe.Client(pncLibrary.socket_sculptprint_ipc_pipe_name, wpipe.Mode.Master,
                                                maxmessagesz=4096)
            pncApp_connection_event.set()
            print('SCULPTPRINT INTERFACE: Pipe %s opened successfully' % pncLibrary.socket_sculptprint_ipc_pipe_name)
            return sculptprint_pipe
        except:
            raise ConnectionRefusedError
    elif connection_type == 'socket':
        try:

            sculptprint_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sculptprint_socket.connect(('localhost', pncLibrary.socket_interface_socket_port))
            pncApp_connection_event.set()
            print('SCULPTPRINT INTERFACE: Socket connected successfully to %s' % 'localhost')
            return sculptprint_socket
        except:
            raise ConnectionRefusedError

def start():
    global mvc_connection
    try:
        mvc_connection = initializeComms(mvc_connection_type)
        if pncLibrary.MVCHandshake(mvc_connection_type, mvc_connection, 'HELLO_SCULPTPRINT_BINARY'):
            print("SUCCESS: Connected to pncApp")
            return True

    except ConnectionRefusedError:
        print("SCULPTPRINT INTERFACE: Could not connect to pncApp")
        return False
    except TimeoutError:
        print("SCULPTPRINT INTERFACE: pncApp handshake failed")
        return False

def checkAppStart(connection_type):
    pncLibrary.sendIPCData(connection_type, mvc_connection, 'CHECKSTATUS')
    return pncLibrary.receiveIPCData(connection_type, mvc_connection)

def isMonitoring():
    return safelyHandleSocketData('ISMONITORING', bool, False)
    # if not pncApp_connection_event.is_set():
    #     return False
    #
    # pncLibrary.sendIPCData(mvc_connection_type, mvc_command_format, mvc_connection, 'ISMONITORING')
    # try:
    #     received_packet = pncLibrary.receiveIPCData(mvc_connection_type, mvc_feedback_format, mvc_connection, pncLibrary.socket_wait_timeout_initial)
    #     print('ismonitoring returning ' + str(received_packet.data))
    #     return received_packet.data
    # except TimeoutError:
    #     print('ismonitoring timed out')
    #     return False

def readMachine(axis_sensor_id):
    return safelyHandleSocketData('READ_' + str(pncLibrary.SP_axis_sensor_IDs[axis_sensor_id]), list, [])

    # if not pncApp_connection_event.is_set():
    #     return []
    #
    # pncLibrary.sendIPCData(mvc_connection_type, mvc_command_format, mvc_connection, 'READ_' + str(pncLibrary.SP_axis_sensor_id))
    # try:
    #     if not pncApp_feedback_synchronization_event.is_set():
    #         timeout = pncLibrary.socket_wait_timeout_initial
    #     else:
    #         timeout = pncLibrary.socket_wait_timeout_initial
    #
    #     received_packet = pncLibrary.receiveIPCData(mvc_connection_type, mvc_feedback_format, mvc_connection, timeout)
    #     pncApp_feedback_synchronization_event.set()
    #     if type(received_packet.data) is not list:
    #         raise TypeError
    #
    #     # if received_packet.data == True:
    #     #     print('break')
    #     return received_packet.data
    # except ConnectionResetError:
    #     print('pncApp closed connection')
    #     pncApp_connection_event.clear()
    #     return []
    # except TimeoutError:
    #     print('read machine timed out')
    #     return []
    # except TypeError:
    #     print('SculptPrint socket out of sync')
    #     return []
    # except EOFError:
    #     print('socket read error')
    #     return []

def safelyHandleSocketData(message, expected_data_type, fallback_data):
    if not pncApp_connection_event.is_set():
        print('pncApp not connected')
        return fallback_data

    try:
        pncLibrary.sendIPCData(mvc_connection_type, mvc_command_format, mvc_connection, message)
        if not pncApp_feedback_synchronization_event.is_set():
            timeout = pncLibrary.socket_wait_timeout_initial
        else:
            #timeout = pncLibrary.socket_wait_timeout*5
            timeout = pncLibrary.socket_wait_timeout_initial

        received_packet = pncLibrary.receiveIPCData(mvc_connection_type, mvc_feedback_format, mvc_connection, timeout)
        pncApp_feedback_synchronization_event.set()
        if type(received_packet.data) is not expected_data_type:
            raise TypeError

        # if received_packet.data == True:
        #     print('break')
        return received_packet.data
    except ConnectionResetError:
        print('pncApp closed connection during ' + str(message))
        pncApp_connection_event.clear()
        return fallback_data
    except TimeoutError:
        print('read machine timed out on call to: ' + str(message))
        return fallback_data
    except TypeError:
        print('SculptPrint socket out of sync on call to: ' + str(message))
        return fallback_data
    except EOFError:
        print('socket read error on call to: ' + str(message))
        return fallback_data

############################# User Functions #############################
def userPythonFunction1(arg0, arg1, arg2, arg3, arg4):
    sculptprint_MVC.command_queue.put('CONNECT')

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
def closeMVCConnection(connection_type, connection):
    if connection_type == 'pipe':
        connection.close()
    elif connection_type == 'socket':
        connection.send('CLOSE_SOCKET'.encode())
        connection.shutdown(socket.SHUT_RDWR)
        connection.close()

def stop():
    if pncApp_connection_event.is_set():
        closeMVCConnection(mvc_connection_type, mvc_connection)
    return True

#initializeComms()

print(setupMachineDescriptors(0))
print(setupMachineDescriptors(1))
if __name__ != 'machinemonitor':
    start()
    #pncLibrary.sendIPCData(mvc_connection_type, mvc_command_format, mvc_connection, 'FASTFORWARD_0_1')
    time.sleep(1)

while True and __name__ != 'machinemonitor':
    z = isMonitoring()
    #zz = readMachine(1)
    if 1:
        print(z)
        yy = readMachine(0)
        print(yy)
        time.sleep(0.1)
        print('encoder data is: ')
        yy = readMachine(1)
        print(yy)

    #time.sleep(0.1)
    #stop()
    #time.sleep(2)
# z = isMonitoring()
# zz = readMachine(1)
# print(zz)
stop()
