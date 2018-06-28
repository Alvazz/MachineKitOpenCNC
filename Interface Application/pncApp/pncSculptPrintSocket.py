import sys
pncApp_project_path = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\MachineKitOpenCNC\\Interface Application\\pncApp\\'
if pncApp_project_path not in sys.path:
    sys.path.append(pncApp_project_path)

import pncLibrary
from multiprocessing import Event
import os, time, logging, socket, pickle, wpipe, numpy as np

#pncLibrary.updatePath()

#Globals
pncApp_connection_event = Event()
pncApp_feedback_synchronization_event = Event()
mvc_connection_type = 'socket'

#There are two set of axis sensors: stepgens (0) and encoders (1)
axis_sensor_id = [0, 1]
SP_data_formats = [['T','X','Z','S','Y','A','B','V','W','BL'], ['T','X','Z','S','Y','A','B','V','W']]

############################# Setup Functions #############################

def monitoredMachineCount():
    return 2

def setupMachineAuxilary():
    return setupAuxiliary()

def setupAuxiliary():
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
    stringArray = [r'Initialize Control',r'Enqueue Movements',r'Execute Motion']
    return stringArray


######################## Operation ########################
def initializeComms(connection_type):
    #global sculptprint_pipe, sculptprint_socket, pncApp_connection_event, sp_pipe_server
    #sculptprint_pipe = wpipe.Client(pncLibrary.sculptprint_ipc_pipe_name, wpipe.Mode.Master, maxmessagesz=4096)

    #sculptprint_pipe_inbound = wpipe.Client(pncLibrary.sculptprint_inbound_ipc_pipe_name, wpipe.Mode.Reader, maxmessagesz=4096)
    #sculptprint_pipe_outbound = wpipe.Client(pncLibrary.sculptprint_outbound_ipc_pipe_name, wpipe.Mode.Writer, maxmessagesz=4096)
    if connection_type == 'pipe':
        try:
            print("SCULPTPRINT INTERFACE: Acquiring pipe %s..." % pncLibrary.sculptprint_ipc_pipe_name)
            sculptprint_pipe = wpipe.Client(pncLibrary.sculptprint_ipc_pipe_name, wpipe.Mode.Master,
                                                maxmessagesz=4096)
            # sp_pipe_server = wpipe.Server('testpipe', wpipe.Mode.Slave,
            #                                     maxmessagesz=4096)
            #print(str(wpipe.getpipepath(self.mvc_pipe)))
            pncApp_connection_event.set()
            print('SCULPTPRINT INTERFACE: Pipe %s opened successfully' % pncLibrary.sculptprint_ipc_pipe_name)
            return sculptprint_pipe
        except:
            raise ConnectionRefusedError
    elif connection_type == 'socket':
        try:

            sculptprint_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sculptprint_socket.connect(('localhost', pncLibrary.mvc_socket_port))
            pncApp_connection_event.set()
            print('SCULPTPRINT INTERFACE: Socket connected successfully to %s' % 'localhost')
            return sculptprint_socket
        except:
            raise ConnectionRefusedError
    #print(sculptprint_pipe)
    #sculptprint_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    #sculptprint_socket.settimeout(1)
    # mvc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    # MVC_socket_outbound = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    #sculptprint_socket.bind((pncLibrary.localhost, pncLibrary.mvc_outbound_port))

def start():
    global MVC_connection
    #global sculptprint_MVC, MVC_socket, machine, synchronizer, terminal_printer, feedback_state
    # global sculptprint_socket
    # sculptprint_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    # sculptprint_socket.setblocking(1)
    # #mvc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    # #MVC_socket_outbound = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    # sculptprint_socket.bind((pncLibrary.localhost, pncLibrary.mvc_outbound_port))

    try:
        MVC_connection = initializeComms(mvc_connection_type)
        if pncLibrary.MVCHandshake(mvc_connection_type, MVC_connection, 'HELLO'):
            print("SUCCESS: Connected to pncApp")
            return True
        # else:
        #     print("SCULPTPRINT INTERFACE: Handshake failed")
        #     return False
    except ConnectionRefusedError:
        print("SCULPTPRINT INTERFACE: Could not connect to pncApp")
        return False
    except TimeoutError:
        print("SCULPTPRINT INTERFACE: pncApp handshake failed")
        return False
    #MVC_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #MVC_socket.bind(('127.0.0.1', 666))
    #sculptprint_MVC, machine, synchronizer, terminal_printer, feedback_state = waitForObjects(MVC_socket)

#def waitForObjects():
def checkAppStart(connection_type):
    pncLibrary.sendIPCData(connection_type, MVC_connection, 'CHECKSTATUS')
    return pncLibrary.receiveIPCData(connection_type, MVC_connection)

# def getPipe():
#     pncLibrary.sendToMVC(sculptprint_socket, 'CHECKSTATUS')
#     return pncLibrary.receiveFromMVC(sculptprint_socket)

def isMonitoring():
    #MVC_socket.sendto('isMonitoring()',('127.0.0.1',666))
    if not pncApp_connection_event.is_set():
        return False

    pncLibrary.sendIPCData(mvc_connection_type, MVC_connection, 'ISMONITORING')
    #MVC_socket_outbound.sendto('ISMONITORING'.encode(), ('127.0.0.1', 666))
    try:
        received_packet = pncLibrary.receiveIPCData(mvc_connection_type, MVC_connection, pncLibrary.pipe_wait_timeout)
        print('ismonitoring returning ' + str(received_packet.data))
        return received_packet.data
    except TimeoutError:
        print('ismonitoring timed out')
        return False
    #return pncLibrary.receiveIPCData(sculptprint_pipe) or False

def readMachine(axis_sensor_id):
    #MVC_socket.sendto('readMachine(' + str(axis_sensor_id) + ')', ('127.0.0.1', 666))
    if not pncApp_connection_event.is_set():
        return []

    pncLibrary.sendIPCData(mvc_connection_type, MVC_connection, 'READ ' + str(axis_sensor_id))
    try:
        if not pncApp_feedback_synchronization_event.is_set():
            timeout = 100
        else:
            timeout = pncLibrary.pipe_wait_timeout

        received_packet = pncLibrary.receiveIPCData(mvc_connection_type, MVC_connection, timeout)
        pncApp_feedback_synchronization_event.set()
        if received_packet.data == True:
            print('break')
        return received_packet.data
    except TimeoutError:
        print('read machine timed out')
        return []
    except EOFError:
        print('socket read error')
        return []

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
        #pncLibrary.sendIPCData(connection_type, connection, 'CLOSE_SP_INTERFACE')
        connection.send(b'\x00')
        #FIXME what a joke
        time.sleep(1)


        # try:
        #     received_packet = pncLibrary.receiveIPCData(mvc_connection_type, MVC_connection, pncLibrary.pipe_wait_timeout)
        #     if received_packet.data == 'CLOSE_SP_INTERFACE_ACK':
        #         return True
        # except TimeoutError:
        #     print("SCULPTPRINT MVC: Socket close ack timeout")
        #     return False

        connection.shutdown(socket.SHUT_RDWR)
        connection.close()

def stop():
    #print('closing')
    #appClose()
    #sculptprint_MVC.command_queue.put('CLOSE')
    #synchronizer.mvc_app_shutdown_event.wait()
    if pncApp_connection_event.is_set():
        closeMVCConnection(mvc_connection_type, MVC_connection)
    return True

def testMonitoring():
    while True:
        z = readMachine(0)
        if z == []:
            print('returning nothing')
        else:
            print('returning good data')
        print('read machine')


#
#     while True:
#         print(eval(input("command: ")))
#         print('looping')

#initializeComms()

start()
while True:
    z = isMonitoring()
    #zz = readMachine(1)
    #print(zz)
    yy = readMachine(0)
    print(yy)
    time.sleep(0.5)
    #stop()
    #time.sleep(2)
# z = isMonitoring()
# zz = readMachine(1)
# print(zz)
stop()
