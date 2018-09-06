import sys
#dir_pncApp_project_path = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\MachineKitOpenCNC\\Interface Application\\pncApp\\'
dir_pncApp_project_path = 'E:\\SculptPrint\\PocketNC\\OpenCNC\\Interface Application\\pncApp'
if dir_pncApp_project_path not in sys.path:
    sys.path.append(dir_pncApp_project_path)

print('trying to import library')
import pncLibrary
#import pnc.pncLibrary as pncLibrary
print('imported library')
import os, time

#pncLibrary.updatePath()

#Globals
pncApp_connector = pncLibrary.PNCAppConnection('socket', 'text', 'binary')

#There are two set of axis sensors: stepgens (0) and encoders (1)

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

def setupMachineDescriptors(nMachine):
    return [0] + [1 for k in range(0,len(pncLibrary.SP_CAM_machine_axes[nMachine]))] + [2 for k in range(0,len(pncLibrary.SP_auxiliary_data_labels[nMachine])) if pncLibrary.SP_auxiliary_data_labels[nMachine][k] != '']

def setupUserDataNames():
    stringArray = [r'Start File', r'End File', r'my data 3', r'my data 4', r'my data 5']
    return stringArray

# Returns an array of user defined function names that are displayed on the function buttons in the feature UI. The array affects
# feature UI functionality only.  The array must be sized on the interval [1,3]. Defining this method is optional.
def setupUserFunctionNames():
    stringArray = [r'Plan to Sequence',r'NULL',r'Execute Motion']
    return stringArray

def start():
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

def userPythonFunction1(arg0, arg1, arg2, arg3, arg4):
    if arg0 == 0:
        arg0 = 5
    # return 'VOXELENQUEUE' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'VOXELENQUEUE_' + str(int(arg0)) + '_' + str(int(arg1)), str, '')
    #return 'TRAPENQUEUE' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'TRAPENQUEUE_' + str(int(arg0)), str, '')
    return 'PLAN' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'PLAN_' + str(int(arg0)), str, '')


def userPythonFunction2(arg0, arg1, arg2, arg3, arg4):
    #print('execute userPythonFunction2(' + str(arg0) + ',' + str(arg1) + ',' + str(arg2) + ',' + str(arg3) + ',' + str(arg4) + ')\n')
    #return 'TRAPENQUEUE' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'TRAPENQUEUE_' + str(int(arg0)), str, '')
    #return 'EXECUTE' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'EXECUTE', str, '')
    return True

def userPythonFunction3(arg0, arg1, arg2, arg3, arg4):
    #return 'PLAN' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'PLAN_' + str(int(arg0)), str, '')
    return 'EXECUTE' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'EXECUTE', str, '')

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
    userPythonFunction1(2, 0, 0, 0, 0)
    #time.sleep(5)
    #userPythonFunction2(0, 0, 0, 0, 0)
    #userPythonFunction3(2,0,0,0,0)
    #time.sleep(3)
    #userPythonFunction3(0,0,0,0,0)

    #zz = readMachine(1)
    while 1:
        z = isMonitoring()
        print(z)
        yy = readMachine(0)
        try:
            print(yy)
        except:
            pass
        #time.sleep(0.1)
        print('encoder data is: ')
        yy = readMachine(1)
        try:
            print(yy[-1])
        except:
            pass
            print("print break")

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
