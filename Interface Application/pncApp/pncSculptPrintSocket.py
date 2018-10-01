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
    return 2

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

############# NEW FUNCTIONS ###############

def setupToolPath(valueDict):
    print('Setting up toolpath ...')
    print('Setup values...\n')
    for key, val in valueDict.items():
        print(key + ' = ' + str(val))
    # global feed_thread
    # feed_thread = feed_server(valueDict)
    return 'SETUPTOOLPATH' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'SETUPTOOLPATH', str, '', valueDict)

# returns False if no points are requested, or a tuple of (True, nRequestedMoves)
def checkForPointRequests():
    #global feed_thread
    return 'CHECKPOINTREQUESTS' in pncLibrary.safelyHandleSocketData(pncApp_connector, 'CHECKPOINTREQUESTS', str, '')
    #return feed_thread.getHasPointRequests()


# updates the toolpath for planning
# this is a list of lists, where the outer list is the requested move, and the inner list is a list of floats representing each contact point
# every TOOLPATHPOINTSIZE float value seperates each point. The floats for each point are the machine axis values, israpid flag as a 1 or 0, volume, move type (1 for step, 0 otherwise)
def updateToolPathPoints(listsOfPoints):
    print('Updating tool path points...')
    global feed_thread
    return feed_thread.updateToolPathPoints(listsOfPoints)


# no functions exists to get these formats as a merged list, because SP needs to know what is a trajectory and auxilary format

# the axis data source formats returned as a list of tuples. Each tuple contains (name, property, length). The property must also be the attribute of the python object
# containing read values returning back to SP
def getAxisDataSourceFormats():
    print('execute getAxisDataSourceFormats()\n')
    formatArray = [(r'planned trajectory', 'planned_positions', NUMAXES + 1),
                   (r'actual trajectory', 'encoder_positions', NUMAXES + 1),
                   (r'stepgen trajectory', 'stepgen_positions', NUMAXES + 1)]
    return formatArray


# the auxilary data source formats returned as a list of tuples. Each tuple contains (name, property, length). The property must also be the attribute of the python object
# containing read values returning back to SP
def getAuxliaryDataSourceFormats():
    print('execute getAuxliaryDataSourceFormats()\n')
    formatArray = [(r'aux data 1', 'aux_values1', 2), (r'aux data 2', 'aux_values2', 2)]
    return formatArray


# user defined functions
# each function is sent a dictionary containing the state values of the user controls
# the dictionary has properties as follows corresponding to each available user control:
# userData1
# userData2
# userData3
# userData4
# userData5
# userSlider1 : value on the interval [0,1]
# userSlider2 : value on the interval [0,1]
# userOption1 : an integer representing the option, currently 1 or 2
# userOption2 : an integer representing the option, currently 1 or 2

def userPythonFunction1(valueDict):
    print('execute userPythonFunction1(...)\n')
    print('stateObject values...\n')
    for key, val in valueDict.items():
        print(key + ' = ' + str(val))
    return True;


def userPythonFunction2(valueDict):
    print('execute userPythonFunction2(...)\n')
    print('stateObject values...\n')
    for key, val in valueDict.items():
        print(key + ' = ' + str(val))
    return True;


def userPythonFunction3(valueDict):
    print('execute userPythonFunction3(...)\n')
    print('stateObject values...\n')
    for key, val in valueDict.items():
        print(key + ' = ' + str(val))
    return True;


def userSlider1StateChange(valueDict):
    print('execute userSlider1StateChange(...)\n')
    print('stateObject values...\n')
    for key, val in valueDict.items():
        print(key + ' = ' + str(val))
    return True;


def userSlider2StateChange(valueDict):
    print('execute userSlider2StateChange(...)\n')
    print('stateObject values...\n')
    for key, val in valueDict.items():
        print(key + ' = ' + str(val))
    return True;


def setupUserDataNames():
    stringArray = [r'my data 1', r'my data 2', r'my data 3', r'my data 4', r'my data 5']
    return stringArray


def setupUserFunctionNames():
    stringArray = [r'my function 1', r'my function 2', r'my function 3']
    return stringArray


def setupUserSliderLabels():
    stringArray = [r'my slider 1', r'my slider 2']
    return stringArray


def setupUserOptionLabels():
    stringArray = [(r'my options 1', 'my option 1', 'my option 2'), (r'my options 2', 'my option 1', 'my option 2')]
    return stringArray



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
    # yy = readMachine(1)
    # try:
    #     print(yy[0])
    # except:
    #     pass
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
