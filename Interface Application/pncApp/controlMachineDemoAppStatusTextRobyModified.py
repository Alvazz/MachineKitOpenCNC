# This python script serves as an example for providing a control
# machine monitoring capability to SculptPrint for feeding back axes values
# to the Control Machine feature.
import time
import threading
import copy
import numpy as np

# the number of axes for the machine model, must be the same as the machine model in SP
NUMAXES = 8
#NUMAXES + 3, the machine axis data plus the israpid flag, volume, and move type (1 step, 0 otherwise)
TOOLPATHPOINTSIZE = 11
SP_axis_data_sample_format = ['T','X','Z','S','Y','A','B','V','W']
SP_auxiliary_data_sample_format = ['T', 'D']

feed_thread = None
last_read_time = time.clock()
integrated_read_time = 0
num_times_read = 0
first_read = False

#utility class to encapsulate the various points with attributes of the same name as property names contained in the AxisDataFormats and AuxilaryDataFormats
class PointBuffers:
    planned_positions = []
    encoder_positions = []
    stepgen_positions = []
    aux_values1 = []
    aux_values2 = []

# feed_server is an example of a thread that feeds axis values
# to the control machine monitor.  A real thread would be reading values
# from the machine through a socket or other live connection.
# Replace feed_server with a thread that connects to the machine
# and reads back values.
class feed_server(threading.Thread):
    def __init__(self, setupDict):
        super(feed_server, self).__init__()
        self.numPoints = setupDict.get('numberofPoints')
        self.nMoves = setupDict.get('numberofSequences')
        self.nCurrentMove = 0
        self.numPointsRead = 0
        self.active = True;
        self.toolpathBuffer = []
        self.currentMove = 0
        self.currentPoint = 0
        self.needsPathPoints = True
        self.buffers = PointBuffers()
        self.lock = threading.Lock()

        self.command_data = np.genfromtxt('E:\\SculptPrint\\PocketNC\\Experimental Data\\head_export_COMMANDED_SERVO_POSITIONS.csv')[1:]
        self.stepgen_data = np.genfromtxt('E:\\SculptPrint\\PocketNC\\Experimental Data\\head_export_STEPGEN_FEEDBACK_POSITIONS.csv')[1:]
        self.encoder_data = np.genfromtxt('E:\\SculptPrint\\PocketNC\\Experimental Data\\head_export_ENCODER_FEEDBACK_POSITIONS.csv')[1:]
        self.tcq_data = np.genfromtxt('E:\\SculptPrint\\PocketNC\\Experimental Data\\head_export_HIGHRES_TC_QUEUE_LENGTH.csv')[1:]
        self.pid_data = np.genfromtxt('E:\\SculptPrint\\PocketNC\\Experimental Data\\head_export_NETWORK_PID_DELAYS.csv')[1:]

    
    # get the points in the toolpathBuffer, use as is for planned, and then modify the rest to serve as an example
    def run(self):
        t = 0.0
        time_offset = time.clock()
        while self.isActive():
            #apoint = self.getNextPoint()
            current_time = time.clock() - time_offset
            with self.lock:
                try:
                    stepgen_clock_index = np.where(self.stepgen_data[:,0] <= current_time)[0][-1]
                    stepgenValues = self.stepgen_data[:stepgen_clock_index, :].tolist()
                    self.buffers.stepgen_positions += stepgenValues
                    self.stepgen_data = self.stepgen_data[stepgen_clock_index:,:]
                except:
                    stepgenValues = []

                try:
                    planned_clock_index = np.where(self.command_data[:,0] <= current_time)[0][-1]
                    plannedValues = self.command_data[:planned_clock_index, :].tolist()
                    self.buffers.planned_positions += plannedValues
                    self.command_data = self.command_data[planned_clock_index:,:]
                except:
                    plannedValues = []
                try:
                    encoder_clock_index = np.where(self.stepgen_data[:, 0] <= current_time)[0][-1]
                    encoderValues = self.encoder_data[:encoder_clock_index, :].tolist()
                    self.buffers.encoder_positions += encoderValues
                    self.encoder_data = self.encoder_data[encoder_clock_index:,:]
                except:
                    encoderValues = []

                try:
                    tcq_clock_index = np.where(self.tcq_data[:, 0] <= current_time)[0][-1]
                    auxValues1 = self.tcq_data[:tcq_clock_index, :].tolist()
                    self.buffers.aux_values1 += auxValues1
                    self.tcq_data = self.tcq_data[tcq_clock_index:,:]
                except:
                    auxValues1 = []

                try:
                    pid_clock_index = np.where(self.pid_data[:, 0] <= current_time)[0][-1]
                    auxValues2 = self.pid_data[:pid_clock_index, :].tolist()
                    self.buffers.aux_values2 += auxValues2
                    self.pid_data = self.pid_data[pid_clock_index:,:]
                except:
                    auxValues2 = []

    def addToBuffer(self, plannedValues, encoderValues, stepgenValues, auxValues1, auxValues2):
        self.lock.acquire()
        self.buffers.planned_positions += plannedValues
        self.buffers.encoder_positions += encoderValues
        self.buffers.stepgen_positions += stepgenValues
        self.buffers.aux_values1 += auxValues1
        self.buffers.aux_values2 += auxValues2
        self.lock.release()
        
    def getNextPoint(self):
        thepoint = []
        self.lock.acquire()
        if self.currentMove < len(self.toolpathBuffer):
            movepoints = self.toolpathBuffer[self.currentMove]
            if len(movepoints) > 0:
                if self.currentPoint + TOOLPATHPOINTSIZE-1 < len(movepoints):
                    for i in range(0, TOOLPATHPOINTSIZE):
                        thepoint.append(float(movepoints[self.currentPoint + i]))
                    self.currentPoint = self.currentPoint + TOOLPATHPOINTSIZE;
                    if self.currentPoint >= len(movepoints):
                        self.currentMove = self.currentMove + 1
                        self.currentPoint = 0
                    else:
                        if self.currentPoint + TOOLPATHPOINTSIZE >= len(movepoints) and self.currentMove + 1 < self.nMoves:
                            self.needsPathPoints = True
                else:
                    self.currentMove = self.currentMove + 1
                    self.currentPoint = 0
            else:
                self.currentMove = self.currentMove + 1
                self.currentPoint = 0
        else:
            if self.currentMove < self.nMoves and self.needsPathPoints == False:
                self.needsPathPoints = True
            elif self.currentMove >= self.nMoves:
                print('No more moves')    
        self.lock.release()
        return thepoint
        
    def isActive(self):
        localActive = False;
        self.lock.acquire()
        localActive = self.active
        self.lock.release()
        return localActive
        
    def deactivate(self):
        self.lock.acquire()
        self.active = False
        self.lock.release()
        
    def activate(self):
        self.lock.acquire()
        self.active = True
        self.lock.release()
        
    def close(self):
        print('Thread read ' + str(self.numPointsRead) + '.');
        
    def retreiveBuffer(self):
        self.lock.acquire()
        localbuffer = PointBuffers()
        localbuffer.planned_positions = []
        localbuffer.encoder_positions = []
        localbuffer.stepgen_positions = []
        localbuffer.aux_values1 = []
        localbuffer.aux_values2 = []
        
        for i in range(0, len(self.buffers.planned_positions)):
            localbuffer.planned_positions.append(copy.deepcopy(self.buffers.planned_positions[i]))
            
        for i in range(0, len(self.buffers.encoder_positions)):
            localbuffer.encoder_positions.append(copy.deepcopy(self.buffers.encoder_positions[i]))
        
        for i in range(0, len(self.buffers.stepgen_positions)):
            localbuffer.stepgen_positions.append(copy.deepcopy(self.buffers.stepgen_positions[i]))
        
        for i in range(0, len(self.buffers.aux_values1)):
            localbuffer.aux_values1.append(copy.deepcopy(self.buffers.aux_values1[i]))
        
        for i in range(0, len(self.buffers.aux_values2)):
            localbuffer.aux_values2.append(copy.deepcopy(self.buffers.aux_values2[i]))
        
        self.buffers.planned_positions = []
        self.buffers.encoder_positions = []
        self.buffers.stepgen_positions = []
        self.buffers.aux_values1 = []
        self.buffers.aux_values2 = []
        self.lock.release()
        return localbuffer
        
    def incrementNumPointsRead(self):
        self.lock.acquire()
        self.numPointsRead = self.numPointsRead + 1
        self.lock.release()
        
    def getHasPointRequests(self):
        self.lock.acquire()
        b = self.needsPathPoints
        self.lock.release()
        if b:
            ret = (True, 1)
            return ret
        else:
            return False
        
    def updateToolPathPoints(self, listsOfPoints):
        self.lock.acquire()
        print('Updating: ' + str(len(listsOfPoints)) + ' move sequences...');
        #each points is a tuple of 3 elements (0: sequence number, 1: tool space points (x, y, z, theta, phi), 2: joint space points
        for points in listsOfPoints:
            self.toolpathBuffer.append(points[2])
        self.needsPathPoints = False
        print('Toolpath size: ' + str(len(self.toolpathBuffer)) + '.');
        self.lock.release()
        return True

# Called to start monitoring the machine.
# Will execute when the start button is pressed in the Control Machine feature.
# Return true if able to start monitoring.
def start():
    global feed_thread
    print('Starting was called and will attempt to start feed thread.')
    feed_thread.start()
    return True

# Called to read axis values accumulated by the monitoring process.
# Must return a python class with attributes named the same as defined in the getAxisDataSourceFormats() and getAuxliaryDataSourceFormats() functions
# Each attribute contains a list of lists of float values. Not all axis and auxilary formats must have an entry in the returned python object
# The values for the axis formats must be a list of lists of time plus axis values.  For example, pointBuffer[i][j]
# will hold j-th axis values plus time of the i-th set of axis values.  The number of values should match the number contained in the axis data source format. The order of axes
# should match that shown in SculptPrint when selecting the Machine node in the
# document manager and the list of axes is shown on the panel in the lower left.
# Auxilary formats should return a list of lists where the inne list contains the same number of values as defined by the auxilary source data format.
# Currently the UI in SP only will handle 1 value for each auxilary source.
def read():
    global feed_thread, last_read_time, integrated_read_time, num_times_read, first_read
    num_times_read += 1
    pointBuffer = feed_thread.retreiveBuffer()
    #print(str(pointBuffer.planned_positions))
    if first_read is False:
        first_read = True
        # read_dt = time.clock()-last_read_time
        # print('read deltaT is ' + str(read_dt))
        # integrated_read_time += read_dt
        # print('average read dT is ' + str(integrated_read_time / num_times_read))
        last_read_time = time.clock()
    else:
        read_dt = time.clock() - last_read_time
        print('read deltaT is ' + str(read_dt))
        integrated_read_time += read_dt
        print('average read dT is ' + str(integrated_read_time / num_times_read))
        last_read_time = time.clock()
    return pointBuffer
    
#provides information to setup a toolpass
#dictionary contains the following attributes and data
#numberofSequences: the number of contiguous moves in the toolpass
#numberofPoints: number of points in the toolpass
#toolToHolderMatrix: the tool holder matrix
#tableToPartMatrix: the table to part matrix
#sculptprintFileName: the SP filename
#toolpathName: the toolpass name
def setupToolPath(valueDict):
    print('Setting up toolpath ...')
    print('Setup values...\n')
    for key, val in valueDict.items():
        print(key + ' = ' + str(val))
    global feed_thread
    feed_thread = feed_server(valueDict)
    return True;
  
#returns False if no points are requested, or a tuple of (True, nRequestedMoves)
def checkForPointRequests():
    global feed_thread
    return feed_thread.getHasPointRequests()

#updates the toolpath for planning
#this is a list of lists, where the outer list is the requested move, and the inner list is a list of floats representing each contact point
#every TOOLPATHPOINTSIZE float value seperates each point. The floats for each point are the machine axis values, israpid flag as a 1 or 0, volume, move type (1 for step, 0 otherwise)
def updateToolPathPoints(listsOfPoints):
    print('Updating tool path points...')
    global feed_thread
    return feed_thread.updateToolPathPoints(listsOfPoints)

#no functions exists to get these formats as a merged list, because SP needs to know what is a trajectory and auxilary format

#the axis data source formats returned as a list of tuples. Each tuple contains (name, property, length). The property must also be the attribute of the python object
#containing read values returning back to SP
def getAxisDataSourceFormats():
    print('execute getAxisDataSourceFormats()\n')
    formatArray = [(r'planned trajectory', 'planned_positions', NUMAXES+1),(r'actual trajectory', 'encoder_positions', NUMAXES+1), (r'stepgen trajectory', 'stepgen_positions', NUMAXES+1)]
    return formatArray

#the auxilary data source formats returned as a list of tuples. Each tuple contains (name, property, length). The property must also be the attribute of the python object
#containing read values returning back to SP
def getAuxliaryDataSourceFormats():
    print('execute getAuxliaryDataSourceFormats()\n')
    formatArray = [(r'aux data 1', 'aux_values1', 2),(r'aux data 2', 'aux_values2', 2)]
    return formatArray

#user defined functions
#each function is sent a dictionary containing the state values of the user controls
#the dictionary has properties as follows corresponding to each available user control:
#userData1
#userData2
#userData3
#userData4
#userData5
#userSlider1 : value on the interval [0,1]
#userSlider2 : value on the interval [0,1]
#userOption1 : an integer representing the option, currently 1 or 2
#userOption2 : an integer representing the option, currently 1 or 2

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
    stringArray = [r'my function 1',r'my function 2',r'my function 3']
    return stringArray

def setupUserSliderLabels():
    stringArray = [r'my slider 1',r'my slider 2']
    return stringArray

def setupUserOptionLabels():
    stringArray = [(r'my options 1', 'my option 1', 'my option 2'),(r'my options 2', 'my option 1', 'my option 2')]
    return stringArray

def getStatusText():
    global feed_thread
    if feed_thread == None:
        return "Unknown: setup required"
    elif feed_thread.is_alive():
        return 'Machine ON\nTrajectory Planner Connected\nReady for Motion'
    else:
        return 'not monitoring'

# Called to stop monitoring the machine.
# Will execute when the stop button is pressed in the Monitor Machine feature.
def stop():
    global feed_thread
    if feed_thread.is_alive():
        print('Feed thread is still alive')
    else:
        print('Feed thread is not alive')
    feed_thread.deactivate()
    feed_thread.join()
    feed_thread.close()
    return True;
    

time.clock()