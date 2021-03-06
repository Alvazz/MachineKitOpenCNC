import socket
import sys

#PNC Modules
from pnc.pncMachineControl import MachineController
from pnc.pncMachineFeedback import MachineFeedbackListener
from pnc.pncDataStore import DataStore
from pnc.pncMachineModel import MachineModel

#from pncApp.pncMachineControl import MachineController
#from pncApp.pncMachineFeedback import MachineFeedbackListener
#from pncApp.pncDataStore import DataStore
#from pncApp.pncMachineModel import MachineModel

#Handles
global data_source, machine_controller

# Default connection parameters
def_feedback_listen_ip = '0.0.0.0'
def_feedback_listen_port = 514
def_control_client_ip = '129.1.15.5'
def_control_client_port = 5007

# Initialize control communication with PocketNC using TCP and feedback read
# communication with UDP.
def appInit(feedback_listen_ip = def_feedback_listen_ip,
             feedback_listen_port = def_feedback_listen_port,
             control_client_ip = def_control_client_ip,
             control_client_port = def_control_client_port):
    global data_source, machine_controller

    machine = MachineModel()

    try:
        feedback_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        feedback_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        feedback_socket.bind((feedback_listen_ip, feedback_listen_port))
    except socket.error:
        print ('Failed to bind to feedback socket to listen on 2')
        sys.exit()

    try:
        control_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        control_socket.connect((control_client_ip, control_client_port))
    except socket.error:
        print ('Failed to connect to client ip for giving it control')
        sys.exit()      

    print ('[+] Listening for feedback data on port', feedback_listen_port)
    print ('[+] Connection to control client (emcrsh) established at address',
                control_client_ip,'on port', control_client_port)

    feedback_listener = MachineFeedbackListener(feedback_socket, data_source)
    feedback_listener.start()

    machine_controller = MachineController(control_socket, machine, data_source)
    machine_controller.start()

def appClose():
	print('closing sockets')
	if machine_controller != []:
		#machine_controller.shutdown()
		machine_controller.close()
	if feedback_listener != []:
		#feedback_listener.shutdown()
		feedback_listener.close()
	#bokehIntf.close()
	
# Global variables
data_source = DataStore()
machine_controller = []
feedback_listener = []
