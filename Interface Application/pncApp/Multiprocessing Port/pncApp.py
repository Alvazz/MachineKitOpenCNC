import socket
import sys
import numpy as np
import threading
import time

#PNC Modules
#from pnc.pncMachineControl import MachineController
#from pnc.pncMachineFeedback import MachineFeedbackListener
#from pnc.pncDataStore import DataStore
#from pnc.pncMachineModel import MachineModel

from pncMachineControl import MachineController
from pncMachineFeedback import MachineFeedbackListener, SerialInterface
from pncDataStore import DataStore
from pncDataStore import DataStoreManager
from pncMachineModel import MachineModel
from pncCamUserInterface import SculptPrintInterface

#Handles
global data_store, machine_controller, machine, encoder_interface, feedback_listener

# Default connection parameters
# def_feedback_listen_ip = '0.0.0.0'
# def_feedback_listen_port = 515
# def_control_client_ip = '129.1.15.5'
# #def_control_client_ip = '129.1.15.69'
# def_control_client_port = 5007

# Initialize control communication with PocketNC using TCP and feedback read
# communication with UDP.
def appInit(feedback_listen_ip = -1,
             feedback_listen_port = -1,
             control_client_ip = -1,
             control_client_port = -1):
    global data_store, machine_controller, machine, encoder_interface, feedback_listener

    #Begin CPU clock
    local_epoch = time.clock()

    data_store = DataStore()
    data_store_manager = DataStoreManager()
    machine = MachineModel()

    machine.sculptprint_interface = SculptPrintInterface()
    machine.local_epoch = local_epoch
    machine.data_store_manager_thread_handle = data_store_manager
    data_store_manager.machine = machine
    #print(machine.axis_offsets)

    # if feedback_listen_ip == -1:
    #     feedback_listen_ip = machine.listen_ip
    # if feedback_listen_port == -1:
    #     feedback_listen_port = machine.udp_port
    if control_client_ip == -1:
        control_client_ip = machine.ip_address
    if control_client_port == -1:
        control_client_port = machine.tcp_port

    # try:
    #     feedback_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #     feedback_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #     feedback_socket.bind((feedback_listen_ip, feedback_listen_port))
    # except socket.error:
    #     print ('Failed to bind to feedback socket to listen on 2')
    #     sys.exit()

    try:
        control_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        control_socket.connect((machine.ip_address, control_client_port))
        machine.rsh_socket = control_socket
    except socket.error:
        print ('Failed to connect to client IP. Is machine ON?')
        sys.exit()      

    print ('[+] Listening for feedback data on port', feedback_listen_port)
    print ('[+] Connection to control client (emcrsh) established at address',
                control_client_ip,'on port', control_client_port)

    machine.data_store_manager_thread_handle = data_store_manager
    data_store_manager.start()

    encoder_interface = SerialInterface(machine)
    machine.encoder_thread_handle = encoder_interface
    encoder_interface.start()

    machine_controller = MachineController(machine, data_store)
    machine.machine_controller_thread_handle = machine_controller
    machine_controller.start()

    feedback_listener = MachineFeedbackListener(machine)
    machine.feedback_listener_thread_handle = feedback_listener
    feedback_listener.start()

    print('pncApp Started Successfully')

    return machine, feedback_listener, machine_controller, encoder_interface, data_store

def appClose():
    print('closing sockets')
    if machine_controller != []:
        #machine_controller.shutdown()
        print('flagging machine controller thread to shutdown')
        machine_controller._running_thread = False
        while not machine_controller._shutdown:
            #Wait for shutdown flag
            #print('pncApp: waiting for machine controller shutdown')
            pass
        print('pncApp: machine controller shutdown')
    print('active threads are ' + str(threading.enumerate()))
	
# Global variables
#data_store = DataStore()
data_store = []
machine_controller = []
feedback_listener = []
encoder_interface = []

#appInit()
#machine_controller.connectAndLink()
#machine_controller.setBinaryMode(1)
# machine_controller.login()
# machine_controller.testMachine(1,1,1,1,1)