import multiprocessing, socket, sys,  time, inspect

#PNC Modules
from pncMachineModel import MachineModel, MachineModelProxy#, MachineModelMethods
from pncDatabase import DatabaseServer, DatabaseServerProxy
from pncMachineControl import MachineController
from pncMachineFeedback import MachineFeedbackListener
from pncEncoderInterface import EncoderInterface
from pncDataStore import DataStore

from pncCamUserInterface import SculptPrintInterface

#Handles
#global data_store, machine_controller, machine, encoder_interface, feedback_listener

# Default connection parameters
# def_feedback_listen_ip = '0.0.0.0'
# def_feedback_listen_port = 515
# def_control_client_ip = '129.1.15.5'
# #def_control_client_ip = '129.1.15.69'
# def_control_client_port = 5007

# class IPCPrimitives():
#     def __init__(self, manager):
#         self.database_push_queue = manager.Queue()

        #self.database_lock = manager.Lock()

# def initSynchronizationPrimitives(manager):
#     database_queue = manager.Queue()
#     database_lock = manager.Lock()

# Initialize control communication with PocketNC using TCP and feedback read
# communication with UDP.
def appInit(control_client_ip = -1, control_client_port = -1, simulate = False):
    global data_store, machine_controller, machine, encoder_interface, feedback_listener

    registerProxy('MachineModel',MachineModel,MachineModelProxy, PNCAppManager)
    #registerProxy('DatabaseServer', DatabaseServer, DatabaseServerProxy)


    pncAppManager = PNCAppManager()
    pncAppManager.start()

    #synchronizers = Synchronizers(pncAppManager)
    machine = pncAppManager.MachineModel()

    #machine.synchronizers = Synchronizers(pncAppManager)
    machine.local_epoch = time.clock()

    database_push_queue_proxy = pncAppManager.Queue()
    database_pull_queue_proxy = pncAppManager.Queue()
    database_output_queue_proxy = pncAppManager.Queue()

    #machine.database_push_queue_proxy = database_push_queue_proxy

    database = DatabaseServer(machine, database_push_queue_proxy, database_pull_queue_proxy, database_output_queue_proxy)
    database.start()


    #synchronizers = Synchronizers(pncAppManager)
    #machine.synchronizers = initSynchronizers(pncAppManager)

    # testprocess = test()
    # testprocess.start()

    #database = pncAppManager.DatabaseServer(machine)

    #machine.database_proxy = database

    #machineself = machine.getSelf()


    #machine.database_lock = database_lock
    #machine.database_input_queue = synchronizers.database_intput_queue
    #machine.database_output_queue = synchronizers.database_intput_queue

    #machine_model_methods = MachineModelMethods(machine)
    #machine = pncAppManager.Namespace()

    #Begin CPU clock
    # machine.setRSHError()
    # print('rsh error is ' + str(machine.rsh_error))
    # machine.resetRSHError()
    # print('rsh error is ' + str(machine.rsh_error))
    #local_epoch = time.clock()

    #data_store = DataStore()
    #data_store_manager = DatabaseServer(machine, database_input_queue, database_output_queue)
    #data_store_manager.start()
    #machine = MachineModel()

    #machine.sculptprint_interface = SculptPrintInterface()

    #machine.data_store_manager_thread_handle = data_store_manager
    #data_store_manager.machine = machine
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
        machine.rsh_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        machine.rsh_socket.connect((control_client_ip, control_client_port))
        print('CONNECTED to ' + machine.name + ' remote shell at ' + str(control_client_ip) + ':' + str(control_client_port))
        #machine.rsh_socket = control_socket
    except socket.error:
        print ('Failed to connect to client IP. Is machine ON?')
        #return appClose()
        #sys.exit()

    #print ('[+] Listening for feedback data on port', feedback_listen_port)


    # serial_port = serial.Serial()
    # serial_port.port = machine.comm_port
    # serial_port.baudrate = machine.baudrate
    # FIXME handle if serial is not connected


    # machine.data_store_manager_thread_handle = data_store_manager
    # data_store_manager.start()

    #encoder_interface = EncoderInterface(machine, database_push_queue_proxy)
    encoder_interface = EncoderInterface(machine, database_push_queue_proxy)
    #machine.encoder_thread_handle = encoder_interface
    encoder_interface.start()

    machine_controller = MachineController(machine, database_push_queue_proxy)
    #machine.machine_controller_thread_handle = machine_controller
    machine_controller.start()

    feedback_listener = MachineFeedbackListener(machine, database_push_queue_proxy)
    #machine.feedback_listener_thread_handle = feedback_listener
    feedback_listener.start()

    print('pncApp Started Successfully')

    #return machine, feedback_listener, machine_controller, encoder_interface, data_store

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
    #print('active threads are ' + str(threading.enumerate()))
	

### MULTIPROCESSING STUFF ###
class PNCAppManager(multiprocessing.managers.SyncManager): pass

# self.connection_change_event = Event()
#         self.link_change_event = Event()
#         self.echo_change_event = Event()
#         self.estop_change_event = Event()
#         self.drive_power_change_event = Event()
#         self.status_change_event = Event()
#         #self.status_change_event.clear()
#         self.mode_change_event = Event()
#         self.logging_mode_change_event = Event()
#         self.comm_mode_change_event = Event()
#         self.home_change_event = Event()
#         self.all_homed_event = Event()
#         self.restore_mode_event = Event()
#         self.ping_event = Event()
#         self.clock_event = Event()
#         self.buffer_level_reception_event = Event()
#         self.servo_feedback_reception_event = Event()
#         #FIXME implement this
#         self.position_change_event = Event()
#         self.initial_position_set_event = Event()
#         self.encoder_init_event = Event()

def registerProxy(name, cls, proxy, manager):
    for attr in dir(cls):
        if inspect.ismethod(getattr(cls, attr)) and not attr.startswith("__"):
            proxy._exposed_ += (attr,)
            setattr(proxy, attr,
                    lambda s: object.__getattribute__(s, '_callmethod')(attr))
    manager.register(name, cls, proxy)


# Global variables
#data_store = DataStore()
# data_store = []
# machine_controller = []
# feedback_listener = []
# encoder_interface = []

if __name__ == '__main__':
    appInit()