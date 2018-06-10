import multiprocessing, socket, sys,  time, inspect
from threading import Thread, current_thread

#PNC Modules
from pncMachineModel import MachineModel, MachineModelProxy
from pncDatabase import DatabaseServer
from pncMachineControl import MachineController
from pncMachineFeedback import MachineFeedbackHandler
from pncEncoderInterface import EncoderInterface
from pncCamUserInterface import SculptPrintInterface
import pncLibrary

class PNCAppManager(multiprocessing.managers.SyncManager): pass

def registerProxy(name, cls, proxy, manager):
    for attr in dir(cls):
        if inspect.ismethod(getattr(cls, attr)) and not attr.startswith("__"):
            proxy._exposed_ += (attr,)
            setattr(proxy, attr,
                    lambda s: object.__getattribute__(s, '_callmethod')(attr))
    manager.register(name, cls, proxy)

def openNetworkConnection(machine, control_client_ip, control_client_port):
    try:
        machine.rsh_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        machine.rsh_socket.connect((control_client_ip, control_client_port))
        pncLibrary.printTerminalString(machine.connection_string, machine.name, control_client_ip, control_client_port)
        return True
    except socket.error:
        pncLibrary.printTerminalString(machine.failed_connection_string, machine.name, control_client_ip, control_client_port)
        return False

def appInit():
    main_process_name = str(multiprocessing.current_process().name)
    main_process_pid = str(multiprocessing.current_process().pid)

    print('Current process: ' + str(multiprocessing.current_process().pid) + ', current thread: ' + str(
        current_thread().name))
    registerProxy('MachineModel', MachineModel, MachineModelProxy, PNCAppManager)
    # registerProxy('Synchronizers', Synchronizers, SynchronizersProxy, PNCAppManager)
    # registerProxy('DatabaseServer', DatabaseServer, DatabaseServerProxy)

    # FIXME ignore SIGINT
    print('starting new process')
    pnc_app_manager = PNCAppManager()
    pnc_app_manager.name = 'pnc_app_manager'
    pnc_app_manager.start()

    # synchronizers = Synchronizers(pnc_app_manager)
    machine = pnc_app_manager.MachineModel()
    synchronizer = pncLibrary.Synchronizer(pnc_app_manager)
    pncLibrary.setTaskRunFlags(synchronizer)
    machine.sculptprint_interface = SculptPrintInterface()
    machine.local_epoch = time.clock()

    pncLibrary.printTerminalString(machine.process_launch_string, pnc_app_manager.name, pnc_app_manager._process.pid,
                                   main_process_name, main_process_pid)

    return pnc_app_manager, machine, synchronizer

def appStart(type, pnc_app_manager, machine, synchronizer, control_client_ip = -1, control_client_port = -1):
    #global pnc_app_manager, machine, synchronizer

    # main_process_name = str(multiprocessing.current_process().name)
    # main_process_pid = str(multiprocessing.current_process().pid)
    #
    # print('Current process: ' + str(multiprocessing.current_process().pid) + ', current thread: ' + str(current_thread().name))
    # registerProxy('MachineModel', MachineModel, MachineModelProxy, PNCAppManager)
    # #registerProxy('Synchronizers', Synchronizers, SynchronizersProxy, PNCAppManager)
    # #registerProxy('DatabaseServer', DatabaseServer, DatabaseServerProxy)
    #
    # #FIXME ignore SIGINT
    # print('starting new process')
    # pnc_app_manager = PNCAppManager()
    # pnc_app_manager.name = 'pnc_app_manager'
    # pnc_app_manager.start()
    #
    # #synchronizers = Synchronizers(pnc_app_manager)
    # machine = pnc_app_manager.MachineModel()
    # synchronizer = pncLibrary.Synchronizer(pnc_app_manager)
    # pncLibrary.setTaskRunFlags(synchronizer)
    # machine.sculptprint_interface = SculptPrintInterface()
    # machine.local_epoch = time.clock()

    #pncLibrary.printTerminalString(machine.process_launch_string, pnc_app_manager.name, pnc_app_manager._process.pid, main_process_name, main_process_pid)

    if type == 'simulator':
        control_client_ip = machine.simulator_ip_address
        machine.name = machine.simulator_name
    elif type == 'pocketnc':
        control_client_ip = machine.ip_address
        machine.name = machine.machine_name
    else:
        print('Unrecognized type ' + type + ' for machine initialization, aborting')
        return
    control_client_port = machine.tcp_port

    # database_push_queue_proxy = pnc_app_manager.Queue()
    # database_pull_queue_proxy = pnc_app_manager.Queue()
    # database_output_queue_proxy = pnc_app_manager.Queue()
    #testevent = pnc_app_manager.Event()

    database = DatabaseServer(machine, synchronizer)
    database.start()
    pncLibrary.printTerminalString(machine.process_launch_string, database.name, database.pid, main_process_name, main_process_pid)

    encoder_interface = EncoderInterface(machine, synchronizer)
    encoder_interface.start()
    pncLibrary.printTerminalString(machine.process_launch_string, encoder_interface.name, encoder_interface.pid, main_process_name, main_process_pid)

    feedback_handler = MachineFeedbackHandler(machine, synchronizer)
    feedback_handler.start()
    pncLibrary.printTerminalString(machine.process_launch_string, feedback_handler.name, feedback_handler.pid, main_process_name, main_process_pid)

    machine_controller = MachineController(machine, synchronizer)
    machine_controller.start()
    pncLibrary.printTerminalString(machine.process_launch_string, machine_controller.name, machine_controller.pid, main_process_name, main_process_pid)

    openNetworkConnection(machine, control_client_ip, control_client_port)
    waitForAppStart(synchronizer)

    pnc_app_manager.machine = machine
    pnc_app_manager.database = database
    pnc_app_manager.encoder_interface = encoder_interface
    pnc_app_manager.machine_controller = machine_controller
    pnc_app_manager.feedback_handler = feedback_handler

    signalProcessWake(synchronizer)
    pncLibrary.printTerminalString(machine.pncApp_launch_string, multiprocessing.cpu_count())

    return database, encoder_interface, feedback_handler, machine_controller

def appStop():
    #FIXME just pause activity, don't actually shut down
    pass

def appClose():
    print('pnc_app_manager is ' + str(pnc_app_manager))
    pnc_app_manager.machine.run_machine_controller = False
    pnc_app_manager.machine_controller.join()

def waitForAppStart(synchronizer):
    synchronizer.db_startup_event.wait()
    synchronizer.ei_startup_event.wait()
    synchronizer.fb_startup_event.wait()
    synchronizer.mc_startup_event.wait()

def signalProcessWake(synchronizer):
    synchronizer.process_start_signal.set()

    # if machine_controller != []:
    #     #machine_controller.shutdown()
    #     print('flagging machine controller thread to shutdown')
    #     machine_controller._running_thread = False
    #     while not machine_controller._shutdown:
    #         #Wait for shutdown flag
    #         #print('pncApp: waiting for machine controller shutdown')
    #         pass
    #     print('pncApp: machine controller shutdown')

    #print('active threads are ' + str(threading.enumerate()))

# if __name__ == '__main__':
#     pnc_app_manager, machine, database, encoder_interface, machine_controller, feedback_handler, synchronizer = appInit('pocketnc')
#     #Store process handles in pnc_app_manager?
#     synchronizer.mvc_connect_event.set()
#     machine_controller.join()
#     #time.sleep(1)
#     appClose()