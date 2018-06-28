import multiprocessing, socket, sys,  time, inspect
from threading import Thread, current_thread

#PNC Modules
from pncMachineModel import MachineModel, MachineModelProxy
from pncDatabase import DatabaseServer
from pncMachineControl import MachineController
from pncMachineFeedback import MachineFeedbackHandler
from pncEncoderInterface import EncoderInterface
#from pncLibrary import SculptPrintInterface
import pncLibrary

class PNCAppManager(multiprocessing.managers.SyncManager): pass

def registerProxy(name, cls, proxy, manager):
    for attr in dir(cls):
        if inspect.ismethod(getattr(cls, attr)) and not attr.startswith("__"):
            proxy._exposed_ += (attr,)
            setattr(proxy, attr,
                    lambda s: object.__getattribute__(s, '_callmethod')(attr))
    manager.register(name, cls, proxy)

def openNetworkConnection(machine, synchronizer, control_client_ip, control_client_port):
    try:
        #machine.rsh_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rsh_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rsh_socket.settimeout(machine.socket_timeout)
        #machine.rsh_socket = rsh_socket
        #FIXME put in a timeout here
        #machine.rsh_socket.settimeout(machine.socket_timeout)
        rsh_socket.connect((control_client_ip, control_client_port))
        #machine.rsh_socket.connect((control_client_ip, control_client_port))
        #machine.rsh_socket.settimeout(None)
        rsh_socket.settimeout(None)
        machine.rsh_socket = rsh_socket
        pncLibrary.printStringToTerminalMessageQueue(synchronizer.q_print_server_message_queue, machine.connection_string, machine.name, control_client_ip, control_client_port)
        return True
    except socket.error:
        #Set up dummy socket to not crash other processes
        machine.rsh_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        pncLibrary.printStringToTerminalMessageQueue(synchronizer.q_print_server_message_queue, machine.failed_connection_string, machine.name, control_client_ip, control_client_port)
        return False

def appInit():
    registerProxy('MachineModel', MachineModel, MachineModelProxy, PNCAppManager)

    # FIXME ignore SIGINT
    pnc_app_manager = PNCAppManager()
    pnc_app_manager.name = 'pnc_app_manager'
    pnc_app_manager.start()

    machine = pnc_app_manager.MachineModel()
    synchronizer = pncLibrary.Synchronizer(pnc_app_manager)
    synchronizer.main_process_name = str(multiprocessing.current_process().name)
    synchronizer.main_process_pid = str(multiprocessing.current_process().pid)
    pncLibrary.setTaskRunFlags(synchronizer)
    #machine.sculptprint_interface = pncLibrary.SculptPrintInterface()
    machine.local_epoch = time.clock()

    pncLibrary.printTerminalString(machine.process_launch_string, pnc_app_manager.name, pnc_app_manager._process.pid,
                                   synchronizer.main_process_name, synchronizer.main_process_pid)
    synchronizer.mvc_pncApp_initialized_event.set()

    return pnc_app_manager, machine, synchronizer

def appStart(type, pnc_app_manager, machine, synchronizer):
    pncLibrary.setTaskRunFlags(synchronizer)
    synchronizer.mvc_run_feedback_event.set()
    #global pnc_app_manager, machine, synchronizer
    # #FIXME ignore SIGINT?

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

    #main_process_name = str(multiprocessing.current_process().name)
    #main_process_pid = str(multiprocessing.current_process().pid)
    #test = testProcess(machine, synchronizer)
    #test.start()

    database, db_pipe_sender = startProcess(machine, synchronizer, DatabaseServer)
    encoder_interface, ei_pipe_sender = startProcess(machine, synchronizer, EncoderInterface)
    feedback_handler, fb_pipe_sender = startProcess(machine, synchronizer, MachineFeedbackHandler)
    machine_controller, mc_pipe_sender = startProcess(machine, synchronizer, MachineController)

    pnc_app_manager.machine = machine
    pnc_app_manager.database = database
    pnc_app_manager.encoder_interface = encoder_interface
    pnc_app_manager.machine_controller = machine_controller
    pnc_app_manager.feedback_handler = feedback_handler

    if openNetworkConnection(machine, synchronizer, control_client_ip, control_client_port):
        waitForAppStart(synchronizer)
        #pncLibrary.printTerminalString(machine.pncApp_launch_string, multiprocessing.cpu_count())
        synchronizer.mc_socket_connected_event.set()
        signalProcessWake(synchronizer)
        if checkSuccessfulStart(pnc_app_manager, synchronizer):
            pncLibrary.printTerminalString(machine.pncApp_launch_string, multiprocessing.cpu_count())
        else:
            pncLibrary.printTerminalString(machine.pncApp_launch_failure_string, multiprocessing.cpu_count())
            return appClose(pnc_app_manager, synchronizer)
    else:
        waitForAppStart(synchronizer)
        pncLibrary.printTerminalString(machine.pncApp_launch_failure_string, multiprocessing.cpu_count())
        signalProcessWake(synchronizer)
        return appClose(pnc_app_manager, synchronizer)

    #pncLibrary.printTerminalString(machine.pncApp_launch_string, multiprocessing.cpu_count())
    synchronizer.mvc_pncApp_started_event.set()
    closePipes(db_pipe_sender, ei_pipe_sender, mc_pipe_sender, fb_pipe_sender)

    return database, encoder_interface, feedback_handler, machine_controller

def appStop(pnc_app_manager):
    #FIXME just pause activity, don't actually shut down
    pnc_app_manager.synchronizer.mvc_run_feedback_event.clear()
    # FIXME unflag run_machine_controlled event?

    pass

def appClose(pnc_app_manager, synchronizer):
    #FIXME need timeout and force terminate
    closeProcess(pnc_app_manager, synchronizer, "encoder_interface")
    closeProcess(pnc_app_manager, synchronizer, "machine_controller")
    closeProcess(pnc_app_manager, synchronizer, "feedback_handler")
    closeProcess(pnc_app_manager, synchronizer, "database")
    closeRSHSocket(pnc_app_manager, synchronizer)
    #FIXME close app manager
    pncLibrary.printTerminalString(pnc_app_manager.machine.pncApp_terminate_string)
    synchronizer.mvc_app_shutdown_event.set()
    #pnc_app_manager.shutdown()
    #pnc_app_manager.machine_controller.join()

def checkSuccessfulStart(pnc_app_manager, synchronizer):
    try:
        synchronizer.db_successful_start_event.wait(pnc_app_manager.machine.event_wait_timeout)
        synchronizer.ei_successful_start_event.wait(pnc_app_manager.machine.event_wait_timeout)
        synchronizer.fb_successful_start_event.wait(pnc_app_manager.machine.event_wait_timeout)
        synchronizer.mc_successful_start_event.wait(pnc_app_manager.machine.event_wait_timeout)
        return True
    except:
        return False

def waitForAppStart(synchronizer):
    synchronizer.db_startup_event.wait()
    synchronizer.ei_startup_event.wait()
    synchronizer.fb_startup_event.wait()
    synchronizer.mc_startup_event.wait()

def signalProcessWake(synchronizer):
    synchronizer.process_start_signal.set()

def startProcess(machine, synchronizer, process_class):
    pipe_sender, pipe_receiver = multiprocessing.Pipe()
    process = process_class(machine, pipe_receiver)
    process.start()
    pncLibrary.setSynchronizer(pipe_sender, synchronizer)
    pncLibrary.printTerminalString(machine.process_launch_string, process.name, process.pid,
                                   synchronizer.main_process_name, synchronizer.main_process_pid)
    return process, pipe_sender

def closeProcess(pnc_app_manager, synchronizer, process):
    getattr(synchronizer, "p_run_"+process+"_event").clear()
    try:
        getattr(pnc_app_manager, process).join(pnc_app_manager.machine.join_timeout)
        pncLibrary.printTerminalString(pnc_app_manager.machine.process_terminate_string,
                                       getattr(pnc_app_manager, process).name, getattr(pnc_app_manager, process).pid)
    except:
        getattr(pnc_app_manager, process).terminate()
        getattr(pnc_app_manager, process).join()
        pncLibrary.printTerminalString(pnc_app_manager.machine.process_force_terminate_string,
                                       getattr(pnc_app_manager, process).name, getattr(pnc_app_manager, process).pid)

def closePipes(*args):
    for pipe in args:
        pipe.close()

def closeRSHSocket(pnc_app_manager, synchronizer):
    if synchronizer.mc_socket_connected_event.is_set():
        pnc_app_manager.machine.rsh_socket.shutdown(socket.SHUT_RDWR)
    pnc_app_manager.machine.rsh_socket.close()
    synchronizer.mc_socket_connected_event.clear()
    pncLibrary.printStringToTerminalMessageQueue(synchronizer.q_print_server_message_queue,
                                                 pnc_app_manager.machine.connection_close_string,
                                                 pnc_app_manager.machine.name,
                                                 pnc_app_manager.machine.ip_address + ':' + str(
                                                     pnc_app_manager.machine.tcp_port))
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

if __name__ == '__main__' and 0:
    pnc_app_manager, machine, synchronizer = appInit()
    database, encoder_interface, feedback_handler, machine_controller = appStart('pocketnc',pnc_app_manager,machine,synchronizer)

    #appStart('pocketnc')
#     appStart()
#     pnc_app_manager, machine, database, encoder_interface, machine_controller, feedback_handler, synchronizer = appInit('pocketnc')
#     #Store process handles in pnc_app_manager?
#     synchronizer.mvc_connect_event.set()
#     machine_controller.join()
#     #time.sleep(1)
#     appClose()