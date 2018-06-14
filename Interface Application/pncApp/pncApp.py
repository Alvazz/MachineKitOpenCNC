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
    machine.sculptprint_interface = pncLibrary.SculptPrintInterface()
    machine.local_epoch = time.clock()

    pncLibrary.printTerminalString(machine.process_launch_string, pnc_app_manager.name, pnc_app_manager._process.pid,
                                   synchronizer.main_process_name, synchronizer.main_process_pid)
    synchronizer.mvc_pncApp_initialized_event.set()

    return pnc_app_manager, machine, synchronizer

def appStart(type, pnc_app_manager, machine, synchronizer):
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

    database = DatabaseServer(machine, synchronizer)
    database.start()
    pncLibrary.printTerminalString(machine.process_launch_string, database.name, database.pid, synchronizer.main_process_name, synchronizer.main_process_pid)

    encoder_interface = EncoderInterface(machine, synchronizer)
    encoder_interface.start()
    pncLibrary.printTerminalString(machine.process_launch_string, encoder_interface.name, encoder_interface.pid, synchronizer.main_process_name, synchronizer.main_process_pid)

    feedback_handler = MachineFeedbackHandler(machine, synchronizer)
    feedback_handler.start()
    pncLibrary.printTerminalString(machine.process_launch_string, feedback_handler.name, feedback_handler.pid, synchronizer.main_process_name, synchronizer.main_process_pid)

    machine_controller = MachineController(machine, synchronizer)
    machine_controller.start()
    pncLibrary.printTerminalString(machine.process_launch_string, machine_controller.name, machine_controller.pid, synchronizer.main_process_name, synchronizer.main_process_pid)

    openNetworkConnection(machine, control_client_ip, control_client_port)
    waitForAppStart(synchronizer)

    pnc_app_manager.machine = machine
    pnc_app_manager.database = database
    pnc_app_manager.encoder_interface = encoder_interface
    pnc_app_manager.machine_controller = machine_controller
    pnc_app_manager.feedback_handler = feedback_handler

    signalProcessWake(synchronizer)
    pncLibrary.printTerminalString(machine.pncApp_launch_string, multiprocessing.cpu_count())
    synchronizer.mvc_pncApp_started_event.set()

    return database, encoder_interface, feedback_handler, machine_controller

def appStop(pnc_app_manager):
    #FIXME just pause activity, don't actually shut down
    pnc_app_manager.synchronizer.mvc_run_feedback_event.clear()
    # FIXME unflag run_machine_controlled event?

    pass

def appClose(pnc_app_manager, synchronizer):
    #FIXME need timeout and force terminate
    #print('pnc_app_manager is ' + str(pnc_app_manager))
    #pnc_app_manager.machine.run_machine_controller = False

    # synchronizer.p_run_encoder_interface_event.clear()
    # try:
    #     pnc_app_manager.encoder_interface.join(pnc_app_manager.machine.join_timeout)
    #     pncLibrary.printTerminalString(pnc_app_manager.machine.process_terminate_string,
    #                                pnc_app_manager.encoder_interface.name, pnc_app_manager.encoder_interface.pid)
    # except:
    #     pnc_app_manager.encoder_interface.terminate()
    #     pncLibrary.printTerminalString(pnc_app_manager.machine.process_force_terminate_string,
    #                                    pnc_app_manager.encoder_interface.name, pnc_app_manager.encoder_interface.pid)

    closeProcess(pnc_app_manager, synchronizer, "encoder_interface")
    closeProcess(pnc_app_manager, synchronizer, "machine_controller")
    closeProcess(pnc_app_manager, synchronizer, "feedback_handler")
    closeProcess(pnc_app_manager, synchronizer, "database")
    closeRSHSocket(pnc_app_manager, synchronizer)

    # synchronizer.p_run_machine_controller_event.clear()
    # synchronizer.p_enable_machine_controller_event.clear()
    # pnc_app_manager.machine_controller.join()
    # pncLibrary.printTerminalString(pnc_app_manager.machine.process_terminate_string,
    #                                pnc_app_manager.machine_controller.name, pnc_app_manager.machine_controller.pid)
    #
    # synchronizer.p_run_feedback_handler_event.clear()
    # pnc_app_manager.feedback_handler.join()
    # pncLibrary.printTerminalString(pnc_app_manager.machine.process_terminate_string,
    #                                pnc_app_manager.feedback_handler.name, pnc_app_manager.feedback_handler.pid)
    #
    # pnc_app_manager.machine.rsh_socket.shutdown(socket.SHUT_RDWR)
    # pnc_app_manager.machine.rsh_socket.close()
    # pncLibrary.printStringToTerminalMessageQueue(synchronizer.q_print_server_message_queue,
    #                                              pnc_app_manager.machine.connection_close_string, pnc_app_manager.machine.name,
    #                                              pnc_app_manager.machine.ip_address + ':' + str(pnc_app_manager.machine.tcp_port))
    #
    # synchronizer.p_run_database_event.clear()
    # pnc_app_manager.database.join()
    # pncLibrary.printTerminalString(pnc_app_manager.machine.process_terminate_string,
    #                                pnc_app_manager.database.name, pnc_app_manager.database.pid)


    pncLibrary.printTerminalString(pnc_app_manager.machine.pncApp_terminate_string, multiprocessing.cpu_count())
    synchronizer.mvc_app_shutdown_event.set()
    #pnc_app_manager.machine_controller.join()

def waitForAppStart(synchronizer):
    synchronizer.db_startup_event.wait()
    synchronizer.ei_startup_event.wait()
    synchronizer.fb_startup_event.wait()
    synchronizer.mc_startup_event.wait()

def signalProcessWake(synchronizer):
    synchronizer.process_start_signal.set()

def closeProcess(pnc_app_manager, synchronizer, process):
    getattr(synchronizer, "p_run_"+process+"_event").clear()
    try:
        getattr(pnc_app_manager, process).join(pnc_app_manager.machine.join_timeout)
        pncLibrary.printTerminalString(pnc_app_manager.machine.process_terminate_string,
                                       getattr(pnc_app_manager, process).name, getattr(pnc_app_manager, process).pid)
    except:
        getattr(pnc_app_manager, process).terminate()
        pncLibrary.printTerminalString(pnc_app_manager.machine.process_force_terminate_string,
                                       getattr(pnc_app_manager, process).name, getattr(pnc_app_manager, process).pid)

def closeRSHSocket(pnc_app_manager, synchronizer):
    pnc_app_manager.machine.rsh_socket.shutdown(socket.SHUT_RDWR)
    pnc_app_manager.machine.rsh_socket.close()
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

# if __name__ == '__main__':
#     appInit()
#     appStart()
#     pnc_app_manager, machine, database, encoder_interface, machine_controller, feedback_handler, synchronizer = appInit('pocketnc')
#     #Store process handles in pnc_app_manager?
#     synchronizer.mvc_connect_event.set()
#     machine_controller.join()
#     #time.sleep(1)
#     appClose()