import pncLibrary, socket, os, pickle, wpipe, select#socketserver
from multiprocessing import Event, Queue, Pipe, current_process
from queue import Empty
from pncApp import *#appInit, appStart, appStop, appClose
from pncMachineControl import OperatingSystemController
import numpy as np

pncLibrary.updatePath()
sculptprint_MVC = None

class CAM_MVC(Thread):
    def __init__(self):
        super(CAM_MVC, self).__init__()
        self.name = "pncApp_controller"
        self.machine_type = "pocketnc"
        self.main_thread_name = self.name + ".MainThread"
        self.feedback_state = pncLibrary.SculptPrintFeedbackState()

        self.command_queue = Queue()
        self.startup_event = Event()
        self.t_run_CAM_MVC_thread = Event()
        self.pncApp_initialized_event = Event()
        self.pncApp_started_event = Event()
        self.feedback_fast_forward_event = Event()

        self.main_process_name = str(current_process().name)
        self.main_process_pid = str(current_process().pid)

        self.mvc_pipe = wpipe.Server(pncLibrary.socket_sculptprint_ipc_pipe_name, wpipe.Mode.Slave)

        self.pnc_app_manager = None
        self.machine = None
        self.synchronizer = None
        self.terminal_printer = None

    def run(self):
        print('PNCAPP CONTROLLER: Starting...')
        #current_process().name = self.name
        pncLibrary.waitForThreadStart(self, PNCAppInterfaceSocketLauncher)
        self.startup_event.set()
        while self.t_run_CAM_MVC_thread.is_set():
            try:
                command = self.command_queue.get(pncLibrary.queue_wait_timeout)
                self.handleCommand(command)
            except TimeoutError:
                print('queue wait timed out')
            except ConnectionResetError:
                print('PNCAPP CONTROLLER: Connection closed during transfer')
            except Empty:
                pass
            # except ConnectionResetError:
            #     #FIXME move to socket manager thread?
            #     print('SCULPTPRINT MVC: Client %s forcibly disconnected during transfer' % str(self.CAM_client_address))

        self.mvc_pipe.close()
        pncLibrary.waitForThreadStop(self, self.interface_socket_launcher)
        #pncLibrary.printTerminalString(self.machine.thread_launch_string, current_process().name, self.name)

    def returnAck(self, connection_type, connection):
        if connection is not None:
            pncLibrary.sendIPCData(connection_type, 'string', connection, 'ACK')

    def handleCommand(self, command):
        print(command.command.split('_')[0].strip())
        command_data = command.command.split('_')
        command_type = command_data[0].strip()
        if command_type == 'INIT':
            self.init_pncApp()
        elif command_type == 'CHECKSTATUS':
            if self.pncApp_initialized_event.is_set():
                pncLibrary.sendToSP(self.mvc_socket, self.synchronizer.mvc_pncApp_started_event.is_set())
            else:
                pncLibrary.sendToSP(self.mvc_socket, False)
        elif command_type == 'UN_INIT':
            self.uninit_pncApp()
        elif command_type == 'START':
            self.synchronizer.mvc_pncApp_initialized_event.wait()
            try:
                self.start_pncApp()
            except Exception as error:
                print("SCULPTPRINT MVC: Could not launch pncApp, error " + str(error))
                self.command_queue = Queue()
        elif command_type == 'CLOSE':
            #FIXME only after connect to guarantee encoder sync?
            self.synchronizer.mvc_pncApp_started_event.wait()
            self.close_pncApp()
        elif command_type == 'CONNECT':
            try:
                self.synchronizer.mvc_pncApp_started_event.wait(self.machine.event_wait_timeout)
                self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('CONNECT', None))
            except Exception as error:
                print("SCULPTPRINT MVC: Connection problem, error " + str(error))
        elif command_type == 'ENQUEUE':
            self.synchronizer.mvc_connect_event.wait()
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('ENQUEUE', (int(command_type[1]), int(command_type[2]))))
            #self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('ENQUEUE', None))
        elif command_type == 'EXECUTE':
            self.synchronizer.mvc_connect_event.wait()
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('EXECUTE', None))
        elif command_type == 'ISMONITORING':
            #self.sculptprint_socket.sendto(pickle.dumps(pncLibrary.SP.isMonitoring(self.synchronizer)), ('127.0.0.1', 42069))
            #pncLibrary.sendToSP('pipe', self.mvc_pipe, pncLibrary.SP.isMonitoring(self.synchronizer))
            pncLibrary.sendIPCData(command.connection_type, command.connection_format, command.connection, pncLibrary.SP.isMonitoring(self.synchronizer))
        elif command_type == 'READ':
            #pncLibrary.sendIPCData(self.mvc_pipe.clients[0], 'abcd')
            #pncLibrary.sendIPCData(self.mvc_pipe.clients[0],np.random.rand(200,200))
            pncLibrary.sendIPCData(command.connection_type, command.connection_format, command.connection, pncLibrary.SP.readMachine(self.synchronizer, self.feedback_state, int(command_data[1])))
            # try:
            #     #data = pncLibrary.SP.readMachine(self.machine, self.synchronizer, self.feedback_state, int(command_type[1]))
            #     pncLibrary.sendIPCData(command.connection_type, command.connection_format, command.connection, pncLibrary.SP.readMachine(self.machine, self.synchronizer, self.feedback_state, int(command_data[1])))
            # except Exception as error:
            #     print('break')
        elif command_type == 'FASTFORWARD':
            pncLibrary.updateInterfaceData('touch', self.synchronizer, self.feedback_state, pncLibrary.SP_main_data_streams, pncLibrary.SP_auxiliary_data_streams, [int(id) for id in command_data[1:]])
            pncLibrary.updateInterfaceClockOffsets(self.feedback_state.last_values_read, self.feedback_state.clock_offsets)
            #if command.connection_format != 'internal'
            pncLibrary.sendIPCAck(command.connection_type, command.connection_format, command.connection, command_type)
        elif command_type == 'CLOSE_SP_INTERFACE':
            pncLibrary.sendIPCData(self.CAM_socket_listener.client_type, self.CAM_client, command_type + '_ACK')
            self.CAM_socket_listener.client_connected_event.clear()
            print('SCULPTPRINT MVC: Client %s cleanly disconnected' % str(self.CAM_socket_listener.CAM_client_address))

    def init_pncApp(self):
        self.pnc_app_manager, self.machine, self.synchronizer = appInit()
        self.terminal_printer = pncLibrary.startPrintServer(self.machine, self.synchronizer)

        pncLibrary.waitForThreadStart(self, OperatingSystemController)
        self.operating_system_controller.command_queue.put(pncLibrary.OSCommand('CHECK_RSH_RUN'))
        print('RSH running: ' + str(self.synchronizer.os_linuxcncrsh_running_event.is_set()))

        self.pncApp_initialized_event.set()
        #self.synchronizer.mvc_pncApp_initialized_event.set()

    def uninit_pncApp(self):
        pass

    def start_pncApp(self):
        #pncLibrary.printTerminalString(self.machine.sculptprint_interface_initialization_string, self.main_process_name,self.main_process_pid)
        self.database, self.encoder_interface, self.feedback_handler, self.machine_controller = appStart(self.machine_type, self.pnc_app_manager,
                                                                                     self.machine, self.synchronizer)

    def stop_pncApp(self):
        appStop(self.pnc_app_manager, self.synchronizer)

    def close_pncApp(self):
        # FIXME do something about MVC still running
        appClose(self.pnc_app_manager, self.synchronizer)
        print('closed')

class PNCAppInterfaceSocketLauncher(Thread):
    def __init__(self, parent):
        super(PNCAppInterfaceSocketLauncher, self).__init__()
        self.name = "interface_socket_launcher"
        self.parent = parent
        self.pncApp_command_queue = self.parent.command_queue

        self.interface_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
        #self.interface_socket.setsockopt(socket.SIO_LOOPBACK_FAST_PATH)
        self.interface_socket.settimeout(pncLibrary.socket_wait_timeout)

        self.client_list = []

        self.t_run_interface_socket_launcher_event = Event()
        self.startup_event = Event()

        self.t_run_interface_socket_launcher_event.set()

    def run(self):
        self.interface_socket.bind(('localhost', pncLibrary.socket_interface_socket_port))
        self.interface_socket.listen()
        pncLibrary.printTerminalString(pncLibrary.printout_thread_launch_string, current_process().name, self.name)
        self.startup_event.set()
        while self.t_run_interface_socket_launcher_event.is_set():
            try:
                (client_connection, client_address) = self.interface_socket.accept()
                self.pncApp_command_queue.put(pncLibrary.PNCAppCommand('FASTFORWARD_0_1', None, client_connection, 'socket', 'internal'))
                self.launchClient(client_connection, client_address)
            except socket.timeout:
                pass

        self.terminateClients()

        self.interface_socket.shutdown()
        self.interface_socket.close()

    def launchClient(self, connection, address):
        client = PNCAppInterfaceSocket(self, connection, address)
        client.client_connected_event.set()
        client.start()
        client.startup_event.wait()
        self.client_list.append(client)

    def terminateClients(self):
        for client in self.client_list:
            client.terminate()

class PNCAppInterfaceSocket(Thread):
    def __init__(self, parent, connection, address):
        super(PNCAppInterfaceSocket, self).__init__()
        self.name = "interface_socket"
        self.client_name = None
        self.connection_type = 'socket'
        self.client_address = address
        self.parent = parent
        self.connection = connection

        self.client_connected_event = Event()
        self.login_event = Event()
        self.startup_event = Event()

    def run(self):
        self.startup_event.set()
        while self.client_connected_event.is_set():
            try:
                if not self.login_event.is_set():
                    self.waitForSocketLogin()
                    pncLibrary.printTerminalString(pncLibrary.printout_interface_socket_connection_string, self.client_name,
                                                   self.client_address)
                inbound_message = pncLibrary.receiveIPCData(self.connection_type, 'string', self.connection)
                if inbound_message.strip() == 'CLOSE_SOCKET':
                    raise EOFError

                self.parent.pncApp_command_queue.put(pncLibrary.PNCAppCommand(inbound_message, None, self.connection, self.connection_type, self.connection_format))
            except (ConnectionAbortedError, ConnectionResetError):
                self.client_connected_event.clear()
                print('INTERFACE SOCKET: Client %s forcibly disconnected' % str(self.client_address))
                break
            except EOFError:
                self.client_connected_event.clear()
                print('INTERFACE SOCKET: Client %s cleanly disconnected' % str(self.client_address))
                break
            except TimeoutError:
                pass
                #print('Connection timed out')

        self.parent.client_list.pop(self.parent.client_list.index(self))

    def waitForSocketLogin(self):
        login_message = pncLibrary.receiveIPCData(self.connection_type, 'string', self.connection)
        try:
            login_command, name, format = login_message.split('_')
            if login_command == 'HELLO':
                self.client_name = name.strip()
                self.connection_format = format.lower().strip()
                pncLibrary.sendIPCData(self.connection_type, 'string', self.connection, 'HELLO_' + self.client_name + '_' + self.connection_format.upper() + '_ACK')
                self.login_event.set()
        except Exception as error:
            print("I dont know what error type this is, so you should fill it in when this error inevitably happens: " + str(error))

def startMVC():
    sculptprint_MVC = CAM_MVC()
    #sculptprint_MVC.feedback_state = pncLibrary.SculptPrintFeedbackState()
    sculptprint_MVC.t_run_CAM_MVC_thread.set()
    sculptprint_MVC.start()
    sculptprint_MVC.startup_event.wait()

    return sculptprint_MVC

def startPncApp():
    global sculptprint_MVC, machine, synchronizer, terminal_printer, feedback_state
    if sculptprint_MVC is None:
        sculptprint_MVC = startMVC()

        sculptprint_MVC.command_queue.put(pncLibrary.PNCAppCommand('INIT', None, None, None, 'internal'))
        sculptprint_MVC.pncApp_initialized_event.wait()

        machine = sculptprint_MVC.machine
        synchronizer = sculptprint_MVC.synchronizer
        terminal_printer = sculptprint_MVC.terminal_printer
        feedback_state = sculptprint_MVC.feedback_state


    sculptprint_MVC.command_queue.put(pncLibrary.PNCAppCommand('START', None, None, None, 'internal'))
    pncLibrary.printTerminalString(pncLibrary.printout_sculptprint_interface_initialization_string, sculptprint_MVC.name, sculptprint_MVC.main_process_name, sculptprint_MVC.main_process_pid)
    print('PNCAPP CONTROLLER: Waiting for startup...')
    synchronizer.mvc_pncApp_started_event.wait(machine.max_mvc_startup_wait_time)
    return True

if __name__ == '__main__':
    #multiprocessing.freeze_support()
    #multiprocessing.log_to_stderr(logging.ERROR)
    #start()

    startPncApp()
    sculptprint_MVC.command_queue.put(pncLibrary.PNCAppCommand('CONNECT', None, None, None, 'internal'))

    #sculptprint_MVC.join()
    if 0:
        sculptprint_MVC = startMVC()
        sculptprint_MVC.command_queue.put(pncLibrary.PNCAppCommand('INIT', None, None, None, 'binary'))
        sculptprint_MVC.pncApp_initialized_event.wait()
        # #
        machine = sculptprint_MVC.machine
        synchronizer = sculptprint_MVC.synchronizer
        terminal_printer = sculptprint_MVC.terminal_printer
        feedback_state = sculptprint_MVC.feedback_state

    # sculptprint_MVC = startMVC()
    # machine = sculptprint_MVC.machine
    # synchronizer = sculptprint_MVC.synchronizer
    # terminal_printer = sculptprint_MVC.terminal_printer
    # feedback_state = sculptprint_MVC.feedback_state
    # sculptprint_MVC.init_pncApp()
    # sculptprint_MVC.start_pncApp()

    #connectToMachine()
    #userPythonFunction1(0,0,0,0,0)
    #userPythonFunction2(1,5,0,0,0)
    #userPythonFunction3(0,0,0,0,0)
    #
    # while True:
    #     print(eval(input("command: ")))
    #     print('looping')