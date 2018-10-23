import pncLibrary, socket, os, pickle, wpipe, select#socketserver
from multiprocessing import Event, Queue, Pipe, current_process
from queue import Empty
from pncApp import *#appInit, appStart, appStop, appClose
from pncMachineControl import OperatingSystemController
import numpy as np

pncLibrary.updatePath()
pncApp_controller = None

class PNCAppController(Thread):
    def __init__(self):
        super(PNCAppController, self).__init__()
        self.name = "pncApp_controller"
        self.machine_type = "pocketnc"
        self.main_thread_name = self.name + ".MainThread"
        self.feedback_state = pncLibrary.SculptPrintFeedbackState()
        # self.feedback_state.SP_axis_data_source_format = pncLibrary.SP.buildAxisDataSourceArray()
        # self.feedback_state.SP_auxiliary_data_source_format = pncLibrary.SP.buildAuxiliaryDataSourceArray()


        self.command_queue = Queue()
        self.startup_event = Event()
        self.t_run_pncApp_controller_thread = Event()
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
        while self.t_run_pncApp_controller_thread.is_set():
            try:
                command = self.command_queue.get(pncLibrary.queue_wait_timeout)
                self.handleCommand(command)
            except TimeoutError:
                print('queue wait timed out')
            except ConnectionResetError:
                print('PNCAPP CONTROLLER: Connection closed during transfer')
            except Empty:
                pass

        self.mvc_pipe.close()
        pncLibrary.waitForThreadStop(self, self.interface_socket_launcher)

    # def returnAck(self, connection_type, connection):
    #     if connection is not None:
    #         pncLibrary.sendIPCData(connection_type, 'text', connection, 'ACK')

    def handleCommand(self, command):
        print('PNCAPP CONTROLLER: Received ' + command.command)
        #command_data = command.command.split('_')
        #command_type = command.command
        if command.command == 'INIT':
            self.init_pncApp()
        elif command.command == 'CHECKSTATUS':
            if self.pncApp_initialized_event.is_set():
                pncLibrary.sendToSP(self.mvc_socket, self.synchronizer.mvc_pncApp_started_event.is_set())
            else:
                pncLibrary.sendToSP(self.mvc_socket, False)
        elif command.command == 'UN_INIT':
            self.uninit_pncApp()
        elif command.command == 'START':
            self.synchronizer.mvc_pncApp_initialized_event.wait()
            try:
                self.start_pncApp()
            except Exception as error:
                print("PNCAPP INTERFACE: Could not launch pncApp, error " + str(error))
                self.command_queue = Queue()
        elif command.command == 'CONNECT':
            try:
                self.synchronizer.mvc_pncApp_started_event.wait(self.machine.event_wait_timeout)
                self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('CONNECT', None))
                pncLibrary.sendIPCAck(command.connection_type, command.connection_format, command.connection,command.command)
            except Exception as error:
                print("SCULPTPRINT MVC: Connection problem, error " + str(error))
        elif command.command == 'LOAD':
            self.synchronizer.mvc_connect_event.wait()
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('LOAD', [int(data) for data in command.command_data]))
            pncLibrary.sendIPCAck(command.connection_type, command.connection_format, command.connection, command.command)
        elif command.command == 'ENQUEUE':
            self.synchronizer.mvc_connect_event.wait()
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('ENQUEUE', [int(data) for data in command.command_data]))
            pncLibrary.sendIPCAck(command.connection_type, command.connection_format, command.connection, command.command)
        elif command.command == 'TRAPENQUEUE':
            self.synchronizer.mvc_connect_event.wait()
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('ENQUEUE_TRAPEZOID', [int(data) for data in command.command_data]))
            pncLibrary.sendIPCAck(command.connection_type, command.connection_format, command.connection, command.command)
        elif command.command == 'VOXELENQUEUE':
            self.synchronizer.mvc_connect_event.wait()
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('ENQUEUE_VOXELIZED', [int(data) for data in command.command_data]))
            pncLibrary.sendIPCAck(command.connection_type, command.connection_format, command.connection, command.command)
        elif command.command == 'EXECUTE':
            self.synchronizer.mvc_connect_event.wait()
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('EXECUTE', None))
            pncLibrary.sendIPCAck(command.connection_type, command.connection_format, command.connection, command.command)
        elif command.command == 'PLAN':
            self.synchronizer.mvc_connect_event.wait()
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('PLAN', [int(data) for data in command.command_data]))
            pncLibrary.sendIPCAck(command.connection_type, command.connection_format, command.connection, command.command)
        elif command.command == 'ISMONITORING':
            pncLibrary.sendIPCData(command.connection_type, command.connection_format, command.connection, pncLibrary.SP.isMonitoring(self.synchronizer))
        elif command.command == 'READ1':
            pncLibrary.sendIPCData(command.connection_type, command.connection_format, command.connection, pncLibrary.SP.readMachine(self.synchronizer, self.feedback_state, int(command.command_data[0])))
        elif command.command == 'READ':
            pncLibrary.sendIPCData(command.connection_type, command.connection_format, command.connection, pncLibrary.SP.read(self.synchronizer, self.feedback_state))
        # elif command.command == 'SETUPDATA':
        #     print('Setting up data format for SP')
        #     self.machine.SP_axis_data_source_formats = pncLibrary.SP.buildAxisDataSourceArray()
        #     self.machine.SP_auxiliary_data_source_formats = pncLibrary.SP.buildAuxiliaryDataSourceArray()
        elif command.command == 'FASTFORWARD':
            pncLibrary.updateInterfaceClockOffsets(self.machine, self.feedback_state.clock_offsets)
            #pncLibrary.updateInterfaceData('touch', self.synchronizer, self.feedback_state, pncLibrary.SP_main_data_streams, pncLibrary.SP_auxiliary_data_streams, command.command_data)
            pncLibrary.updateFullInterfaceData('touch', self.synchronizer, self.feedback_state, pncLibrary.SP_main_data_streams, pncLibrary.SP_auxiliary_data_streams)
            pncLibrary.sendIPCAck(command.connection_type, command.connection_format, command.connection, command.command)
        elif command.command == 'SETUPTOOLPATH':
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('SETUPTOOLPATH', command.command_data))
            pncLibrary.sendIPCAck(command.connection_type, command.connection_format, command.connection, command.command)
        elif command.command == 'CHECKPOINTREQUEST':
            #self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('CHECKPOINTREQUESTS', command.command_data))
            pncLibrary.sendIPCData(command.connection_type, command.connection_format, command.connection, pncLibrary.SP.getPointRequest(self.synchronizer))
        elif command.command == 'UPDATETOOLPATHPOINTS':
            self.synchronizer.tp_need_points_event.clear()
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('UPDATETOOLPATHPOINTS', command.command_data))
            pncLibrary.sendIPCAck(command.connection_type, command.connection_format, command.connection, command.command)
        elif command.command == 'CLOSESPINTERFACE':
            pncLibrary.sendIPCData(self.CAM_socket_listener.client_type, self.CAM_client, command.command + '_ACK')
            self.CAM_socket_listener.client_connected_event.clear()
            print('SCULPTPRINT MVC: Client %s cleanly disconnected' % str(self.CAM_socket_listener.CAM_client_address))


    def init_pncApp(self):
        self.pnc_app_manager, self.machine, self.synchronizer = appInit()
        self.terminal_printer = pncLibrary.startPrintServer(self.machine, self.synchronizer)

        pncLibrary.waitForThreadStart(self, OperatingSystemController)
        linuxcnc_RSH_status = self.getRSHStatus(pncLibrary.ssh_wait_timeout)
        #print('RSH status is: ' + str(linuxcnc_RSH_status))
        #self.operating_system_controller.command_queue.put(pncLibrary.OSCommand('CHECK_RSH_RUN'))
        #print('RSH running: ' + str(self.synchronizer.os_linuxcncrsh_running_event.is_set()))

        if linuxcnc_RSH_status[1]:
            pncLibrary.printTerminalString(pncLibrary.printout_ssh_process_ready_string, 'linuxcncrsh')
            self.pncApp_initialized_event.set()
        else:
            pncLibrary.printTerminalString(pncLibrary.printout_ssh_bad_state_string, 'linuxcncrsh')

    def uninit_pncApp(self):
        pass

    def start_pncApp(self):
        self.database, self.encoder_interface, self.feedback_handler, self.machine_controller = appStart(self.machine_type, self.pnc_app_manager,
                                                                                     self.machine, self.synchronizer)

    def stop_pncApp(self):
        appStop(self.pnc_app_manager, self.synchronizer)

    def close_pncApp(self):
        # FIXME do something about MVC still running
        appClose(self.pnc_app_manager, self.synchronizer)
        print('closed')


    #################### OS INTERFACE ####################
    def getRSHStatus(self, timeout = 0.5):
        self.synchronizer.os_linuxcncrsh_running_event.clear()
        self.operating_system_controller.command_queue.put(pncLibrary.OSCommand('CHECK_RSH_RUN'))
        if self.synchronizer.os_linuxcncrsh_running_event.wait(timeout):
            return (True, self.synchronizer.os_linuxcncrsh_running_event.is_set())
        else:
            return (False, self.synchronizer.os_linuxcncrsh_running_event.is_set())

class PNCAppInterfaceSocketLauncher(Thread):
    def __init__(self, parent):
        super(PNCAppInterfaceSocketLauncher, self).__init__()
        self.name = "interface_socket_launcher"
        self.parent = parent
        self.pncApp_command_queue = self.parent.command_queue

        self.interface_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
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
                if not self.parent.pncApp_initialized_event.is_set():
                    self.parent.pncApp_initialized_event.wait()
                    if not self.parent.synchronizer.mvc_pncApp_started_event.is_set():
                        print("PNCAPP CONTROLLER: Waiting for pncApp startup before launching client")
                        self.parent.synchronizer.mvc_pncApp_started_event.wait()

                self.pncApp_command_queue.put(pncLibrary.PNCAppCommand('FASTFORWARD', [0, 1], client_connection, 'socket', 'internal'))
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
        self.connection_format = None

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
                #inbound_message = pncLibrary.receiveIPCData(self.connection_type, 'text', self.connection)
                inbound_message = pncLibrary.receiveIPCData(self.connection_type, self.connection_format, self.connection)
                #Gross hack hack
                if type(inbound_message) is str:
                    if inbound_message.strip() == 'CLOSESOCKET':
                        pncLibrary.sendIPCAck(self.connection_type, self.connection_format, self.connection, inbound_message)
                        raise EOFError
                    else:
                        command = pncLibrary.PNCAppCommand(inbound_message.strip().split('_')[0],
                                                           inbound_message.strip().split('_')[1:], self.connection,
                                                           self.connection_type, self.connection_format)
                elif type(inbound_message) is pncLibrary.IPCDataWrapper:
                    if inbound_message.data == 'CLOSESOCKET':
                        pncLibrary.sendIPCAck(self.connection_type, self.connection_format, self.connection, inbound_message.data)
                        raise EOFError
                    else:
                        command = pncLibrary.PNCAppCommand(inbound_message.data, inbound_message.payload, self.connection,
                                                           self.connection_type, self.connection_format)
                # if inbound_message is str and inbound_message.strip() == 'CLOSESOCKET' or inbound_message.data == 'CLOSESOCKET':
                #     pncLibrary.sendIPCAck(self.connection_type, self.connection_format, self.connection, inbound_message)
                #     raise EOFError
                #
                # command = pncLibrary.PNCAppCommand(inbound_message.strip().split('_')[0], inbound_message.strip().split('_')[1:], self.connection, self.connection_type, self.connection_format)
                try:
                    self.parent.pncApp_command_queue.put(command)
                except:
                    print('break')
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
        login_message = pncLibrary.receiveIPCData(self.connection_type, 'text', self.connection)
        try:
            login_command, name, format = login_message.split('_')
            if login_command == 'HELLO':
                self.client_name = name.strip()
                self.connection_format = format.lower().strip()
                pncLibrary.sendIPCData(self.connection_type, 'text', self.connection, 'HELLO_' + self.client_name + '_' + self.connection_format.upper() + '_ACK')
                self.login_event.set()
        except Exception as error:
            print("I dont know what error type this is, so you should fill it in when this error inevitably happens: " + str(error))

def startPncAppController():
    pncApp_controller = PNCAppController()
    pncApp_controller.t_run_pncApp_controller_thread.set()
    pncApp_controller.start()
    pncApp_controller.startup_event.wait()

    return pncApp_controller

def startPncApp():
    global pncApp_controller, machine, synchronizer, terminal_printer, feedback_state
    if pncApp_controller is None:
        pncApp_controller = startPncAppController()

        pncApp_controller.command_queue.put(pncLibrary.PNCAppCommand('INIT', None, None, None, 'internal'))
        if not pncApp_controller.pncApp_initialized_event.wait(pncLibrary.pncApp_init_wait_timeout):
            print('PNCAPP CONTROLLER: Initialization failed')
            return False

        machine = pncApp_controller.machine
        synchronizer = pncApp_controller.synchronizer
        terminal_printer = pncApp_controller.terminal_printer
        feedback_state = pncApp_controller.feedback_state

    pncApp_controller.command_queue.put(pncLibrary.PNCAppCommand('START', None, None, None, 'internal'))
    pncLibrary.printTerminalString(pncLibrary.printout_sculptprint_interface_initialization_string, pncApp_controller.name, pncApp_controller.main_process_name, pncApp_controller.main_process_pid)
    print('PNCAPP CONTROLLER: Waiting for startup...')
    try:
        synchronizer.mvc_pncApp_started_event.wait(machine.max_mvc_startup_wait_time)
        return True
    except Exception as error:
        print('Could not start pncApp, error: ' + str(error))
        return False

if __name__ == '__main__':
    #multiprocessing.freeze_support()
    #multiprocessing.log_to_stderr(logging.ERROR)
    #start()

    if startPncApp():
        pncApp_controller.command_queue.put(pncLibrary.PNCAppCommand('CONNECT', None, None, None, 'internal'))

    #pncApp_controller.join()
    # if 0:
    #     pncApp_controller = startMVC()
    #     pncApp_controller.command_queue.put(pncLibrary.PNCAppCommand('INIT', None, None, None, 'binary'))
    #     pncApp_controller.pncApp_initialized_event.wait()
    #     # #
    #     machine = pncApp_controller.machine
    #     synchronizer = pncApp_controller.synchronizer
    #     terminal_printer = pncApp_controller.terminal_printer
    #     feedback_state = pncApp_controller.feedback_state

    # pncApp_controller = startMVC()
    # machine = pncApp_controller.machine
    # synchronizer = pncApp_controller.synchronizer
    # terminal_printer = pncApp_controller.terminal_printer
    # feedback_state = pncApp_controller.feedback_state
    # pncApp_controller.init_pncApp()
    # pncApp_controller.start_pncApp()

    #connectToMachine()
    #userPythonFunction1(0,0,0,0,0)
    #userPythonFunction2(1,5,0,0,0)
    #userPythonFunction3(0,0,0,0,0)
    #
    # while True:
    #     print(eval(input("command: ")))
    #     print('looping')