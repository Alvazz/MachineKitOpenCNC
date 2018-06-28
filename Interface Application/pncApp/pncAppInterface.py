import pncLibrary, socket, os, pickle, wpipe, select#socketserver
from multiprocessing import Event, Queue, Pipe, current_process
from queue import Empty
from pncApp import *#appInit, appStart, appStop, appClose
import numpy as np
#from pncSculptPrintIntegration import *

pncLibrary.updatePath()
sculptprint_MVC = None

class CAMSocketListener(Thread):
    def __init__(self, parent):
        super(CAMSocketListener, self).__init__()
        self.name = "CAM_socket_listener"
        self.parent = parent
        self.MVC_command_queue = self.parent.command_queue

        self.CAM_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
        #self.CAM_socket.setsockopt(socket.SIO_LOOPBACK_FAST_PATH, socket.SIO_RCVALL)
        #self.CAM_socket.ioctl(socket.SIO_RCVALL, socket.RCVALL_ON)
        self.t_run_CAM_listener_event = Event()
        self.startup_event = Event()
        self.client_connected_event = Event()

        self.t_run_CAM_listener_event.set()

    def run(self):
        self.CAM_socket.bind(('localhost', pncLibrary.mvc_socket_port))
        self.CAM_socket.listen()
        pncLibrary.printTerminalString(pncLibrary.thread_launch_string, current_process().name, self.name)
        self.startup_event.set()
        while self.t_run_CAM_listener_event.is_set():
            try:
                self.CAM_socket.settimeout(1)
                (self.CAM_client, self.CAM_client_address) = self.CAM_socket.accept()
                print("SCULPTPRINT MVC: Connected to client %s" % str(self.CAM_client))
                self.client_type = 'socket'
                self.parent.CAM_client = self.CAM_client
                self.client_connected_event.set()
            except socket.timeout:
                pass
                #print('No client found')

            while self.client_connected_event.is_set():
                try:
                    received_data = pncLibrary.receiveIPCData('socket', self.CAM_client)
                    self.MVC_command_queue.put(received_data.data)
                except (ConnectionAbortedError, ConnectionResetError):
                    self.client_connected_event.clear()
                    print('SCULPTPRINT MVC: Client %s forcibly disconnected' % str(self.CAM_client_address))
                except EOFError:
                    self.client_connected_event.clear()
                    print('SCULPTPRINT MVC: Client %s cleanly disconnected' % str(self.CAM_client_address))
                # except ConnectionResetError:
                #     self.client_connected_event.clear()
                #     print('SCULPTPRINT MVC: Client %s disconnected' % str(CAM_client_address))
                except TimeoutError:
                    print('Connection timed out')

        self.CAM_socket.shutdown()
        self.CAM_socket.close()

class CAM_MVC(Thread):
    def __init__(self):
        super(CAM_MVC, self).__init__()
        self.name = "sculptprint_MVC"
        self.machine_type = "pocketnc"
        self.main_thread_name = self.name + ".MainThread"

        self.command_queue = Queue()
        self.startup_event = Event()
        self.t_run_CAM_MVC_thread = Event()
        self.pncApp_initialized_event = Event()
        self.pncApp_started_event = Event()

        self.main_process_name = str(current_process().name)
        self.main_process_pid = str(current_process().pid)

        self.mvc_pipe = wpipe.Server(pncLibrary.sculptprint_ipc_pipe_name, wpipe.Mode.Slave)

        self.pnc_app_manager = None
        self.machine = None
        self.synchronizer = None
        self.terminal_printer = None

    def run(self):
        print('CAM MVC: Starting...')
        pncLibrary.waitForThreadStart(self, CAMSocketListener)
        self.startup_event.set()
        #self.pnc_app_initialized_event.wait()
        print(current_process().pid)
        while self.t_run_CAM_MVC_thread.is_set():
            try:
                command = self.command_queue.get_nowait()
                self.handleCommand(command)
            except Empty:
                pass
            except ConnectionResetError:
                #FIXME move to socket manager thread?
                print('SCULPTPRINT MVC: Client %s forcibly disconnected during transfer' % str(self.CAM_client_address))

            # try:
            #     command = pncLibrary.receiveIPCData(self.mvc_pipe.clients[0])
            #     self.command_queue.put(command.data)
            # except ConnectionAbortedError:
            #     print('SCULPTPRINT INTERFACE: Pipe disconnected from client')
            # except TimeoutError:
            #     pass
            # except IndexError:
            #     #No clients connected
            #     pass

            #self.mvc_pipe.dropdeadclients()

        self.mvc_pipe.close()
        pncLibrary.waitForThreadStop(self, self.CAM_socket_listener)
        #pncLibrary.printTerminalString(self.machine.thread_launch_string, current_process().name, self.name)

    def handleCommand(self, command):
        print(command.split()[0])
        command = command.split()
        if command[0] == 'HELLO':
            #pncLibrary.sendToSP('pipe', self.mvc_pipe, command[0] + '_ACK')
            pncLibrary.sendIPCData(self.CAM_socket_listener.client_type, self.CAM_client, command[0] + '_ACK')
            #pncLibrary.sendIPCData(self.mvc_pipe.clients[0], command[0] + '_ACK')
        elif command[0] == 'INIT':
            self.init_pncApp()
        # elif command[0] == 'GET_PIPE':
        #     pncLibrary.sendToSP('udp', self.mvc_socket, self.sculptprint_pipe_connection)
        elif command[0] == 'CHECKSTATUS':
            if self.pncApp_initialized_event.is_set():
                pncLibrary.sendToSP(self.mvc_socket, self.synchronizer.mvc_pncApp_started_event.is_set())
            else:
                pncLibrary.sendToSP(self.mvc_socket, False)
        elif command[0] == 'UN_INIT':
            self.uninit_pncApp()
        elif command[0] == 'START':
            self.synchronizer.mvc_pncApp_initialized_event.wait()
            try:
                self.start_pncApp()
            except Exception as error:
                print("SCULPTPRINT MVC: Could not launch pncApp, error " + str(error))
                self.command_queue = Queue()
        elif command[0] == 'CLOSE':
            #FIXME only after connect to guarantee encoder sync?
            self.synchronizer.mvc_pncApp_started_event.wait()
            self.close_pncApp()
        elif command[0] == 'CONNECT':
            try:
                self.synchronizer.mvc_pncApp_started_event.wait(self.machine.event_wait_timeout)
                self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('CONNECT', None))
            except Exception as error:
                print("SCULPTPRINT MVC: Connection problem, error " + str(error))
        elif command[0] == 'ENQUEUE':
            self.synchronizer.mvc_connect_event.wait()
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('ENQUEUE', (int(command[1]), int(command[2]))))
            #self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('ENQUEUE', None))
        elif command[0] == 'EXECUTE':
            self.synchronizer.mvc_connect_event.wait()
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('EXECUTE', None))
        elif command[0] == 'ISMONITORING':
            #self.sculptprint_socket.sendto(pickle.dumps(pncLibrary.SP.isMonitoring(self.synchronizer)), ('127.0.0.1', 42069))
            #pncLibrary.sendToSP('pipe', self.mvc_pipe, pncLibrary.SP.isMonitoring(self.synchronizer))
            pncLibrary.sendIPCData(self.CAM_socket_listener.client_type, self.CAM_client, pncLibrary.SP.isMonitoring(self.synchronizer))
        elif command[0] == 'READ':
            #pncLibrary.sendIPCData(self.mvc_pipe.clients[0], 'abcd')
            #pncLibrary.sendIPCData(self.mvc_pipe.clients[0],np.random.rand(200,200))
            try:
                data = pncLibrary.SP.readMachine(self.machine, self.synchronizer, self.feedback_state, int(command[1]))
                pncLibrary.sendIPCData(self.CAM_socket_listener.client_type, self.CAM_client, pncLibrary.SP.readMachine(self.machine, self.synchronizer, self.feedback_state, int(command[1])))
            except Exception as error:
                print('break')
        elif command[0] == 'CLOSE_SP_INTERFACE':
            pncLibrary.sendIPCData(self.CAM_socket_listener.client_type, self.CAM_client, command[0] + '_ACK')
            self.CAM_socket_listener.client_connected_event.clear()
            print('SCULPTPRINT MVC: Client %s cleanly disconnected' % str(self.CAM_socket_listener.CAM_client_address))

    # def SendOutputData(self, data):
    #     self.outbound_socket.sendto(pickle.dumps(data), ('127.0.0.1', 42069))

    def init_pncApp(self):
        #if self.pnc_app_manager is None:
        #Only start a new process if we haven't inited yet
        self.pnc_app_manager, self.machine, self.synchronizer = appInit()
        self.terminal_printer = pncLibrary.startPrintServer(self.machine, self.synchronizer)
        self.pncApp_initialized_event.set()
        #self.synchronizer.mvc_pncApp_initialized_event.set()

    def uninit_pncApp(self):
        pass

    def start_pncApp(self):
        #pncLibrary.printTerminalString(self.machine.sculptprint_interface_initialization_string, self.main_process_name,self.main_process_pid)
        self.database, self.encoder_interface, self.feedback_handler, self.machine_controller = appStart(self.machine_type, self.pnc_app_manager,
                                                                                     self.machine, self.synchronizer)
        #self.pncApp_started_event.set()
        #self.synchronizer.mvc_pncApp_started_event.set()
        #return True

    def stop_pncApp(self):
        appStop(self.pnc_app_manager, self.synchronizer)

    def close_pncApp(self):
        # FIXME do something about MVC still running
        appClose(self.pnc_app_manager, self.synchronizer)
        print('closed')

def startMVC():
    sculptprint_MVC = CAM_MVC()
    sculptprint_MVC.feedback_state = pncLibrary.SculptPrintFeedbackState()
    sculptprint_MVC.t_run_CAM_MVC_thread.set()
    sculptprint_MVC.start()
    sculptprint_MVC.startup_event.wait()

    return sculptprint_MVC

def startPncApp():
    global sculptprint_MVC, machine, synchronizer, terminal_printer, feedback_state
    if sculptprint_MVC is None:
        sculptprint_MVC = startMVC()

        sculptprint_MVC.command_queue.put('INIT')
        sculptprint_MVC.pncApp_initialized_event.wait()

        machine = sculptprint_MVC.machine
        synchronizer = sculptprint_MVC.synchronizer
        terminal_printer = sculptprint_MVC.terminal_printer
        feedback_state = sculptprint_MVC.feedback_state


    sculptprint_MVC.command_queue.put('START')
    #pncLibrary.printTerminalString(machine.sculptprint_interface_initialization_string, sculptprint_MVC.main_process_name, sculptprint_MVC.main_process_pid)
    pncLibrary.printTerminalString(machine.sculptprint_interface_initialization_string, sculptprint_MVC.main_process_name, sculptprint_MVC.main_process_pid)
    print('SCULPTPRINT MVC: Waiting for startup...')
    synchronizer.mvc_pncApp_started_event.wait(machine.max_mvc_startup_wait_time)
    return True

if __name__ == '__main__':
    #multiprocessing.freeze_support()
    #multiprocessing.log_to_stderr(logging.ERROR)
    #start()

    startPncApp()
    sculptprint_MVC.command_queue.put('CONNECT')

    #sculptprint_MVC.join()
    if 0:
        sculptprint_MVC = startMVC()
        sculptprint_MVC.command_queue.put('INIT')
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