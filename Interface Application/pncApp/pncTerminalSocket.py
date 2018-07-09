import socket, select
import pncLibrary
from threading import Thread
from multiprocessing import Queue, Event, current_process

class PNCAppInterfaceSocketLauncher(Thread):
    def __init__(self, parent):
        super(PNCAppInterfaceSocketLauncher, self).__init__()
        self.name = "interface_socket_launcher"
        self.parent = parent
        self.pncApp_command_queue = self.parent.command_queue

        self.interface_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
        self.interface_socket.setsockopt(socket.SIO_LOOPBACK_FAST_PATH)
        self.interface_socket.settimeout(pncLibrary.socket_wait_timeout)

        self.clients_connected = 0
        self.client_list = []
        self.t_run_interface_socket_launcher_event = Event()

        #self.client_connected_event = Event()
        self.startup_event = Event()

        self.t_run_interface_socket_listener_event.set()

    def run(self):
        self.interface_socket.bind(('localhost', pncLibrary.socket_interface_socket_port))
        self.interface_socket.listen()
        pncLibrary.printTerminalString(pncLibrary.thread_launch_string, current_process().name, self.name)
        self.startup_event.set()
        while self.t_run_interface_socket_launcher_event.is_set():
            try:
                (client_connection, client_address) = self.interface_socket.accept()
                self.launchClient(self, client_connection, client_address)
                print("PNCAPP INTERFACE SOCKET LAUNCHER: Connected to client %s at %s" % str(self.terminal_client), client_address)
            except socket.timeout:
                pass

        self.terminateClients()

        self.interface_socket.shutdown()
        self.interface_socket.close()

    def launchClient(self, connection):
        client = PNCAppInterfaceSocket(self, connection)
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
        self.client_type = 'socket'
        self.client_address = address
        self.parent = parent
        self.connection = connection
        self.pncApp_command_queue = self.parent.command_queue

        self.client_connected_event = Event()
        self.login_event = Event()
        self.startup_event = Event()

    def run(self):
        self.startup_event.set()
        while self.client_connected_event.is_set():
            try:
                if not self.login_event.is_set():
                    self.waitForSocketLogin()
                    pncLibrary.printTerminalString(pncLibrary.interface_socket_connection_string, self.client_name, self.client_address)
                inbound_message = pncLibrary.receiveIPCData(self.client_type, 'text', self.terminal_client)
                self.pncApp_command_queue.put(inbound_message)
            except (ConnectionAbortedError, ConnectionResetError):
                self.client_connected_event.clear()
                print('INTERFACE SOCKET: Client %s forcibly disconnected' % str(self.terminal_client_address))
            except EOFError:
                self.client_connected_event.clear()
                print('INTERFACE SOCKET: Client %s cleanly disconnected' % str(self.terminal_client_address))
            except TimeoutError:
                print('Connection timed out')

        self.parent.client_list.pop(self.parent.client_list.index(self))

    def waitForSocketLogin(self):
        login_message = pncLibrary.receiveIPCData(self.client_type, 'text', self.connection)
        try:
            login_command, name = login_message.data.split('_')
            if login_command == 'HELLO':
                self.client_name = name
                self.login_event.set()
        except Exception as error:
            print("I dont know what error type this is, so you should fill it in when this error inevitably happens: " + str(error))


# class UserCommandLineInterface(Thread):
#     def __init__(self, parent):
#         super(UserCommandLineInterface, self).__init__()
#         self.name = "terminal_socket_listener"
#         self.parent = parent
#         self.pncApp_command_queue = self.parent.command_queue
#
#         self.terminal_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
#         self.terminal_socket.setsockopt(socket.SIO_LOOPBACK_FAST_PATH)
#         self.terminal_socket.settimeout(pncLibrary.socket_wait_timeout)
#
#         self.t_run_terminal_socket_listener_event = Event()
#         self.startup_event = Event()
#         self.client_connected_event = Event()
#
#         self.t_run_terminal_socket_listener_event.set()

