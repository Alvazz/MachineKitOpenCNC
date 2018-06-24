import pncLibrary, os
from multiprocessing import Event, Queue, current_process
from pncApp import *#appInit, appStart, appStop, appClose


class CAM_MVC(Thread):
    def __init__(self):
        super(CAM_MVC, self).__init__()
        self.name = "sculptprint_MVC"
        self.machine_type = 'pocketnc'

        self.command_queue = Queue()
        self.startup_event = Event()
        self.t_run_CAM_MVC_thread = Event()
        self.pncApp_initialized_event = Event()
        self.pncApp_started_event = Event()

        self.main_process_name = str(current_process().name)
        self.main_process_pid = str(current_process().pid)

        self.pnc_app_manager = None
        self.machine = None
        self.synchronizer = None
        self.terminal_printer = None

    def run(self):
        #self.terminal_printer.print_message_queue.put("SculptPrint MVC thread started")
        print('SculptPrint MVC started')
        self.startup_event.set()
        #self.pnc_app_initialized_event.wait()
        while self.t_run_CAM_MVC_thread.is_set():
            command = self.command_queue.get()
            self.handleCommand(command)
        #pncLibrary.printTerminalString(self.machine.thread_launch_string, current_process().name, self.name)

    def handleCommand(self, command):
        command = command.split()
        if command[0] == 'INIT':
            self.init_pncApp()
        if command[0] == 'UN_INIT':
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
            #self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('ENQUEUE', (int(command[1]), int(command[2]))))
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('ENQUEUE', None))
        elif command[0] == 'EXECUTE':
            self.synchronizer.mvc_connect_event.wait()
            self.synchronizer.q_machine_controller_command_queue.put(pncLibrary.MachineCommand('EXECUTE', None))

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
