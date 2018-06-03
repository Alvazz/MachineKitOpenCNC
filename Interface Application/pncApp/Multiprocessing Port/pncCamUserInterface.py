import threading

class SculptPrintInterface():
    #MVC for SculptPrint UI
    def __init__(self):
        super(SculptPrintInterface, self).__init__()
        #self.machine = machine
        self.connected = False

        #Flags for user events
        self.enqueue_moves = False

        #UI Data Fields
        self.start_file = 0
        self.end_file = 0

        #Events
        self.connect_event = threading.Event()
        self.enqueue_moves_event = threading.Event()
        self.moves_queued_event = threading.Event()
        self.run_motion_event = threading.Event()