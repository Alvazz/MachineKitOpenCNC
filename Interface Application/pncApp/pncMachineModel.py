from pncMachineControl import MachineController

class MachineModel():
    global machine_controller
    def __init__(self):
        #State variables
        self.modes = ['MANUAL', 'MDI', 'AUTO']
        self.statuses = ['IDLE', 'RUNNING', 'PAUSED']
        self.rsh_feedback_strings = ['bL=', 'PROGRAM_STATUS', 'MODE', 'ON', 'NAK']
        self.axes = ['X','Y','Z','A','B']
        self.axis_offsets = [-0.00085, 2.5, .0013, .114, -.002]
        self.linked = 0

        #Encoder calibration
        self.machine_zero = [-1.75, -2.05, 0.1, -5, 0]
        self.limits = [[-1.75, 2.55], [-2.05, 2.95], [-3.45, 0.1], [-5, 95], [-99999, 99999]]
        self.encoder_init = 1000000
        self.encoder_offset = [155836, 180838, 2283, 9121, 0]
        #self.encoder_scale = [1/5/8000, 1/5/8000, 1/5/8000, 1/35.5368/8000, 1/35.5555/8000]
        self.encoder_scale = [.096 / 8000, .096 / 8000, .096 / 8000, -1.0 / 172, -1.0 / 167]

        #Init states
        self.mode = self.modes[0]
        self.status = self.statuses[2]
        self.binaryMode = 0
        self.loggingMode = 0
        self.units = 'inch'
        self.rsh_error = 0
        
        #State stack
        self.prev_mode = self.modes[0]

    ### State Machine ###
    def saveState(self):
        #save machine state
        self.prev_mode = self.mode
        return #saved state structure

    def restoreState(self, stateStruct):
        machine_controller.modeSwitchWait(self.prev_mode)
        #Restore state to prev state after op
        return
