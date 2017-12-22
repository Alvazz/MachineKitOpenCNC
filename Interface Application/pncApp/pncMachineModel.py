class MachineModel():
    def __init__(self):
        #State variables
        self.modes = ['MANUAL', 'MDI', 'AUTO']
        self.statuses = ['IDLE', 'RUNNING', 'PAUSED']
        self.rsh_feedback_strings = ['bL=', 'PROGRAM_STATUS', 'MODE', 'NAK']
        self.axes = ['X','Y','Z','A','B']

        #Init states
        self.mode = self.modes[0]
        self.status = self.statuses[2]
        self.binaryMode = 0
        self.loggingMode = 0
        self.units = 'inch'
        self.rsh_error = 0
        

    ### State Machine ###
    def saveState(self):
        #save machine state
        return #saved state structure

    def restoreState(self, stateStruct):
        #Restore state to prev state after op
        return
