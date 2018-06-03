import serial, math, threading, time
from multiprocessing import Process
from pncMachineModel import MachineModel

class SerialInterface(Process):
    def __init__(self, machine):
        super(SerialInterface, self).__init__()
        self.machine = machine

        #self._running = True
        self._shutdown = False

        self.serial_port = serial.Serial()
        self.serial_port.port = self.machine.comm_port
        self.serial_port.baudrate = self.machine.baudrate
        #FIXME handle if serial is not connected

        try:
            if self.serial_port.isOpen():
                print('serial port already open')
            self.serial_port.open()

            time.sleep(0.5)
            self._running_thread = True

            # if self.machine.initial_position_set_event.wait():
            #     self.setAllEncoderCounts(self.machine.axes,self.machine.current_position)
            #     self.machine.encoder_init_event.set()
            #while not self.machine.data_store_manager_thread_handle.pull(['stepgen_feedback_positions'],[-1],[None])[0]:

        except Exception as error:
            print('Could not open serial port, error: ' + str(error))


        print('serialInterface started')
        #time.sleep(0.5)

    def run(self):
        #global encoderData, serialLock
        #self.setEncoderCount(self.machine.encoder_init)
        if not self.acknowledgeBoot():
            print('Decoder board not responsive')
            return
        else:
            print('Successful boot of decoder board')

        if self.machine.initial_position_set_event.wait():
            self.setAllEncoderCounts(self.machine.axes, self.positionsToCounts(self.machine.axes,list(map(lambda cp,mz: cp-mz,self.machine.current_position,self.machine.machine_zero))))
            self.machine.encoder_init_event.set()

        self.setBaudrate(250000)

        while self._running_thread:
            #counts = self.getEncoderCounts('current')
            start = time.clock()
            self.writeUnicode('G')
            if self.serial_port.in_waiting > self.machine.max_encoder_transmission_length:
                self.serial_port.readline()

        #Flag set, shutdown
        print('Closing serial port')
        print('ENCODER INTERFACE: thread shut down')
        self.serial_port.close()
        #self._shutdown = True

    def read(self):
        line = self.serial_port.readline()
        return line

    def writeUnicode(self, string):
        self.serial_port.write(string.encode('utf-8'))

    ########################### UTILITIES ###########################

    def countBytesInTransmission(self, transmission):
        return math.floor(round(math.log(transmission, 10), 6)) + 1

    def positionToCounts(self, axis, position):
        axis_index = self.machine.axes.index(axis.upper())
        return round(position / self.machine.encoder_scale[axis_index]) + self.machine.encoder_offset[axis_index]

    def positionsToCounts(self, axes, positions):
        counts = []
        for axis in range(0,len(axes)):
            axis_index = self.machine.axes.index(axes[axis].upper())
            counts.append(int(round(positions[axis] / self.machine.encoder_scale[axis_index]) + self.machine.encoder_offset[axis_index]))
        return counts

    def countsToPosition(self, axis, counts):
        axis_index = self.machine.axes.index(axis.upper())
        return round(counts * self.machine.encoder_scale[axis_index]) - self.machine.encoder_offset[axis]

    def waitForBytes(self, number_of_bytes, timeout = 0.5):
        read_byte_count = 0
        read_data = ''
        start_time = time.clock()
        while read_byte_count < number_of_bytes and (time.clock() - start_time) < timeout:
            read_data += self.serial_port.read(1).decode('utf-8')
            read_byte_count += 1

        if read_byte_count == number_of_bytes:
            return (True, read_data)
        else:
            return (False, None)

    def waitForString(self, string, timeout = 0.5):
        # #read_byte_count = 0
        # read_data = ''
        # start_time = time.clock()
        # while string not in read_data and (time.clock() - start_time) < timeout:
        #     read_data += self.serial_port.readline().decode('utf-8')
        #     #read_byte_count += 1

        start = time.clock()
        read_data = self.serial_port.readline().decode('utf-8')
        print('read took ' + str(time.clock()-start))
        if string in read_data:
            return (True, read_data)
        else:
            return (False, None)

    def rebootDevice(self):
        self.serial_port.setDTR(0)
        self.serial_port.setDTR(1)

    ########################### COMMS ###########################

    def acknowledgeBoot(self):
        return self.waitForString(self.machine.encoder_ack_strings[0])[0]

    def setAxisEncoderCount(self, axis, count):
        #number_of_bytes = math.floor(round(math.log(count, 10),6)) + 1
        command_string = 'S' + str(axis) + str(self.countBytesInTransmission(count)) + str(count)
        self.writeUnicode(command_string)
        if self.waitForString(self.machine.encoder_ack_strings[1]):
        #if self.waitForBytes(len(self.machine.encoder_ack_strings[0]))[1] == self.machine.encoder_ack_strings[0]:
            return True
        else:
            return False

    def setAllEncoderCounts(self, axes, counts):
        for axis in axes:
            axis_index = self.machine.axes.index(axis)
            self.setAxisEncoderCount(axis_index+1,counts[axis_index])

    def getEncoderCounts(self, request_type):
        #print('requesting encoder count')
        #self.serial_port.write('G'.encode('utf-8'))
        if request_type.upper() == 'STORED':
            request_string = self.machine.encoder_command_strings[1]
            ack_string = self.machine.encoder_ack_strings[2]
        elif request_type.upper() == 'CURRENT':
            request_string = self.machine.encoder_command_strings[2]
            ack_string = self.machine.encoder_ack_strings[3]
        else:
            return (False, None)

        self.writeUnicode(request_string)
        received_counts = self.waitForString(ack_string)

        if received_counts[0]:
            #First character should be G
            counts = received_counts[1].split(' ')
            if counts[0] == request_string and counts[-1] == ack_string:
                if len(counts[1]) < 9:
                    print('break')
                return (True, list(map(int,counts[1:-1])))
        #else:
        return (False, None)

    def setBaudrate(self, baudrate):
        self.writeUnicode(self.machine.encoder_command_strings[3] + str(self.countBytesInTransmission(baudrate)) + str(baudrate))
        ack = self.waitForString(self.machine.encoder_ack_strings[4])
        if int(ack[1].split(' ')[1]) == baudrate:
            time.sleep(0.5)
            self.serial_port.baudrate = baudrate
            print('SUCCESS: Decoder board baudrate is now ' + str(baudrate))

    def close(self):
        print('setting serial port close flag')
        self._running_thread = False
        #sys.exit()

if __name__ == '__main__':
    machine = MachineModel()
    encoder_interface = SerialInterface(machine)
    encoder_interface.start()