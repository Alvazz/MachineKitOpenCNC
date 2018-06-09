import serial, time, struct, math
import numpy as np
from pncDatabase import Record
from multiprocessing import Process, current_process
from threading import current_thread
import pncLibrary

class EncoderInterface(Process):
    def __init__(self, machine, synchronizer):
        super(EncoderInterface, self).__init__()
        self.name = "encoder_interface"
        self.main_thread_name = self.name + ".MainThread"
        self.device_name = "Quadrature Decoder Rev.2"
        self.machine = machine
        self.synchronizer = synchronizer

        #self.serial_port = serial_port

        #self._running = True
        #self._shutdown = False

        self.serial_port = serial.Serial()
        self.serial_port.port = self.machine.comm_port
        self.serial_port.baudrate = self.machine.baudrate

        self.encoder_reads = 0
        self.encoder_reading_time = 0
        self.encoder_read_time_moving_average = 0


        #print('serialInterface started')
        #time.sleep(0.5)
        #self._running_process = True

    def run(self):
        current_thread().name = self.main_thread_name
        print('break')
        try:
            if self.serial_port.isOpen():
                print('Serial port already open')
                self.serial_port.close()
                self.serial_port.open()
            else:
                self.serial_port.open()

            time.sleep(0.5)

            if not self.acknowledgeBoot():
                print('Decoder board not responsive')
                return
            else:
                pncLibrary.printTerminalString(self.machine.device_boot_string, self.device_name, self.serial_port.name)
            # serial_port.open()
            # time.sleep(0.5)
            #self._running_process = True
        except Exception as error:
            print('Could not open serial port, error: ' + str(error))

        self.synchronizer.ei_startup_event.set()
        self.synchronizer.process_start_signal.wait()
        time.clock()

        #FIXME this logic is not necessary
        if self.serial_port.isOpen():
            #print('Successful launch of encoder interface process, PID: ' + str(self.pid) + ', Process: ' + str(current_process()))
            #global encoderData, serialLock
            #self.setEncoderCount(self.machine.encoder_init)

            #FIXME uncomment for actual run
            # if self.synchronizer.mc_initial_position_set_event.wait() or 1:
            #     self.setAllEncoderCounts(self.machine.axes, self.positionsToCounts(self.machine.axes,list(map(lambda cp,mz: cp-mz,self.machine.current_position,self.machine.machine_zero))))
            #     self.machine.encoder_init_event.set()

            self.setBaudrate(250000)
            self.serial_port.flushInput()

            if self.synchronizer.p_enable_encoder_event.is_set():
                self.synchronizer.p_run_encoder_interface_event.wait()
                while self.synchronizer.p_run_encoder_interface_event.is_set():
                    #FIXME move to getEncoderCounts
                    request_time = time.clock()
                    encoder_counts = self.getEncoderCounts('current')
                    #print('serial read time is: ' + str(time.clock()-rx_received_time))

                    if encoder_counts[0]:
                        #Got good data, push to DB
                        self.encoder_reading_time += (time.clock() - request_time)
                        self.encoder_reads += 1
                        self.encoder_read_time_moving_average = self.encoder_reading_time/self.encoder_reads
                        self.machine.average_encoder_read_frequency = 1.0/self.encoder_read_time_moving_average

                        #FIXME should probably push packets of data instead of every sample
                        encoder_data_record = {'ENCODER_FEEDBACK_POSITIONS': np.asarray(self.countsToPositions(self.machine.axes, encoder_counts[1])), 'SERIAL_RECEIVED_TIMES': pncLibrary.estimateMachineClock(self.machine, encoder_counts[2])}
                        self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push',encoder_data_record))
                        time.sleep(1)
                        # self.machine.data_store_manager_thread_handle.push(
                        #     {'ENCODER_FEEDBACK_POSITIONS': np.asarray(self.countsToPositions(self.machine.axes, encoder_counts[1])), 'SERIAL_RECEIVED_TIMES': self.machine.estimateMachineClock(rx_received_time)})

            #Flag set, shutdown
            #print('Closing serial port')
            self.serial_port.close()
            print('ENCODER INTERFACE: Serial port closed')

            #self._shutdown = True
        else:
            print('closing serial interface process because port is not open')
            pncLibrary.printTerminalString(self.machine.process_terminate_string,self.name,self.pid)

    def read(self):
        line = self.serial_port.readline()
        return line

    def writeUnicode(self, string):
        self.serial_port.write(string.encode('utf-8'))
        #time.sleep(0.01)
        #self.serial_port.flush()

    ##    def write(self, data):
    ##        line = self.serial_port.readline()
    ##        return line

    # def reset(self):
    #     global encoderData, serialLock
    #     #serialLock.acquire()
    #     self.serial_port.close()
    #     self.serial_port.open()
    #     encoderData = np.array([0])
    #     queueData = np.array([0])
    #     #serialLock.release()

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

    def countToPosition(self, axis, counts):
        axis_index = self.machine.axes.index(axis.upper())
        return round(counts * self.machine.encoder_scale[axis_index]) - self.machine.encoder_offset[axis]

    def countsToPositions(self, axes, counts):
        positions = []
        for axis in range(0, len(axes)):
            axis_index = self.machine.axes.index(axes[axis].upper())
            positions.append((counts[axis] - self.machine.encoder_offset[axis_index]) * self.machine.encoder_scale[axis_index] + self.machine.machine_zero[axis_index])
        return positions

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
        time.sleep(0.5)

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

    # def setEncoderCount(self, count):
    #     # self.serial_port.write('R'.encode('utf-8'))
    #     # Calculate number of characters in set command
    #     numBytes = math.floor(round(math.log(count, 10),6)) + 1
    #     commandStr = 'S' + str(numBytes) + str(count) + '\n'
    #     print('setting' + commandStr)
    #     self.serial_port.write(commandStr.encode('utf-8'))
    #
    #     ack = self.waitForBytes(2)[1]
    #     if ack == 'S&':
    #         return True
    #     else:
    #         return False
    #     # readData = ''
    #     # while 'S&' not in readData:
    #     #     #print("waiting to read")
    #     #     readData += self.serial_port.read(1).decode("utf-8").strip()
    #     # print('Successful set of encoder count to ' + str(count))

    def getEncoderCounts(self, request_type):
        #print('requesting encoder count')
        #self.serial_port.write('G'.encode('utf-8'))
        if request_type.upper() == 'STORED':
            request_string = self.machine.encoder_command_strings[1]
            ack_string = self.machine.encoder_ack_strings[-1]
        elif request_type.upper() == 'CURRENT':
            request_string = self.machine.encoder_command_strings[2]
            ack_string = self.machine.encoder_ack_strings[-1]
        else:
            return (False, None)

        self.writeUnicode(request_string)
        encoder_bytes = self.serial_port.read(self.machine.max_encoder_transmission_length)
        rx_received_time = time.clock()
        encoder_data = struct.unpack('!' + 'c' + self.machine.number_of_encoders * 'I' + 'c', encoder_bytes)
        if encoder_data[0].decode() == request_string and encoder_data[-1].decode() == ack_string:
            return (True, encoder_data[1:-1], rx_received_time)
        return(False, None)

        # received_counts = self.waitForString(ack_string)
        #
        # if received_counts[0]:
        #     #First character should be G
        #     counts = received_counts[1].split(' ')
        #     if counts[0] == request_string and counts[-1] == ack_string:
        #         if len(counts[1]) < 9:
        #             print('break')
        #         return (True, list(map(int,counts[1:-1])))
        # #else:
        # return (False, None)

        # if self.waitForString(self.machine.encoder_ack_strings[1])[0]:
        #
        # #time.sleep(0.1)
        # readData = ''
        # while 'C&' not in readData:
        #     #print("waiting to read")
        #     readData += self.serial_port.read(1).decode("utf-8")
        # #print('Successful get of encoder count')
        # #print(readData.strip())
        # return readData.strip()

    def setBaudrate(self, baudrate):
        self.writeUnicode(self.machine.encoder_command_strings[3] + str(self.countBytesInTransmission(baudrate)) + str(baudrate))
        ack = self.waitForString(self.machine.encoder_ack_strings[4])
        if int(ack[1].split(' ')[1]) == baudrate:
            time.sleep(0.5)
            self.serial_port.baudrate = baudrate
            self.machine.baudrate = baudrate
            print('SUCCESS: Decoder board baudrate is now ' + str(baudrate))

    def close(self):
        print('setting serial port close flag')
        self._running_process = False
        #sys.exit()