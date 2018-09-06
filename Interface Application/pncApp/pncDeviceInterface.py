import serial, time, struct, math, odrive
import numpy as np
from multiprocessing import Process, Event, Queue, current_process
from threading import Thread, current_thread
from queue import Empty
from odrive.enums import *
import pncLibrary

class SpindleInterface(Thread):
    def __init__(self, parent):
        super(SpindleInterface, self).__init__()
        self.name = "spindle_interface"
        self.device_name = "Spindle ODrive v3.4"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer
        self.command_queue = Queue()

        self.spindle_drive = None
        self.startup_event = Event()

    def run(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_launch_string, current_process().name,
                                                     self.name)
        self.startup_event.set()
        #pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, pncLibrary.printout_spindle_drive_search_string)
        print('SPINDLE CONTROLLER: Looking for ODrive...')
        #return
        if self.connectToODrive():
            self.configureSpindle()

            ramp_thread = Thread(target=self.rampSpindle, args=(100,1000))
            ramp_thread.start()

            while self.synchronizer.t_run_spindle_interface_event.is_set():
                try:
                    command = self.command_queue.get(True, pncLibrary.queue_wait_timeout)
                    self.handleCommand(command)
                except Empty:
                    self.setSpindleSpeed(1000)
                    while True:
                        start = time.clock()
                        print(self.spindle_drive.axis0.encoder.pos_estimate)
                        print(time.clock()-start)
                    #pass
                finally:
                    self.setSpindleSpeed(0)

    def handleCommand(self, command):
        if command.command_type == 'SETSPEED':
            self.setSpindleSpeed(command.command_data)

    def connectToODrive(self):
        try:
            self.spindle_drive = odrive.find_any(timeout=pncLibrary.device_search_timeout)
            #self.spindle_drive.reboot()
            #self.spindle_drive = odrive.find_any(timeout=pncLibrary.device_search_timeout)
            #if self.spindle_drive is not None:
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_device_comm_initialization_string,
                                                         self.device_name, self.name)
            self.synchronizer.di_spindle_drive_connected_event.set()
            return True
            # else:
            #     pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
            #                                                  pncLibrary.printout_device_comm_initialization_failed_string,
            #                                                  self.name, self.device_name)
        except TimeoutError:
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_device_comm_initialization_failed_string,
                                                         self.name, self.device_name)
            return False
            # pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
            #                                              pncLibrary.printout_device_comm_initialization_failed_string,
            #                                              self.name, self.device_name)

    def configureSpindle(self):
        self.setControllerGains()
        self.setMaximumSpindleSpeed(self.machine.spindle_max_rpm)
        self.setSpindleSpeed(0)

    def setControllerGains(self):
        getattr(self.spindle_drive, self.machine.spindle_axis).motor.config.current_lim = self.machine.spindle_motor_current_limit
        getattr(self.spindle_drive, self.machine.spindle_axis).controller.config.vel_gain = self.machine.spindle_velocity_gain
        getattr(self.spindle_drive, self.machine.spindle_axis).controller.config.pos_gain = self.machine.spindle_position_gain
        getattr(self.spindle_drive, self.machine.spindle_axis).controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

    def setSpindleSpeed(self, speed_in_rpm):
        #self.spindle_drive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        #self.spindle_drive.axis1.controller.vel_setpoint = (self.rpmToCountFrequency(speed_in_rpm))
        #getattr(self.spindle_drive, self.machine.spindle_axis).controller.set_vel_setpoint(self.rpmToCountFrequency(speed_in_rpm), self.machine.spindle_current_feed_forward)
        getattr(self.spindle_drive, self.machine.spindle_axis).controller.vel_setpoint = self.rpmToCountFrequency(speed_in_rpm)

    def rampSpindle(self, acceleration, target_rpm):
        #accel_cps = self.rpmToCountFrequency(acceleration)
        target_velocity = self.rpmToCountFrequency(target_rpm)
        loop_delay = 0.1
        loop_time = time.time()

        vel = self.countFrequencyToRPM(self.spindle_drive.axis1.encoder.pll_vel)
        while self.spindle_drive.axis1.encoder.pll_vel <= target_velocity:
            #self.setSpindleSpeed(acceleration * loop_delay)
            dt = time.time()-loop_time
            loop_time = time.time()
            vel += dt*acceleration
            #vel = self.spindle_drive.axis1.encoder.pll_vel + accel_cps * dt
            #self.spindle_drive.axis1.controller.set_vel_setpoint(vel, self.machine.spindle_current_feed_forward)
            self.setSpindleSpeed(vel)
            time.sleep(loop_delay)

        self.setSpindleSpeed(target_rpm)
        print('RAMP COMPLETE')
        #time.sleep(1)
        #self.spindle_drive.axis1.controller.set_vel_setpoint(target_velocity, self.machine.spindle_current_feed_forward)

    def setMaximumSpindleSpeed(self, speed_in_rpm):
        getattr(self.spindle_drive, self.machine.spindle_axis).controller.config.vel_limit = self.rpmToCountFrequency(speed_in_rpm)
        #self.spindle_drive.axis0.controller.config.vel_limit = self.rpmToCountFrequency(speed_in_rpm)

    def rpmToCountFrequency(self, rpm):
        return self.machine.spindle_encoder_resolution * rpm/60

    def countFrequencyToRPM(self, f):
        return f/self.machine.spindle_encoder_resolution * 60

    def syncStateMachine(self):
        pass

class EncoderInterface(Thread):
    def __init__(self, parent):
        super(EncoderInterface, self).__init__()
        #self.parent = parent
        self.name = "encoder_interface"
        self.device_name = "Quadrature Decoder Rev.2"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer
        self.command_queue = Queue()

        self.serial_port = serial.Serial()
        self.serial_port.port = self.machine.comm_port
        self.serial_port.baudrate = self.machine.initial_baudrate
        self.serial_port.timeout = self.machine.serial_read_timeout

        self.encoder_reads = 0
        self.encoder_reading_time = 0
        self.encoder_read_time_moving_average = 0

        self.encoder_position_buffer = np.zeros((self.machine.encoder_read_buffer_size, self.machine.number_of_joints))
        self.encoder_time_buffer = np.zeros((self.machine.encoder_read_buffer_size, 1))

        self.startup_event = Event()

    def run(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_launch_string, current_process().name,
                                                     self.name)
        self.startup_event.set()
        try:
            #self.synchronizer.ei_startup_event.set()
            #self.synchronizer.process_start_signal.wait()
            #time.clock()
            if not self.initializeEncoderCommunication():
                pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, 'ENCODER INTERFACE: Decoder board not responsive, terminating thread')
                raise ConnectionRefusedError

            if not self.syncEncoderToStepgen(self.machine.encoder_event_wait_timeout):
                pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, 'ENCODER INTERFACE: Stepgen synchronization failed, terminating thread')
                raise TimeoutError

            self.initializeBaudrate(self.machine.target_baudrate)

            #if self.synchronizer.p_enable_encoder_event.is_set():
            #self.synchronizer.ei_successful_start_event.set()
            #self.synchronizer.p_run_encoder_interface_event.wait()
            pncLibrary.printTerminalString(self.machine.device_comm_initialization_string, self.device_name,
                                           self.name)
            while self.synchronizer.t_run_encoder_interface_event.is_set():
                #raise serial.SerialException

                encoder_buffer_data_flag = True
                for read_count in range(0, self.machine.encoder_read_buffer_size):
                    request_time = time.time()
                    encoder_counts = self.getEncoderCounts('current')

                    if encoder_counts[0]:
                        self.encoder_position_buffer[read_count] = np.asarray(
                            self.countsToPositions(self.machine.axes, encoder_counts[1]))
                        self.encoder_time_buffer[read_count] = np.asarray(
                            pncLibrary.estimateMachineClock(self.machine, encoder_counts[2]))
                        encoder_buffer_data_flag = encoder_buffer_data_flag and True
                        #print('encoder read time is ' + str(time.time()-request_time))
                    else:
                        print('had bad encoder reading')
                        encoder_buffer_data_flag = False

                if self.synchronizer.mc_xenomai_clock_sync_event.is_set() and encoder_buffer_data_flag:
                    # encoder_data_record = {'ENCODER_FEEDBACK_POSITIONS': np.asarray([self.countsToPositions(self.machine.axes, encoder_counts[1])]), 'SERIAL_RECEIVED_TIMES': np.array([[pncLibrary.estimateMachineClock(self.machine, encoder_counts[2])]])}
                    encoder_data_record = {'ENCODER_FEEDBACK_POSITIONS': self.encoder_position_buffer,
                                           'SERIAL_RECEIVED_TIMES': self.encoder_time_buffer}
                    self.synchronizer.q_database_command_queue_proxy.put(
                        pncLibrary.DatabaseCommand('push', [encoder_data_record]))
            # else:
            #     pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, 'ENCODER INTERFACE: Decoder board not responsive, terminating thread')

            # self.initializeBaudrate(self.machine.target_baudrate)
            #
            # if self.synchronizer.p_enable_encoder_event.is_set():
            #     self.synchronizer.ei_successful_start_event.set()
            #     self.synchronizer.p_run_encoder_interface_event.wait()
            #     pncLibrary.printTerminalString(self.machine.device_comm_initialization_string, self.device_name, self.name)
            #     while self.synchronizer.p_run_encoder_interface_event.is_set():
            #         raise serial.SerialException
            #
            #         encoder_buffer_data_flag = True
            #         for read_count in range(0, self.machine.encoder_read_buffer_size):
            #             request_time = time.time()
            #             encoder_counts = self.getEncoderCounts('current')
            #
            #             if encoder_counts[0]:
            #                 self.encoder_position_buffer[read_count] = np.asarray(self.countsToPositions(self.machine.axes, encoder_counts[1]))
            #                 self.encoder_time_buffer[read_count] = np.asarray(pncLibrary.estimateMachineClock(self.machine, encoder_counts[2]))
            #                 encoder_buffer_data_flag = encoder_buffer_data_flag and True
            #             else:
            #                 print('had bad encoder reading')
            #                 encoder_buffer_data_flag = False
            #
            #         if self.synchronizer.mc_xenomai_clock_sync_event.is_set() and encoder_buffer_data_flag:
            #             #encoder_data_record = {'ENCODER_FEEDBACK_POSITIONS': np.asarray([self.countsToPositions(self.machine.axes, encoder_counts[1])]), 'SERIAL_RECEIVED_TIMES': np.array([[pncLibrary.estimateMachineClock(self.machine, encoder_counts[2])]])}
            #             encoder_data_record = {'ENCODER_FEEDBACK_POSITIONS': self.encoder_position_buffer, 'SERIAL_RECEIVED_TIMES': self.encoder_time_buffer}
            #             self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push',[encoder_data_record]))

        except serial.serialutil.SerialException as error:
            print('Encoder interface error: ' + str(error))
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_device_comm_initialization_failed_string,
                                                         self.name, self.device_name)
            #self.synchronizer.ei_startup_event.set()
        except serial.SerialException as error:
            #self.synchronizer.ei_startup_event.set()
#           pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, pncLibrary.printout_device_comm_initialization_failed_string, self.name, self.device_name)
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.connection_failed_string+str(error), self.machine.comm_port)
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         self.machine.process_self_terminate_string,
                                                         self.name,
                                                         self.pid)
        # else:
        #

        ## FIXME need to __del__() the port?
        self.serial_port.close()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.connection_close_string, self.device_name,
                                                     self.machine.comm_port)


    def initializeEncoderCommunication(self):
        if self.serial_port.is_open:
            pncLibrary.printStringToTerminalMessageQueue('serial port already open')
            self.serial_port.close()
            time.sleep(0.5)
            self.serial_port.open()
        else:
            self.serial_port.open()

        time.sleep(0.5)

        if not self.acknowledgeBoot():
            #print('ENCODER INTERFACE: Decoder board not responsive, terminating process')
            self.synchronizer.di_encoder_comm_init_failed_event.set()
            #raise IOError
            return False
        else:
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         self.machine.device_boot_string, self.device_name,
                                                         self.serial_port.name)
            self.synchronizer.ei_encoder_comm_init_event.set()
            return True

    def syncEncoderToStepgen(self, timeout = None):
        try:
            self.synchronizer.mc_initial_stepgen_position_set_event.wait(timeout)
            self.setAllEncoderCounts(self.machine.axes, self.positionsToCounts(self.machine.axes, self.machine.current_stepgen_position))
            self.synchronizer.ei_encoder_init_event.set()
            return True
        except Exception as error:
            self.synchronizer.q_print_server_message_queue.put("ENCODER INTERFACE: Wait on stepgen sync timed out, error: " + str(error))
            return False

    def initializeBaudrate(self, baudrate):
        self.serial_port.readline()
        self.serial_port.reset_input_buffer()
        self.setBaudrate(baudrate)
        self.serial_port.reset_input_buffer()

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
        return np.asarray(counts)

    def countsToPositions(self, axes, counts):
        positions = []
        for axis in range(0, len(axes)):
            axis_index = self.machine.axes.index(axes[axis].upper())
            positions.append((counts[axis] - self.machine.encoder_offset[axis_index]) * self.machine.encoder_scale[axis_index])# + self.machine.machine_table_center_zero[axis_index])
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
        read_data = self.serial_port.readline().decode('utf-8')
        if string in read_data:
            return (True, read_data)
        else:
            return (False, None)

    def rebootDevice(self):
        self.serial_port.setDTR(0)
        self.serial_port.setDTR(1)
        time.sleep(0.5)

    ########################### COMMS ###########################
    def read(self):
        line = self.serial_port.readline()
        return line

    def writeUnicode(self, string):
        self.serial_port.write(string.encode('utf-8'))

    def acknowledgeBoot(self):
        return self.waitForString(self.machine.encoder_ack_strings[0])[0]

    def setAxisEncoderCount(self, axis, count):
        command_string = 'S' + str(axis) + str(self.countBytesInTransmission(count)) + str(count)
        self.writeUnicode(command_string)
        if self.waitForString(self.machine.encoder_ack_strings[1])[0]:
            return True
        else:
            return False

    def setAllEncoderCounts(self, axes, counts):
        for axis in axes:
            time.sleep(0.05)
            axis_index = self.machine.axes.index(axis)
            while not self.setAxisEncoderCount(axis_index+1,counts[axis_index]):
                print('ENCODER INTERFACE: Retrying set axis encoder count for axis ' + axis)

    def getEncoderCounts(self, request_type):
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
        rx_received_time = time.time()
        encoder_data = struct.unpack('!' + 'c' + self.machine.number_of_encoders * 'I' + 'c', encoder_bytes)
        if encoder_data[0].decode() == request_string and encoder_data[-1].decode() == ack_string:
            return (True, encoder_data[1:-1], rx_received_time)
        return(False, None)

    def setBaudrate(self, baudrate):
        self.writeUnicode(self.machine.encoder_command_strings[3] + str(self.countBytesInTransmission(baudrate)) + str(baudrate))
        ack = self.waitForString(self.machine.encoder_ack_strings[4])
        #FIXME why the fuck did this break itself
        #return True
        if int(ack[1].split(' ')[1]) == baudrate:
            time.sleep(0.5)
            self.serial_port.baudrate = baudrate
            self.machine.baudrate = baudrate
            print('SUCCESS: Decoder board baudrate is now ' + str(baudrate))

class DeviceInterface(Process):
    def __init__(self, machine, pipe):
        super(DeviceInterface, self).__init__()
        self.name = "device_interface"
        self.main_thread_name = self.name + ".MainThread"
        self.device_name = "Quadrature Decoder Rev.2"
        self.machine = machine
        self.feed_pipe = pipe
        #self.synchronizer = synchronizer

    def run(self):
        current_thread().name = self.main_thread_name
        pncLibrary.getSynchronizer(self, self.feed_pipe)
        pncLibrary.waitForThreadStart(self, EncoderInterface, SpindleInterface)

        self.synchronizer.di_startup_event.set()
        self.synchronizer.process_start_signal.wait()
        time.clock()
        #return
        while self.synchronizer.p_run_device_interface_event.is_set():
            try:
                command = self.synchronizer.q_device_interface_command_queue.get(True, pncLibrary.queue_wait_timeout)
                self.routeCommand(command)
            except Empty:
                pass

    def routeCommand(self, command):
        if command.target_device == 'decoder_board':
            self.encoder_interface.command_queue.put(command)
        elif command.target_device == 'spindle_drive':
            self.spindle_interface.command_queue.put(command)

        # try:
        #     # If I remember correctly, this is only necessary because opening the port in __init__(...) caused some picklability error
        #     # if self.serial_port.is_open:
        #     #     pncLibrary.printStringToTerminalMessageQueue('serial port already open')
        #     #     self.serial_port.close()
        #     #     time.sleep(0.5)
        #     #     self.serial_port.open()
        #     # else:
        #     #     self.serial_port.open()
        #     # time.sleep(0.5)
        #     #
        #     # if not self.acknowledgeBoot():
        #     #     print('Decoder board not responsive, terminating process')
        #     #     # FIXME how does this work?
        #     #     raise IOError
        #     # else:
        #     #     #pncLibrary.printTerminalString(self.machine.device_boot_string, self.device_name, self.serial_port.name)
        #     #     pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.device_boot_string, self.device_name, self.serial_port.name)
        #     #     self.synchronizer.ei_encoder_comm_init_event.set()
        #
        #     self.synchronizer.ei_startup_event.set()
        #     self.synchronizer.process_start_signal.wait()
        #     time.clock()
        #
        #     if not self.syncEncoderToStepgen(self.machine.encoder_event_wait_timeout):
        #         raise TimeoutError
        #
        #     self.initializeBaudrate(self.machine.target_baudrate)
        #
        #     if self.synchronizer.p_enable_encoder_event.is_set():
        #         self.synchronizer.ei_successful_start_event.set()
        #         self.synchronizer.p_run_encoder_interface_event.wait()
        #         pncLibrary.printTerminalString(self.machine.device_comm_initialization_string, self.device_name, self.name)
        #         while self.synchronizer.p_run_encoder_interface_event.is_set():
        #             #FIXME should this be on a timer?
        #             #FIXME move to getEncoderCounts
        #             #encoder_position_buffer = []
        #             #encoder_time_buffer = []
        #             raise serial.SerialException
        #
        #             encoder_buffer_data_flag = True
        #             for read_count in range(0, self.machine.encoder_read_buffer_size):
        #                 request_time = time.time()
        #                 encoder_counts = self.getEncoderCounts('current')
        #
        #                 if encoder_counts[0]:
        #                     self.encoder_position_buffer[read_count] = np.asarray(self.countsToPositions(self.machine.axes, encoder_counts[1]))
        #                     self.encoder_time_buffer[read_count] = np.asarray(pncLibrary.estimateMachineClock(self.machine, encoder_counts[2]))
        #                     encoder_buffer_data_flag = encoder_buffer_data_flag and True
        #                 else:
        #                     print('had bad encoder reading')
        #                     encoder_buffer_data_flag = False
        #                     #FIXME should probably push packets of data instead of every sample
        #
        #             if self.synchronizer.mc_xenomai_clock_sync_event.is_set() and encoder_buffer_data_flag:
        #                 #encoder_data_record = {'ENCODER_FEEDBACK_POSITIONS': np.asarray([self.countsToPositions(self.machine.axes, encoder_counts[1])]), 'SERIAL_RECEIVED_TIMES': np.array([[pncLibrary.estimateMachineClock(self.machine, encoder_counts[2])]])}
        #                 encoder_data_record = {'ENCODER_FEEDBACK_POSITIONS': self.encoder_position_buffer, 'SERIAL_RECEIVED_TIMES': self.encoder_time_buffer}
        #                 self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push',[encoder_data_record]))
        #
        # except serial.SerialException as error:
        #     #print('Could not open serial port, error: ' + str(error))
        #     self.synchronizer.ei_startup_event.set()
        #     pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.connection_failed_string+str(error), self.machine.comm_port)
        #     pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
        #                                                  self.machine.process_self_terminate_string,
        #                                                  self.name,
        #                                                  self.pid)
        #
        # else:
        #     pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
        #                                                  self.machine.connection_close_string, self.device_name,
        #                                                  self.machine.comm_port)
        #
        # ## FIXME need to __del__() the port?
        # self.serial_port.close()
        #pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.connection_close_string, self.device_name, self.machine.comm_port)

    # def initializeEncoderCommunication(self):
    #     if self.serial_port.is_open:
    #         pncLibrary.printStringToTerminalMessageQueue('serial port already open')
    #         self.serial_port.close()
    #         time.sleep(0.5)
    #         self.serial_port.open()
    #     else:
    #         self.serial_port.open()
    #     time.sleep(0.5)
    #
    #     if not self.acknowledgeBoot():
    #         print('ENCODER INTERFACE: Decoder board not responsive, terminating process')
    #         raise IOError
    #     else:
    #         pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
    #                                                      self.machine.device_boot_string, self.device_name,
    #                                                      self.serial_port.name)
    #         self.synchronizer.ei_encoder_comm_init_event.set()
    #         self.synchronizer.di_encoder_comm_init_failed_event.set()
    #
    # def syncEncoderToStepgen(self, timeout = None):
    #     try:
    #         self.synchronizer.mc_initial_stepgen_position_set_event.wait(timeout)
    #         self.setAllEncoderCounts(self.machine.axes, self.positionsToCounts(self.machine.axes, self.machine.current_stepgen_position))
    #         self.synchronizer.ei_encoder_init_event.set()
    #         return True
    #     except Exception as error:
    #         self.synchronizer.q_print_server_message_queue.put("ENCODER INTERFACE: Wait on stepgen sync timed out, error: " + str(error))
    #         return False
    #
    # def initializeBaudrate(self, baudrate):
    #     self.serial_port.readline()
    #     self.serial_port.reset_input_buffer()
    #     self.setBaudrate(baudrate)
    #     self.serial_port.reset_input_buffer()
    #
    # ########################### UTILITIES ###########################
    # def countBytesInTransmission(self, transmission):
    #     return math.floor(round(math.log(transmission, 10), 6)) + 1
    #
    # def positionToCounts(self, axis, position):
    #     axis_index = self.machine.axes.index(axis.upper())
    #     return round(position / self.machine.encoder_scale[axis_index]) + self.machine.encoder_offset[axis_index]
    #
    # def positionsToCounts(self, axes, positions):
    #     counts = []
    #     for axis in range(0,len(axes)):
    #         axis_index = self.machine.axes.index(axes[axis].upper())
    #         counts.append(int(round(positions[axis] / self.machine.encoder_scale[axis_index]) + self.machine.encoder_offset[axis_index]))
    #     return np.asarray(counts)
    #
    # def countsToPositions(self, axes, counts):
    #     positions = []
    #     for axis in range(0, len(axes)):
    #         axis_index = self.machine.axes.index(axes[axis].upper())
    #         positions.append((counts[axis] - self.machine.encoder_offset[axis_index]) * self.machine.encoder_scale[axis_index])# + self.machine.machine_table_center_zero[axis_index])
    #     return positions
    #
    # def waitForBytes(self, number_of_bytes, timeout = 0.5):
    #     read_byte_count = 0
    #     read_data = ''
    #     start_time = time.clock()
    #     while read_byte_count < number_of_bytes and (time.clock() - start_time) < timeout:
    #         read_data += self.serial_port.read(1).decode('utf-8')
    #         read_byte_count += 1
    #
    #     if read_byte_count == number_of_bytes:
    #         return (True, read_data)
    #     else:
    #         return (False, None)
    #
    # def waitForString(self, string, timeout = 0.5):
    #     ## FIXME need a timeout here in case sync fails
    #     start = time.clock()
    #     read_data = self.serial_port.readline().decode('utf-8')
    #     #self.synchronizer.q_print_server_message_queue.put('read took ' + str(time.clock()-start))
    #     if string in read_data:
    #         return (True, read_data)
    #     else:
    #         return (False, None)
    #
    # def rebootDevice(self):
    #     self.serial_port.setDTR(0)
    #     self.serial_port.setDTR(1)
    #     time.sleep(0.5)
    #
    # ########################### COMMS ###########################
    # def read(self):
    #     line = self.serial_port.readline()
    #     return line
    #
    # def writeUnicode(self, string):
    #     self.serial_port.write(string.encode('utf-8'))
    #     #time.sleep(0.01)
    #     #self.serial_port.flush()
    #
    # def acknowledgeBoot(self):
    #     return self.waitForString(self.machine.encoder_ack_strings[0])[0]
    #
    # def setAxisEncoderCount(self, axis, count):
    #     command_string = 'S' + str(axis) + str(self.countBytesInTransmission(count)) + str(count)
    #     self.writeUnicode(command_string)
    #     if self.waitForString(self.machine.encoder_ack_strings[1])[0]:
    #         return True
    #     else:
    #         return False
    #
    # def setAllEncoderCounts(self, axes, counts):
    #     for axis in axes:
    #         time.sleep(0.05)
    #         axis_index = self.machine.axes.index(axis)
    #         while not self.setAxisEncoderCount(axis_index+1,counts[axis_index]):
    #             print('ENCODER INTERFACE: Retrying set axis encoder count for axis ' + axis)
    #
    # def getEncoderCounts(self, request_type):
    #     #print('requesting encoder count')
    #     #self.serial_port.write('G'.encode('utf-8'))
    #     if request_type.upper() == 'STORED':
    #         request_string = self.machine.encoder_command_strings[1]
    #         ack_string = self.machine.encoder_ack_strings[-1]
    #     elif request_type.upper() == 'CURRENT':
    #         request_string = self.machine.encoder_command_strings[2]
    #         ack_string = self.machine.encoder_ack_strings[-1]
    #     else:
    #         return (False, None)
    #
    #     self.writeUnicode(request_string)
    #     encoder_bytes = self.serial_port.read(self.machine.max_encoder_transmission_length)
    #     rx_received_time = time.time()
    #     encoder_data = struct.unpack('!' + 'c' + self.machine.number_of_encoders * 'I' + 'c', encoder_bytes)
    #     if encoder_data[0].decode() == request_string and encoder_data[-1].decode() == ack_string:
    #         return (True, encoder_data[1:-1], rx_received_time)
    #     return(False, None)
    #
    # def setBaudrate(self, baudrate):
    #     #ack = (False, False)
    #     #while not ack[0]:
    #     self.writeUnicode(self.machine.encoder_command_strings[3] + str(self.countBytesInTransmission(baudrate)) + str(baudrate))
    #     ack = self.waitForString(self.machine.encoder_ack_strings[4])
    #     #FIXME why the fuck did this break itself
    #     #return True
    #     if int(ack[1].split(' ')[1]) == baudrate:
    #         time.sleep(0.5)
    #         self.serial_port.baudrate = baudrate
    #         self.machine.baudrate = baudrate
    #         print('SUCCESS: Decoder board baudrate is now ' + str(baudrate))




# class EncoderInterface(Process):
#     def __init__(self, machine, pipe):
#         super(EncoderInterface, self).__init__()
#         self.name = "encoder_interface"
#         self.main_thread_name = self.name + ".MainThread"
#         self.device_name = "Quadrature Decoder Rev.2"
#         self.machine = machine
#         self.feed_pipe = pipe
#         #self.synchronizer = synchronizer
#
#         self.serial_port = serial.Serial()
#         self.serial_port.port = self.machine.comm_port
#         self.serial_port.baudrate = self.machine.initial_baudrate
#         self.serial_port.timeout = self.machine.serial_read_timeout
#
#         self.encoder_reads = 0
#         self.encoder_reading_time = 0
#         self.encoder_read_time_moving_average = 0
#
#         self.encoder_position_buffer = np.zeros((self.machine.encoder_read_buffer_size, self.machine.number_of_joints))
#         self.encoder_time_buffer = np.zeros((self.machine.encoder_read_buffer_size, 1))
#
#     def run(self):
#         current_thread().name = self.main_thread_name
#         pncLibrary.getSynchronizer(self, self.feed_pipe)
#
#         try:
#             # If I remember correctly, this is only necessary because opening the port in __init__(...) caused some picklability error
#             if self.serial_port.is_open:
#                 pncLibrary.printStringToTerminalMessageQueue('serial port already open')
#                 self.serial_port.close()
#                 time.sleep(0.5)
#                 self.serial_port.open()
#             else:
#                 self.serial_port.open()
#             time.sleep(0.5)
#
#             if not self.acknowledgeBoot():
#                 print('Decoder board not responsive, terminating process')
#                 # FIXME how does this work?
#                 raise IOError
#             else:
#                 #pncLibrary.printTerminalString(self.machine.device_boot_string, self.device_name, self.serial_port.name)
#                 pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.device_boot_string, self.device_name, self.serial_port.name)
#                 self.synchronizer.ei_encoder_comm_init_event.set()
#
#             self.synchronizer.ei_startup_event.set()
#             self.synchronizer.process_start_signal.wait()
#             time.clock()
#
#             if not self.syncEncoderToStepgen(self.machine.encoder_event_wait_timeout):
#                 raise TimeoutError
#
#             self.initializeBaudrate(self.machine.target_baudrate)
#
#
#             if self.synchronizer.t_enable_encoder_event.is_set():
#                 self.synchronizer.ei_successful_start_event.set()
#                 self.synchronizer.t_run_encoder_interface_event.wait()
#                 pncLibrary.printTerminalString(self.machine.device_comm_initialization_string, self.device_name, self.name)
#                 while self.synchronizer.p_run_encoder_interface_event.is_set():
#                     #FIXME should this be on a timer?
#                     #FIXME move to getEncoderCounts
#                     #encoder_position_buffer = []
#                     #encoder_time_buffer = []
#                     raise serial.SerialException
#
#                     encoder_buffer_data_flag = True
#                     for read_count in range(0, self.machine.encoder_read_buffer_size):
#                         request_time = time.time()
#                         encoder_counts = self.getEncoderCounts('current')
#
#                         if encoder_counts[0]:
#                             self.encoder_position_buffer[read_count] = np.asarray(self.countsToPositions(self.machine.axes, encoder_counts[1]))
#                             self.encoder_time_buffer[read_count] = np.asarray(pncLibrary.estimateMachineClock(self.machine, encoder_counts[2]))
#                             encoder_buffer_data_flag = encoder_buffer_data_flag and True
#                         else:
#                             print('had bad encoder reading')
#                             encoder_buffer_data_flag = False
#                             #FIXME should probably push packets of data instead of every sample
#
#                     if self.synchronizer.mc_xenomai_clock_sync_event.is_set() and encoder_buffer_data_flag:
#                         #encoder_data_record = {'ENCODER_FEEDBACK_POSITIONS': np.asarray([self.countsToPositions(self.machine.axes, encoder_counts[1])]), 'SERIAL_RECEIVED_TIMES': np.array([[pncLibrary.estimateMachineClock(self.machine, encoder_counts[2])]])}
#                         encoder_data_record = {'ENCODER_FEEDBACK_POSITIONS': self.encoder_position_buffer, 'SERIAL_RECEIVED_TIMES': self.encoder_time_buffer}
#                         self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('push',[encoder_data_record]))
#
#         except serial.SerialException as error:
#             #print('Could not open serial port, error: ' + str(error))
#             self.synchronizer.ei_startup_event.set()
#             pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.connection_failed_string+str(error), self.machine.comm_port)
#             pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
#                                                          self.machine.process_self_terminate_string,
#                                                          self.name,
#                                                          self.pid)
#         # except Exception as error:
#         #     pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.process_error_string, self.name, str(error))
#         #     pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
#         #                                                  self.machine.process_self_terminate_string,
#         #                                                  self.machine.name,
#         #                                                  self.machine.comm_port)
#         else:
#             pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
#                                                          self.machine.connection_close_string, self.device_name,
#                                                          self.machine.comm_port)
#
#         ## FIXME need to __del__() the port?
#         self.serial_port.close()
#         #pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, self.machine.connection_close_string, self.device_name, self.machine.comm_port)
#
#     def syncEncoderToStepgen(self, timeout = None):
#         try:
#             #self.synchronizer.mc_initial_stepgen_position_set_event.wait(timeout)
#             self.synchronizer.mc_initial_stepgen_position_set_event.wait(timeout)
#             self.setAllEncoderCounts(self.machine.axes, self.positionsToCounts(self.machine.axes, self.machine.current_stepgen_position))
#             self.synchronizer.ei_encoder_init_event.set()
#             return True
#         except Exception as error:
#             self.synchronizer.q_print_server_message_queue.put("ENCODER INTERFACE: Wait on stepgen sync timed out, error: " + str(error))
#             return False
#
#     def initializeBaudrate(self, baudrate):
#         #self.serial_port.readline()
#         self.serial_port.readline()
#         self.serial_port.reset_input_buffer()
#         self.setBaudrate(baudrate)
#         self.serial_port.reset_input_buffer()
#
#     ########################### UTILITIES ###########################
#     def countBytesInTransmission(self, transmission):
#         return math.floor(round(math.log(transmission, 10), 6)) + 1
#
#     def positionToCounts(self, axis, position):
#         axis_index = self.machine.axes.index(axis.upper())
#         return round(position / self.machine.encoder_scale[axis_index]) + self.machine.encoder_offset[axis_index]
#
#     def positionsToCounts(self, axes, positions):
#         counts = []
#         for axis in range(0,len(axes)):
#             axis_index = self.machine.axes.index(axes[axis].upper())
#             counts.append(int(round(positions[axis] / self.machine.encoder_scale[axis_index]) + self.machine.encoder_offset[axis_index]))
#         return np.asarray(counts)
#
#     def countsToPositions(self, axes, counts):
#         positions = []
#         for axis in range(0, len(axes)):
#             axis_index = self.machine.axes.index(axes[axis].upper())
#             positions.append((counts[axis] - self.machine.encoder_offset[axis_index]) * self.machine.encoder_scale[axis_index])# + self.machine.machine_table_center_zero[axis_index])
#         return positions
#
#     def waitForBytes(self, number_of_bytes, timeout = 0.5):
#         read_byte_count = 0
#         read_data = ''
#         start_time = time.clock()
#         while read_byte_count < number_of_bytes and (time.clock() - start_time) < timeout:
#             read_data += self.serial_port.read(1).decode('utf-8')
#             read_byte_count += 1
#
#         if read_byte_count == number_of_bytes:
#             return (True, read_data)
#         else:
#             return (False, None)
#
#     def waitForString(self, string, timeout = 0.5):
#         ## FIXME need a timeout here in case sync fails
#         start = time.clock()
#         read_data = self.serial_port.readline().decode('utf-8')
#         #self.synchronizer.q_print_server_message_queue.put('read took ' + str(time.clock()-start))
#         if string in read_data:
#             return (True, read_data)
#         else:
#             return (False, None)
#
#     def rebootDevice(self):
#         self.serial_port.setDTR(0)
#         self.serial_port.setDTR(1)
#         time.sleep(0.5)
#
#     ########################### COMMS ###########################
#     def read(self):
#         line = self.serial_port.readline()
#         return line
#
#     def writeUnicode(self, string):
#         self.serial_port.write(string.encode('utf-8'))
#         #time.sleep(0.01)
#         #self.serial_port.flush()
#
#     def acknowledgeBoot(self):
#         return self.waitForString(self.machine.encoder_ack_strings[0])[0]
#
#     def setAxisEncoderCount(self, axis, count):
#         command_string = 'S' + str(axis) + str(self.countBytesInTransmission(count)) + str(count)
#         self.writeUnicode(command_string)
#         if self.waitForString(self.machine.encoder_ack_strings[1])[0]:
#             return True
#         else:
#             return False
#
#     def setAllEncoderCounts(self, axes, counts):
#         for axis in axes:
#             time.sleep(0.05)
#             axis_index = self.machine.axes.index(axis)
#             while not self.setAxisEncoderCount(axis_index+1,counts[axis_index]):
#                 print('ENCODER INTERFACE: Retrying set axis encoder count for axis ' + axis)
#
#     def getEncoderCounts(self, request_type):
#         #print('requesting encoder count')
#         #self.serial_port.write('G'.encode('utf-8'))
#         if request_type.upper() == 'STORED':
#             request_string = self.machine.encoder_command_strings[1]
#             ack_string = self.machine.encoder_ack_strings[-1]
#         elif request_type.upper() == 'CURRENT':
#             request_string = self.machine.encoder_command_strings[2]
#             ack_string = self.machine.encoder_ack_strings[-1]
#         else:
#             return (False, None)
#
#         self.writeUnicode(request_string)
#         encoder_bytes = self.serial_port.read(self.machine.max_encoder_transmission_length)
#         rx_received_time = time.time()
#         encoder_data = struct.unpack('!' + 'c' + self.machine.number_of_encoders * 'I' + 'c', encoder_bytes)
#         if encoder_data[0].decode() == request_string and encoder_data[-1].decode() == ack_string:
#             return (True, encoder_data[1:-1], rx_received_time)
#         return(False, None)
#
#     def setBaudrate(self, baudrate):
#         #ack = (False, False)
#         #while not ack[0]:
#         self.writeUnicode(self.machine.encoder_command_strings[3] + str(self.countBytesInTransmission(baudrate)) + str(baudrate))
#         ack = self.waitForString(self.machine.encoder_ack_strings[4])
#         #FIXME why the fuck did this break itself
#         #return True
#         if int(ack[1].split(' ')[1]) == baudrate:
#             time.sleep(0.5)
#             self.serial_port.baudrate = baudrate
#             self.machine.baudrate = baudrate
#             print('SUCCESS: Decoder board baudrate is now ' + str(baudrate))