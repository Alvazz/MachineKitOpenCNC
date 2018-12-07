import pncLibrary, time, websocket, json, base64, copy, numpy as np
from multiprocessing import Queue, Event, Process, current_process
from threading import Thread, current_thread
from queue import Empty
from io import BytesIO

class WebsocketCommunicatorAchex:
    def __init__(self, parent):
        self.parent = parent
        self.name = "websocket_communicator_achex"
        self.tp_websocket = self.parent.tp_websocket
        self.machine = self.parent.machine
        self.synchronizer = self.parent.synchronizer

    def openConnection(self):
        self.tp_websocket.connect("ws://achex.ca:4010")
        self.tp_websocket.SID = json.loads(self.tp_websocket.recv())['SID']
        self.tp_websocket.send(json.dumps({'setID': self.machine.websocket_username, "passwd": "mypass"}))
        try:
            #auth_message = json.loads(self.tp_websocket.recv())
            self.synchronizer.tp_websocket_auth_received_event.wait(pncLibrary.event_wait_timeout)
            #if auth_message['auth'] == 'ok':
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_websocket_connection_success_string,
                                                         self.machine.websocket_username)
            self.synchronizer.tp_websocket_server_connected_event.set()
            return True
        except ConnectionAbortedError as websocket_error:
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_websocket_connection_failure_string,
                                                         self.name, websocket_error)
            return False
        except Exception as error:
            #Fixme handle the timeout exception
            return False

    def sendMessage(self, **kwargs):
        data_stream = BytesIO()
        #data_stream.seek(0)
        np.savez_compressed(data_stream, **kwargs)
        data_stream.seek(0)
        encoded_string = base64.b64encode(data_stream.read())
        #self.web_socket.send(self.decode_bytes(data_stream))
        self.tp_websocket.send(json.dumps({"to": self.machine.websocket_tp_name, "payload": encoded_string.decode()}))
        return len(encoded_string)

    def sendDataObject(self, data_object):
        bytes_to_send = data_object.encodeNumpy()
        self.tp_websocket.send(json.dumps({"to": self.machine.websocket_tp_name, "payload": bytes_to_send}))
        return len(bytes_to_send)

    def sendDataObjectBySubSID(self, data_object):
        payload_size = data_object.message_data['joint_space_data'].shape[1]
        bytes_sent = 0
        number_of_sub_seqs = int(np.floor(payload_size / self.machine.websocket_block_length))+1
        pncLibrary.updateMachineModelList(self.machine, "tp_state_number_of_subsequences",
                                          number_of_sub_seqs, 0)
        #self.machine.tp_state_number_of_subsequences[0] = number_of_sub_seqs

        for sub_sid in range(0, number_of_sub_seqs):
            begin_index = sub_sid*self.machine.websocket_block_length
            end_index = [(sub_sid + 1) * self.machine.websocket_block_length if (sub_sid + 1) * self.machine.websocket_block_length < payload_size else payload_size][0]

            sub_sequence = copy.deepcopy(data_object)
            sub_sequence.message_data['sid'][1] = sub_sid
            sub_sequence.message_data['number_of_subsequences'] = np.array([number_of_sub_seqs, -1])

            sub_sequence.message_data['joint_space_data'] = data_object.message_data['joint_space_data'][:,begin_index:end_index]
            sub_sequence.message_data['tool_space_data'] = data_object.message_data['tool_space_data'][:,begin_index:end_index]
            sub_sequence.message_data['volumes_removed'] = data_object.message_data['volumes_removed'][:,begin_index:end_index]
            #sub_sequence.message_data['move_flags'] = data_object.message_data['move_flags'][:,begin_index:end_index]
            bytes_to_send = sub_sequence.encodeNumpy()

            self.synchronizer.tp_subsequence_id_ack_event.clear()
            pncLibrary.updateMachineModelList(self.machine, "tp_state_current_requested_subsequence_id",
                                              sub_sid, pncLibrary.tp_move_types.index(sub_sequence.message_data['move_type']))
            #self.tp_state.current_requested_subsequence_id[pncLibrary.tp_move_types.index(sub_sequence.message_data['move_type'])] = sub_sid

            self.tp_websocket.send(json.dumps({"to": self.machine.websocket_tp_name, "payload": bytes_to_send}))
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_trajectory_plan_subsequenced_request_sent_string,
                                                         self.name, len(bytes_to_send), sub_sequence.message_data['sid'][1], sub_sequence.message_data['sid'][0])
            self.synchronizer.tp_subsequence_id_ack_event.wait()

            bytes_sent += len(bytes_to_send)
            sub_sid += 1
        self.synchronizer.tp_sequence_id_ack_event.wait()
        return bytes_sent

    def receiveMessage(self):
        received_paylod = json.loads(self.tp_websocket.recv())
        if 'payload' in received_paylod.keys():
            return pncLibrary.TPData().decodeNumpy(received_paylod['payload'])
        elif 'auth' in received_paylod.keys():
            if received_paylod['auth'] == 'ok':
                self.synchronizer.tp_websocket_auth_received_event.set()

        #except Exception as r:
        #print('receive break')
        #received_data = pncLibrary.TPData()
        #return pncLibrary.TPData().decodeNumpy(received_paylod)
        # return
        # return json.loads(self.tp_websocket.recv())['payload']

class WebsocketCommunicatorNinja:
    def __init__(self, parent):
        self.parent = parent
        self.name = "websocket_communicator_ninja"
        self.tp_websocket = self.parent.tp_websocket
        self.machine = self.parent.machine
        self.synchronizer = self.parent.synchronizer

    def openConnection(self):
        self.tp_websocket.connect("wss://node2.wsninja.io")
        self.tp_websocket.send(json.dumps({'guid': self.machine.websocket_client_GUID}))
        ack_message = json.loads(self.tp_websocket.recv())
        if not ack_message['accepted']:
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_websocket_connection_failure_string,
                                                         self.name, 'Websocket connection not accepted')
            return False
        else:
            self.synchronizer.tp_websocket_server_connected_event.set()
            return True

    def sendMessage(self, **kwargs):
        data_stream = BytesIO()
        np.savez_compressed(data_stream, **kwargs)
        data_stream.seek(0)
        encoded_string = base64.b64encode(data_stream.read())
        self.tp_websocket.send(encoded_string)
        return len(encoded_string)

    def receiveMessage(self):
        return self.tp_websocket.recv()

# class CloudTrajectoryPlannerInterface(Thread):
#     def __init__(self, parent):
#         super(CloudTrajectoryPlannerInterface, self).__init__()
#         self.name = "cloud_trajectory_planner"
#         self.machine = parent.machine
#         self.toolpath_data = None
#         self.remote_tp_name = "MukulLab"
#         self.synchronizer = parent.synchronizer
#         self.tp_websocket = websocket.WebSocket()
#         self.tp_websocket.timeout = self.machine.websocket_timeout
#
#         self.tp_state = pncLibrary.CloudTrajectoryPlannerState()
#
#         if self.machine.websocket_type == 'achex':
#             self.websocket_communicator = WebsocketCommunicatorAchex(self)
#         elif self.machine.websocket_type == 'wsninja':
#             self.websocket_communicator = WebsocketCommunicatorNinja(self)
#
#         self.raw_point_queue = Queue()
#         self.planned_point_output_queue = Queue()
#         self.startup_event = Event()
#
#         self.planned_point_payload_buffer = []
#         self.CAM_point_buffer = []
#         self.requested_point_payload_buffer = []
#         self.planning_time_log = []
#
#         self.starting_sequence_id = 0
#         self.ending_sequence_id = -1
#         self.plan_to_index_delta = 0
#         self.conversion_factor_dist = 1
#         self.send_time = 0
#
#     def run(self):
#         pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
#                                                      self.machine.thread_launch_string, current_process().name,
#                                                      self.name)
#         pncLibrary.waitForThreadStart(self, WebsocketReceiver)
#         self.startup_event.set()
#         #return
#         while not self.websocket_communicator.openConnection():
#             pass
#
#         if self.connectToTP():
#
#             self.synchronizer.tp_toolpath_setup_event.wait()
#             self.sendStartupMetadata()
#
#             while self.synchronizer.t_run_cloud_trajectory_planner_event.is_set() and self.tp_state.tp_connected_event.is_set():
#                 try:
#                     self.requestToolpathPointsFromCAM()
#                     if self.tp_state.send_next_block_event.wait(self.machine.event_wait_timeout) and self.synchronizer.tp_plan_motion_event.wait(self.machine.event_wait_timeout):
#                         #if (self.tp_state.current_requested_CAM_sequence_id-self.starting_sequence_id) < self.plan_to_index_delta:
#                         trajectory_to_plan = self.raw_point_queue.get(True, pncLibrary.queue_move_queue_wait_timeout)
#                         move_type_index = pncLibrary.tp_move_types.index(trajectory_to_plan.message_data['move_type'])
#                         self.tp_state.sequence_id_ack_event.clear()
#                         self.tp_state.matching_sequence_received_event.clear()
#
#                         self.synchronizer.tp_planning_finished_event.clear()
#                         self.tp_state.data_flushed_to_websocket_event.clear()
#
#                         # print('requested sequence id was: ' + str(
#                         #     self.tp_state.current_requested_sequence_id[move_type_index]) + ' incrementing to ' + str(
#                         #     trajectory_to_plan.message_data['sid'][0]))
#                         self.tp_state.current_requested_sequence_id[move_type_index] = \
#                         trajectory_to_plan.message_data['sid'][0]
#
#
#                         self.send_time = time.time()
#                         self.tp_state.send_next_block_event.clear()
#                         self.sendPlanningRequest(trajectory_to_plan)
#                         self.requested_point_payload_buffer.append(trajectory_to_plan)
#
#                 except Empty:
#                     print('TP waiting on another request')
#                 #     if self.tp_state.current_received_sequence_id[0] == self.tp_state.current_requested_sequence_id[0]:
#                 #     #if self.tp_state.current_received_CAM_sequence_id == self.tp_state.current_requested_CAM_sequence_id:
#                 #         if not self.synchronizer.tp_planning_finished_event.is_set():
#                 #             pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
#                 #                                                          pncLibrary.printout_trajectory_planning_finished_string,
#                 #                                                          self.tp_state.current_received_sequence_id[0])
#                 #             self.synchronizer.tp_planning_finished_event.set()
#                 #
#                 #         if self.synchronizer.q_trajectory_planner_planned_move_queue.empty():
#                 #             self.tp_state.all_moves_consumed_event.set()
#                 #             #self.synchronizer.db_write_to_websocket_event.set()
#                 #             if not self.tp_state.data_flushed_to_websocket_event.is_set() and self.synchronizer.mc_motion_complete_event.is_set():
#                 #                 self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('flush_to_websocket', self.machine.motion_start_time))
#                 #                 self.tp_state.data_flushed_to_websocket_event.set()
#                 #
#                 # finally:
#                 #     self.flushDataFeedbackToTP()
#
#         pncLibrary.waitForThreadStop(self, self.cloud_trajectory_planner_receiver)
#         self.tp_websocket.close()
#         pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
#                                                      pncLibrary.printout_thread_terminate_string, current_process().name,
#                                                      self.name)
#
#     def connectToTP(self):
#         #self.websocket_communicator.sendMessage(hello='HELLO')
#         self.websocket_communicator.sendDataObject(pncLibrary.TPData(message_type='HELLO'))
#         if self.synchronizer.tp_connected_event.wait(self.machine.event_wait_timeout):
#             pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
#                                                          pncLibrary.printout_trajectory_planner_connection_string,
#                                                          self.name, self.remote_tp_name)
#             self.tp_state.send_next_block_event.set()
#             return True
#         else:
#             pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
#                                                          pncLibrary.printout_trajectory_planner_connection_failure_string,
#                                                          self.name, self.remote_tp_name)
#             return False
#
#     def initializeToolpathData(self):
#         self.tool_points, self.machine_points, self.retraction_flags = pncLibrary.TP.load_sp_data(
#             self.machine.raw_point_files_path, self.joint_data_file, self.tool_data_file)
#         self.contiguous_sequences = pncLibrary.TP.obtain_contiguous_sequences(self.retraction_flags.astype(int))
#
#     def sendPlanningRequest(self, trajectory):
#         #data_size = self.websocket_communicator.sendMessage(sid=points_to_plan[0], tool=points_to_plan[1], joint=points_to_plan[2], move_type=points_to_plan[3])
#         #data_size = self.websocket_communicator.sendDataObject(trajectory)
#         data_size = self.websocket_communicator.sendDataObjectBySubSID(trajectory)
#         pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
#                                                      pncLibrary.printout_trajectory_plan_request_sent_string,
#                                                      self.name, data_size, trajectory.message_data['sid'][0])
#
#     def resetWebsocketBuffer(self):
#         self.websocket_communicator.sendMessage(reset='RESET')
#         while 'RESET' not in self.tp_websocket.recv():
#             pass
#
#     def requestToolpathPointsFromCAM(self):
#         print('enqueued sequence ID is ' + str(self.tp_state.enqueued_sequence_id) + ' currently_executing_sequence_id is ' + str(self.machine.currently_executing_sequence_id[0]))
#         if (self.tp_state.enqueued_sequence_id - self.machine.currently_executing_sequence_id[0]) <= self.machine.toolpath_point_buffer_length:
#         #if self.machine.toolpath_point_buffer_length > self.synchronizer.q_trajectory_planner_planned_move_queue.qsize():
#             self.synchronizer.tp_need_points_event.set()
#             self.tp_state.flag_set_count+=1
#             print('setting need_points flag for the ' + str(self.tp_state.flag_set_count) + ' time with delta = ' + str((self.tp_state.enqueued_sequence_id - self.machine.currently_executing_sequence_id[0])))
#         else:
#             self.synchronizer.tp_need_points_event.clear()
#
#     def enqueueSequencesForPlanning(self, tool_space_data=None, joint_space_data=None, sequence_slices=None, begin_sequence=None, end_sequence=None):#, output_queue=None):
#         if tool_space_data is None:
#             tool_space_data = self.tool_points
#         if joint_space_data is None:
#             joint_space_data = self.machine_points
#         if sequence_slices is None:
#             sequence_slices = self.contiguous_sequences
#         if begin_sequence is None:
#             begin_sequence = self.starting_sequence_id
#         if end_sequence is None:
#             end_sequence = self.ending_sequence_id if self.ending_sequence_id >= 0 else len(self.contiguous_sequences)
#
#         pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
#                                                      pncLibrary.printout_trajectory_planner_enqueueing_voxel_points_string,
#                                                      str(end_sequence-begin_sequence+1), self.sp_file_name)
#         #enqueue_point_list = []
#         for k in range(begin_sequence, end_sequence+1):
#             move_type = sequence_slices[k][0]
#             #self.tp_state.enqueued_sequence_id += 1
#             self.tp_state.enqueued_sequence_id = k
#             points_to_enqueue = [np.array([self.tp_state.enqueued_sequence_id]), tool_space_data[:, sequence_slices[k][1]], joint_space_data[:, sequence_slices[k][1]], 'SP_trajectory']
#
#             #Hack hack the start/end points of cuts into the rapids
#             if move_type == 1 and k-begin_sequence > 0:
#                 #tool_start_points = enqueue_point_list[k-begin_sequence-1][1][0]
#                 tool_start_points = tool_space_data[:, sequence_slices[k-1][1]].T[-1]
#                 tool_start_points[-1] = move_type
#                 #joint_start_points = enqueue_point_list[k - begin_sequence - 1][2][0]
#                 joint_start_points = joint_space_data[:, sequence_slices[k-1][1]].T[-1]
#                 joint_start_points[-1] = move_type
#                 points_to_enqueue[1] = np.hstack((np.array([tool_start_points]).T, points_to_enqueue[1]))
#                 points_to_enqueue[2] = np.hstack((np.array([joint_start_points]).T, points_to_enqueue[2]))
#
#                 if k < end_sequence-1:
#                     tool_end_points = tool_space_data[:, sequence_slices[k+1][1]].T[0]
#                     tool_end_points[-1] = move_type
#                     joint_end_points = joint_space_data[:, sequence_slices[k+1][1]].T[0]
#                     joint_end_points[-1] = move_type
#                     points_to_enqueue[1] = np.hstack((points_to_enqueue[1], np.array([tool_end_points]).T))
#                     points_to_enqueue[2] = np.hstack((points_to_enqueue[2], np.array([joint_end_points]).T))
#
#             self.raw_point_queue.put(points_to_enqueue)
#
#     def extractLatestPlanRequest(self, requested_sequence_id):
#         for request in self.requested_point_payload_buffer:
#             if request[0] == requested_sequence_id:
#                 return request
#
#     def extractStartingVoxelPoints(self):
#         #return self.machine_points.T[self.contiguous_sequences[self.starting_sequence_id],:5]
#         #return pncLibrary.TP.rotaryAxesToDegrees(np.array([self.machine_points.T[self.contiguous_sequences[self.starting_sequence_id]][:self.machine.number_of_joints]]))
#         #return pncLibrary.TP.rotaryAxesToDegrees(np.array([self.cloud_trajectory_planner_receiver.planned_point_payload_buffer[0]['joint_space_data'].T[0,:self.machine.number_of_joints]]))[0]
#         starting_points = copy.deepcopy(self.cloud_trajectory_planner_receiver.planned_point_payload_buffer[0]['joint_space_data'].T[0,:self.machine.number_of_joints])
#         return pncLibrary.TP.rotaryAxesToDegrees(np.array([starting_points]))[0]
#         #return self.cloud_trajectory_planner_receiver.planned_point_payload_buffer[0]['joint_space_data'].T[0,:self.machine.number_of_joints]
#
#     def flushDataFeedbackToTP(self):
#         while not self.synchronizer.q_trajectory_planner_data_return_queue.empty():
#             data_to_return = self.synchronizer.q_trajectory_planner_data_return_queue.get()
#             self.tp_state.metadata_ack_event.clear()
#             data_size = self.websocket_communicator.sendDataObject(pncLibrary.TPData(
#                 message_type='EXECUTED_DATA',
#                 file_name=self.toolpath_data.sculptprint_file_name,
#                 path_id=self.toolpath_data.toolpath_id,
#                 feedback_data_type=self.machine.servo_log_types[self.machine.servo_log_type],
#                 feedback_period=int(1000 * self.machine.servo_dt * self.machine.servo_log_sub_sample_rate),
#                 starting_sequence=np.array([self.starting_sequence_id]),
#                 ending_sequence=np.array([self.starting_sequence_id+self.plan_to_index_delta]),
#                 motion_start_time=self.machine.motion_start_time,
#                 stepgen_executed_points=data_to_return['stepgen_executed_points'],
#                 encoder_executed_points=data_to_return['encoder_executed_points'],
#                 rt_time=data_to_return['rt_time'],
#                 nonrt_time=data_to_return['nonrt_time'],
#                 buffer_level=data_to_return['buffer_level']))
#
#             pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
#                                                          pncLibrary.printout_websocket_motion_data_sent_string,
#                                                          self.name, data_size, self.starting_sequence_id,
#                                                          self.starting_sequence_id+self.plan_to_index_delta-1)
#
#
#
#     def sendStartupMetadata(self):
#         self.tp_state.metadata_ack_event.clear()
#         meta_size = self.websocket_communicator.sendDataObject(pncLibrary.TPData(message_type='METADATA',
#                                                                                 file_name=self.toolpath_data.sculptprint_file_name,
#                                                                                 path_id=self.toolpath_data.toolpath_id,
#                                                                                 block_length=self.machine.websocket_block_length,
#                                                                                 velocity_limit=0.7*np.vstack((-np.array(self.machine.tp_max_joint_velocity), np.array(self.machine.tp_max_joint_velocity))).T,
#                                                                                 acceleration_limit=0.7*np.vstack((-np.array(self.machine.tp_max_joint_acceleration), np.array(self.machine.tp_max_joint_acceleration))).T,
#                                                                                 servo_dt=self.machine.servo_dt,
#                                                                                 tool_transformation=self.toolpath_data.tool_transformation_matrix,
#                                                                                 part_transformation=self.toolpath_data.work_transformation_matrix,
#                                                                                 mrr_limit=0,
#                                                                                 compute_grid_scale_factor=1))
#                                                                                 #conservative_bounding=True))
#
#         self.tp_state.metadata_ack_event.wait()

class CloudTrajectoryPlannerInterfaceProcess(Process):
    def __init__(self, machine, pipe):
        super(CloudTrajectoryPlannerInterfaceProcess, self).__init__()
        self.name = "cloud_trajectory_planner_interface"
        self.main_thread_name = self.name + ".MainThread"
        self.machine = machine
        self.feed_pipe = pipe

        self.toolpath_data = None
        self.remote_tp_name = "MukulLab"



        #self.tp_state = pncLibrary.CloudTrajectoryPlannerState()

        #self.raw_point_queue = Queue()
        #self.planned_point_output_queue = Queue()
        #self.startup_event = Event()

        self.planned_point_payload_buffer = []
        self.CAM_point_buffer = []
        self.requested_point_payload_buffer = []
        self.planning_time_log = []

        self.starting_sequence_id = 0
        self.ending_sequence_id = -1
        self.plan_to_index_delta = 0
        self.conversion_factor_dist = 1
        self.send_time = 0

    def run(self):
        current_thread().name = self.main_thread_name
        self.tp_websocket = websocket.WebSocket()
        self.tp_websocket.timeout = self.machine.websocket_timeout

        pncLibrary.getSynchronizer(self, self.feed_pipe)
        if self.machine.websocket_type == 'achex':
            self.websocket_communicator = WebsocketCommunicatorAchex(self)
        elif self.machine.websocket_type == 'wsninja':
            self.websocket_communicator = WebsocketCommunicatorNinja(self)

        pncLibrary.waitForThreadStart(self, WebsocketReceiver)

        self.synchronizer.tp_startup_event.set()
        self.synchronizer.process_start_signal.wait()



        # pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
        #                                              self.machine.thread_launch_string, current_process().name,
        #                                              self.name)

        #self.startup_event.set()
        time.clock()

        #return
        if self.synchronizer.p_enable_cloud_trajectory_planner_interface_event.is_set():
            self.synchronizer.tp_successful_start_event.set()
            # self.synchronizer.p_run_machine_controller_event.wait()
            while self.synchronizer.p_run_cloud_trajectory_planner_interface_event.is_set():
                # self.createRapidTrajectoryPlanRequest(*self.generateRapidPoints(end_joint=np.array([0,0,0,0,0])))
                try:
                    command = self.synchronizer.q_cloud_trajectory_planner_interface_command_queue.get(True,
                                                                                       self.machine.process_queue_wait_timeout)
                    self.handleCommand(command)
                except Empty:
                    pass
                except Exception as error:
                    self.synchronizer.q_print_server_message_queue.put("CLOUD TRAJECTORY PLANNER INTERFACE: Had error: " + str(error))
                    pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                                 pncLibrary.printout_trajectory_planner_interface_error_string,
                                                                 str(error))
                finally:
                    self.runTPCycle()

        pncLibrary.waitForThreadStop(self, self.cloud_trajectory_planner_receiver)
        self.tp_websocket.close()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_process_self_terminate_string,
                                                     self.name, current_process().pid)

    def handleCommand(self, command):
        if command.command_type == 'OPEN_CONNECTION':
            if not self.websocket_communicator.openConnection():
                pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                             pncLibrary.printout_trajectory_planner_retrying_open_connection_string,
                                                             self.websocket_communicator.name)
        elif command.command_type == 'CONNECT_TO_TP':
            if self.connectToTP():
                self.synchronizer.tp_connected_event.set()

        elif command.command_type == 'UPDATE_TOOLPATH_DATA':
            self.updateInternalToolpathData(command.command_data)

        elif command.command_type == 'SEND_METADATA':
            self.sendStartupMetadata()

        elif command.command_type == 'UPDATE':
            self.runTPCycle()


    def updateTrajectoryPlannerStateFlags(self):
        self.synchronizer.tp_sequence_id_ack_event.clear()
        self.synchronizer.tp_matching_sequence_received_event.clear()
        self.synchronizer.tp_planning_finished_event.clear()
        self.synchronizer.tp_data_flushed_to_websocket_event.clear()

    def runTPCycle(self):
        if self.synchronizer.tp_connected_event.is_set() and self.synchronizer.tp_metadata_ack_event.is_set():
            self.requestToolpathPointsFromCAM()
            if self.synchronizer.tp_send_next_block_event.is_set() and self.synchronizer.tp_plan_motion_event.is_set():
                try:
                    trajectory_to_plan = self.synchronizer.q_cloud_trajectory_planner_interface_position_point_queue.get(
                        True, pncLibrary.queue_move_queue_wait_timeout)
                    move_type_index = pncLibrary.tp_move_types.index(trajectory_to_plan.message_data['move_type'])
                    self.updateTrajectoryPlannerStateFlags()

                    pncLibrary.updateMachineModelList(self.machine, "tp_state_current_requested_sequence_id",
                                                      trajectory_to_plan.message_data['sid'][0], move_type_index)
                    # self.machine.tp_state.current_requested_sequence_id[move_type_index] = \
                    #     trajectory_to_plan.message_data['sid'][0]

                    self.send_time = time.time()
                    self.synchronizer.tp_send_next_block_event.clear()
                    self.sendPlanningRequest(trajectory_to_plan)
                    self.requested_point_payload_buffer.append(trajectory_to_plan)
                except Empty:
                    print('TP waiting on another request')

            # if self.connectToTP():
            #
            #     self.synchronizer.tp_toolpath_setup_event.wait()
            #     self.sendStartupMetadata()
            #
            #     while self.synchronizer.p_run_cloud_trajectory_planner_event.is_set() and self.tp_state.tp_connected_event.is_set():
            #         try:
            #             self.requestToolpathPointsFromCAM()
            #             if self.tp_state.send_next_block_event.wait(self.machine.event_wait_timeout) and self.synchronizer.tp_plan_motion_event.wait(self.machine.event_wait_timeout):
            #                 #if (self.tp_state.current_requested_CAM_sequence_id-self.starting_sequence_id) < self.plan_to_index_delta:
            #                 trajectory_to_plan = self.synchronizer.q_cloud_trajectory_planner_interface_position_point_queue.get(True, pncLibrary.queue_move_queue_wait_timeout)
            #                 move_type_index = pncLibrary.tp_move_types.index(trajectory_to_plan.message_data['move_type'])
            #                 self.tp_state.sequence_id_ack_event.clear()
            #                 self.tp_state.matching_sequence_received_event.clear()
            #
            #                 self.synchronizer.tp_planning_finished_event.clear()
            #                 self.tp_state.data_flushed_to_websocket_event.clear()
            #
            #                 # print('requested sequence id was: ' + str(
            #                 #     self.tp_state.current_requested_sequence_id[move_type_index]) + ' incrementing to ' + str(
            #                 #     trajectory_to_plan.message_data['sid'][0]))
            #                 self.tp_state.current_requested_sequence_id[move_type_index] = \
            #                 trajectory_to_plan.message_data['sid'][0]
            #
            #
            #                 self.send_time = time.time()
            #                 self.tp_state.send_next_block_event.clear()
            #                 self.sendPlanningRequest(trajectory_to_plan)
            #                 self.requested_point_payload_buffer.append(trajectory_to_plan)
            #
            #         except Empty:
            #             print('TP waiting on another request')
            #         #     if self.tp_state.current_received_sequence_id[0] == self.tp_state.current_requested_sequence_id[0]:
            #         #     #if self.tp_state.current_received_CAM_sequence_id == self.tp_state.current_requested_CAM_sequence_id:
            #         #         if not self.synchronizer.tp_planning_finished_event.is_set():
            #         #             pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
            #         #                                                          pncLibrary.printout_trajectory_planning_finished_string,
            #         #                                                          self.tp_state.current_received_sequence_id[0])
            #         #             self.synchronizer.tp_planning_finished_event.set()
            #         #
            #         #         if self.synchronizer.q_trajectory_planner_planned_move_queue.empty():
            #         #             self.tp_state.all_moves_consumed_event.set()
            #         #             #self.synchronizer.db_write_to_websocket_event.set()
            #         #             if not self.tp_state.data_flushed_to_websocket_event.is_set() and self.synchronizer.mc_motion_complete_event.is_set():
            #         #                 self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('flush_to_websocket', self.machine.motion_start_time))
            #         #                 self.tp_state.data_flushed_to_websocket_event.set()
            #         #
            #         # finally:
            #         #     self.flushDataFeedbackToTP()



    def connectToTP(self):
        #self.websocket_communicator.sendMessage(hello='HELLO')
        self.websocket_communicator.sendDataObject(pncLibrary.TPData(message_type='HELLO'))
        if self.synchronizer.tp_connected_event.wait(self.machine.event_wait_timeout):
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_trajectory_planner_connection_string,
                                                         self.name, self.remote_tp_name)
            self.synchronizer.tp_send_next_block_event.set()
            return True
        else:
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_trajectory_planner_connection_failure_string,
                                                         self.name, self.remote_tp_name)
            return False

    def initializeToolpathData(self):
        self.tool_points, self.machine_points, self.retraction_flags = pncLibrary.TP.load_sp_data(
            self.machine.raw_point_files_path, self.joint_data_file, self.tool_data_file)
        self.contiguous_sequences = pncLibrary.TP.obtain_contiguous_sequences(self.retraction_flags.astype(int))

    def sendPlanningRequest(self, trajectory):
        #data_size = self.websocket_communicator.sendMessage(sid=points_to_plan[0], tool=points_to_plan[1], joint=points_to_plan[2], move_type=points_to_plan[3])
        #data_size = self.websocket_communicator.sendDataObject(trajectory)
        data_size = self.websocket_communicator.sendDataObjectBySubSID(trajectory)
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_trajectory_plan_request_sent_string,
                                                     self.name, data_size, trajectory.message_data['sid'][0])

    def resetWebsocketBuffer(self):
        self.websocket_communicator.sendMessage(reset='RESET')
        while 'RESET' not in self.tp_websocket.recv():
            pass

    def requestToolpathPointsFromCAM(self):
        print('enqueued sequence ID is ' + str(self.machine.tp_state_enqueued_sequence_id) + ' currently_executing_sequence_id is ' + str(self.machine.currently_executing_sequence_id[0]))
        if (self.machine.tp_state_enqueued_sequence_id - self.machine.currently_executing_sequence_id[0]) <= self.machine.toolpath_point_buffer_length:
        #if self.machine.toolpath_point_buffer_length > self.synchronizer.q_trajectory_planner_planned_move_queue.qsize():
            self.synchronizer.tp_need_points_event.set()
            self.machine.tp_state_flag_set_count = self.machine.tp_state_flag_set_count + 1
            print('setting need_points flag for the ' + str(self.machine.tp_state_flag_set_count) + ' time with delta = ' + str((self.machine.tp_state_enqueued_sequence_id - self.machine.currently_executing_sequence_id[0])))
        else:
            self.synchronizer.tp_need_points_event.clear()

    def enqueueSequencesForPlanning(self, tool_space_data=None, joint_space_data=None, sequence_slices=None, begin_sequence=None, end_sequence=None):#, output_queue=None):
        if tool_space_data is None:
            tool_space_data = self.tool_points
        if joint_space_data is None:
            joint_space_data = self.machine_points
        if sequence_slices is None:
            sequence_slices = self.contiguous_sequences
        if begin_sequence is None:
            begin_sequence = self.starting_sequence_id
        if end_sequence is None:
            end_sequence = self.ending_sequence_id if self.ending_sequence_id >= 0 else len(self.contiguous_sequences)

        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_trajectory_planner_enqueueing_voxel_points_string,
                                                     str(end_sequence-begin_sequence+1), self.sp_file_name)
        #enqueue_point_list = []
        for k in range(begin_sequence, end_sequence+1):
            move_type = sequence_slices[k][0]
            #self.tp_state.enqueued_sequence_id += 1
            self.tp_state.enqueued_sequence_id = k
            points_to_enqueue = [np.array([self.tp_state.enqueued_sequence_id]), tool_space_data[:, sequence_slices[k][1]], joint_space_data[:, sequence_slices[k][1]], 'SP_trajectory']

            #Hack hack the start/end points of cuts into the rapids
            if move_type == 1 and k-begin_sequence > 0:
                #tool_start_points = enqueue_point_list[k-begin_sequence-1][1][0]
                tool_start_points = tool_space_data[:, sequence_slices[k-1][1]].T[-1]
                tool_start_points[-1] = move_type
                #joint_start_points = enqueue_point_list[k - begin_sequence - 1][2][0]
                joint_start_points = joint_space_data[:, sequence_slices[k-1][1]].T[-1]
                joint_start_points[-1] = move_type
                points_to_enqueue[1] = np.hstack((np.array([tool_start_points]).T, points_to_enqueue[1]))
                points_to_enqueue[2] = np.hstack((np.array([joint_start_points]).T, points_to_enqueue[2]))

                if k < end_sequence-1:
                    tool_end_points = tool_space_data[:, sequence_slices[k+1][1]].T[0]
                    tool_end_points[-1] = move_type
                    joint_end_points = joint_space_data[:, sequence_slices[k+1][1]].T[0]
                    joint_end_points[-1] = move_type
                    points_to_enqueue[1] = np.hstack((points_to_enqueue[1], np.array([tool_end_points]).T))
                    points_to_enqueue[2] = np.hstack((points_to_enqueue[2], np.array([joint_end_points]).T))

            self.raw_point_queue.put(points_to_enqueue)

    def extractLatestPlanRequest(self, requested_sequence_id):
        for request in self.requested_point_payload_buffer:
            if request[0] == requested_sequence_id:
                return request

    # def extractStartingVoxelPoints(self):
    #     #return self.machine_points.T[self.contiguous_sequences[self.starting_sequence_id],:5]
    #     #return pncLibrary.TP.rotaryAxesToDegrees(np.array([self.machine_points.T[self.contiguous_sequences[self.starting_sequence_id]][:self.machine.number_of_joints]]))
    #     #return pncLibrary.TP.rotaryAxesToDegrees(np.array([self.cloud_trajectory_planner_receiver.planned_point_payload_buffer[0]['joint_space_data'].T[0,:self.machine.number_of_joints]]))[0]
    #     starting_points = copy.deepcopy(self.cloud_trajectory_planner_receiver.planned_point_payload_buffer[0]['joint_space_data'].T[0,:self.machine.number_of_joints])
    #     return pncLibrary.TP.rotaryAxesToDegrees(np.array([starting_points]))[0]
    #     #return self.cloud_trajectory_planner_receiver.planned_point_payload_buffer[0]['joint_space_data'].T[0,:self.machine.number_of_joints]

    def flushDataFeedbackToTP(self):
        while not self.synchronizer.q_trajectory_planner_data_return_queue.empty():
            data_to_return = self.synchronizer.q_trajectory_planner_data_return_queue.get()
            self.tp_state.metadata_ack_event.clear()
            data_size = self.websocket_communicator.sendDataObject(pncLibrary.TPData(
                message_type='EXECUTED_DATA',
                file_name=self.toolpath_data.sculptprint_file_name,
                path_id=self.toolpath_data.toolpath_id,
                feedback_data_type=self.machine.servo_log_types[self.machine.servo_log_type],
                feedback_period=int(1000 * self.machine.servo_dt * self.machine.servo_log_sub_sample_rate),
                starting_sequence=np.array([self.starting_sequence_id]),
                ending_sequence=np.array([self.starting_sequence_id+self.plan_to_index_delta]),
                motion_start_time=self.machine.motion_start_time,
                stepgen_executed_points=data_to_return['stepgen_executed_points'],
                encoder_executed_points=data_to_return['encoder_executed_points'],
                rt_time=data_to_return['rt_time'],
                nonrt_time=data_to_return['nonrt_time'],
                buffer_level=data_to_return['buffer_level']))

            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_websocket_motion_data_sent_string,
                                                         self.name, data_size, self.starting_sequence_id,
                                                         self.starting_sequence_id+self.plan_to_index_delta-1)



    def sendStartupMetadata(self):
        self.synchronizer.tp_metadata_ack_event.clear()
        meta_size = self.websocket_communicator.sendDataObject(pncLibrary.TPData(message_type='METADATA',
                                                                                file_name=self.machine.toolpath_data.sculptprint_file_name,
                                                                                path_id=self.machine.toolpath_data.toolpath_id,
                                                                                block_length=self.machine.websocket_block_length,
                                                                                velocity_limit=0.6*np.vstack((-np.array(self.machine.tp_max_joint_velocity), np.array(self.machine.tp_max_joint_velocity))).T,
                                                                                acceleration_limit=0.6*np.vstack((-np.array(self.machine.tp_max_joint_acceleration), np.array(self.machine.tp_max_joint_acceleration))).T,
                                                                                servo_dt=self.machine.servo_dt,
                                                                                tool_transformation=self.machine.toolpath_data.tool_transformation_matrix,
                                                                                part_transformation=self.machine.toolpath_data.work_transformation_matrix,
                                                                                mrr_limit=0,
                                                                                compute_grid_scale_factor=1))
                                                                                #conservative_bounding=True))

        self.synchronizer.tp_metadata_ack_event.wait()

    def updateInternalToolpathData(self, toolpath_data):
        self.toolpath_data = toolpath_data

class WebsocketReceiver(Thread):
    def __init__(self, parent):
        super(WebsocketReceiver, self).__init__()
        self.parent = parent
        self.name = "cloud_trajectory_planner_receiver"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer
        self.tp_websocket = parent.tp_websocket
        self.websocket_communicator = self.parent.websocket_communicator
        #self.tp_state = parent.tp_state

        self.assembled_payload_points = np.empty((0,6))
        #self.planned_point_output_queue = Queue()
        self.planned_point_payload_buffer = []

        self.startup_event = Event()

    def run(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_subthread_launch_string, self.parent.name,
                                                     self.name)

        self.startup_event.set()

        while self.synchronizer.t_run_cloud_trajectory_planner_receiver_event.is_set():
            if self.synchronizer.tp_websocket_server_connected_event.is_set():
                try:
                    tp_message = self.websocket_communicator.receiveMessage()
                    if not tp_message:
                        raise websocket.WebSocketPayloadException

                    if tp_message.message_data['message_type'] == 'REQUESTED_DATA_ACK':
                        move_type_index = pncLibrary.tp_move_types.index(tp_message.message_data['move_type'])
                        self.doStateAckSanityChecks(tp_message, move_type_index)

                        pncLibrary.updateMachineModelList(self.machine, 'tp_state_subsequence_ack_id',
                                                          tp_message.message_data['sid'][1], move_type_index)
                        #self.tp_state.subsequence_ack_id[move_type_index] = tp_message.message_data['sid'][1]
                        self.synchronizer.tp_subsequence_id_ack_event.set()

                        if tp_message.message_data['sid'][1] == tp_message.message_data['number_of_subsequences'][0]-1:
                            pncLibrary.updateMachineModelList(self.machine, 'tp_state_sequence_ack_id',
                                                              tp_message.message_data['sid'][0], move_type_index)
                            #self.tp_state.sequence_ack_id[move_type_index] = tp_message.message_data['sid'][0]
                            self.synchronizer.tp_sequence_id_ack_event.set()

                    elif tp_message.message_data['message_type'] == 'HELLO_ACK':
                        self.synchronizer.tp_connected_event.set()

                    elif tp_message.message_data['message_type'] == 'METADATA_ACK':
                        self.synchronizer.tp_metadata_ack_event.set()

                    elif tp_message.message_data['message_type'] == 'RESET_CONNECTION_ACK':
                        pass

                    elif tp_message.message_data['message_type'] == 'PLANNED_DATA':
                        tp_message.message_data['planning_time'] = time.time()-self.parent.send_time
                        self.planned_point_payload_buffer.append(tp_message.message_data)
                        #self.doStatePlannedSanityChecks(tp_message, move_type_index)
                        if not (tp_message.message_data['joint_space_data'] == -1).all() and not tp_message.message_data['joint_space_data'].size == 1:
                            move_type_index = pncLibrary.tp_move_types.index(tp_message.message_data['move_type'])

                            #self.tp_state.number_of_subsequences[1] = tp_message.message_data['number_of_subsequences'][1]
                            pncLibrary.updateMachineModelList(self.machine, 'tp_state_number_of_subsequences',
                                                              tp_message.message_data['number_of_subsequences'][1], 1)
                            #self.tp_state.current_received_subsequence_id[move_type_index] = tp_message.message_data['sid'][1]
                            pncLibrary.updateMachineModelList(self.machine, 'tp_state_current_received_subsequence_id',
                                                              tp_message.message_data['sid'][1], move_type_index)
                            print('received subsequence id ' + str(tp_message.message_data['sid'][1]))

                            if self.machine.tp_state_current_received_subsequence_id[move_type_index] == 0:
                                self.synchronizer.tp_send_next_block_event.set()

                            self.assembled_payload_points = np.vstack((self.assembled_payload_points, tp_message.message_data['joint_space_data'].T))
                            pncLibrary.printStringToTerminalMessageQueue(
                                self.synchronizer.q_print_server_message_queue,
                                pncLibrary.printout_trajectory_planner_subsequence_received_string, self.name, tp_message.payload_size,
                                self.machine.tp_state_current_received_subsequence_id[move_type_index], tp_message.message_data['sid'][0], self.parent.remote_tp_name)

                            if self.machine.tp_state_current_received_subsequence_id[move_type_index] == tp_message.message_data['number_of_subsequences'][1]-1:
                            #if tp_message.message_data['sub_sid'] == tp_message.message_data['num_sub_seqs']-1:
                                pncLibrary.updateMachineModelList(self.machine, 'tp_state_current_received_sequence_id',
                                                                  tp_message.message_data['sid'][0], move_type_index)
                                #self.tp_state.current_received_sequence_id[move_type_index] = tp_message.message_data['sid'][0]
                                move_to_enqueue = pncLibrary.Move(pncLibrary.TP.rotaryAxesToDegrees(self.assembled_payload_points[:, 0:self.machine.number_of_joints]),
                                        move_type=tp_message.message_data['move_type'], sequence_id=tp_message.message_data['sid'][0])
                                if tp_message.message_data['move_type'] == 'rapid':
                                    #self.tp_state.current_received_rapid_sequence_id = tp_message.message_data['sid']
                                    self.synchronizer.q_cloud_trajectory_planner_interface_reposition_move_queue.put(move_to_enqueue)
                                    self.synchronizer.tp_reposition_move_received_event.set()
                                    pncLibrary.printStringToTerminalMessageQueue(
                                        self.synchronizer.q_print_server_message_queue,
                                        pncLibrary.printout_trajectory_planner_rapid_sequence_enqueued_string, self.name,
                                        self.machine.tp_state_current_received_sequence_id[1])
                                    self.synchronizer.q_database_command_queue_proxy.put(
                                        pncLibrary.DatabaseCommand('push_object',
                                                                   [{"PLANNED_RAPID_REQUESTS": move_to_enqueue}]))
                                elif tp_message.message_data['move_type'] == 'SP_trajectory':
                                    #self.tp_state.current_received_CAM_sequence_id = tp_message.message_data['sid']
                                    self.synchronizer.tp_all_moves_consumed_event.clear()
                                    self.synchronizer.tp_first_trajectory_received_event.set()
                                    self.synchronizer.q_cloud_trajectory_planner_interface_planned_move_queue.put(move_to_enqueue)
                                    pncLibrary.printStringToTerminalMessageQueue(
                                        self.synchronizer.q_print_server_message_queue,
                                        pncLibrary.printout_trajectory_planner_cutting_sequence_enqueued_string, self.name,
                                        self.machine.tp_state_current_received_sequence_id[0])

                                    self.synchronizer.q_database_command_queue_proxy.put(
                                        pncLibrary.DatabaseCommand('push_object',
                                                                   [{"PLANNED_CAM_TOOLPATH_REQUESTS": move_to_enqueue}]))
                                self.assembled_payload_points = np.empty((0, 6))

                        else:
                            self.tp_state.current_received_sequence_id = tp_message.message_data['sid']
                            self.machine.tp_state_current_received_sequence_id = tp_message.message_data['sid']
                            self.assembled_payload_points = np.empty((0, 5))
                            if tp_message.message_data['move_type'] == 'rapid':
                                pncLibrary.printStringToTerminalMessageQueue(
                                    self.synchronizer.q_print_server_message_queue,
                                    pncLibrary.printout_trajectory_planner_bad_rapid_sequence_received_string,
                                    self.parent.remote_tp_name,
                                    self.machine.tp_state_current_received_sequence_id)
                            elif tp_message.message_data['move_type'] == 'SP_trajectory':
                                pncLibrary.printStringToTerminalMessageQueue(
                                    self.synchronizer.q_print_server_message_queue,
                                    pncLibrary.printout_trajectory_planner_bad_cutting_sequence_received_string,
                                    self.parent.remote_tp_name,
                                    self.machine.tp_state_current_received_sequence_id)

                    elif tp_message.message_data['message_type'] == 'ERROR':
                        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, pncLibrary.printout_trajectory_planner_error_string, tp_message.message_data['error_string'])

                    else:
                        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                                     pncLibrary.printout_trajectory_planner_unrecognized_message_string,
                                                                     tp_message.message_data['message_type'])
                        print('unrecognized message type')

                except websocket.WebSocketTimeoutException:
                    #print('WEBSOCKET TIMEOUT')
                    pass

                except websocket.WebSocketPayloadException:
                    #Bad transmission
                    print("WEBSOCKET RECEIVER: Received empty transmission")
                    pass

                except pncLibrary.TPStateError as error:
                    print('WEBSOCKET RECEIVER: State machine mismatch, ' + error.message)

                except Exception as error:
                    print("WEBSOCKET RECEIVER: Had error: ", str(error))
                    import traceback
                    traceback.print_exc()

        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_subthread_terminate_string, self.parent.name,
                                                     self.name)

    def doStatePlannedSanityChecks(self, tp_message, move_type_index):
        if not tp_message.message_data['sid'][0] == self.tp_state.current_requested_sequence_id[move_type_index]:
            raise pncLibrary.TPStateError('TP sequence mismatch in PLANNED_DATA')

        # if self.tp_state.current_received_subsequence_id[move_type_index] == 0:
        #     self.tp_state.send_next_block_event.set()

    def doStateAckSanityChecks(self, tp_message, move_type_index):
        if not tp_message.message_data['sid'][0] == self.machine.tp_state_current_requested_sequence_id[move_type_index]:
            raise pncLibrary.TPStateError('TP sequence mismatch in REQUESTED_DATA_ACK')
        #if not self.tp_state.subsequence_ack_id[move_type_index] == self.tp_state.current_requested_subsequence_id[move_type_index]:
        if not tp_message.message_data['sid'][1] == self.machine.tp_state_current_requested_subsequence_id[move_type_index]:
            raise pncLibrary.TPStateError('TP subsequence mismatch in REQUESTED_DATA_ACK')

    def plotPlanningTime(self):
        import matplotlib.pyplot as plt
        plt.plot([self.planned_point_payload_buffer[k]['planned_points'].shape[0] for k in
                  range(0, len(self.planned_point_payload_buffer))],
                 [self.planned_point_payload_buffer[k].planning_time for k in
                  range(0, len(self.planned_point_payload_buffer))], 'ro')
        plt.show()

    def extractStartingPoints(self, payload_index):
        return self.planned_point_payload_buffer[payload_index]['planned_points'][:,0]