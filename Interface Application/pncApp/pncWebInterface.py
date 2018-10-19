import pncLibrary, time, websocket, json, base64, numpy as np
from multiprocessing import Queue, Event, current_process
from threading import Thread, current_thread
from queue import Empty
from io import BytesIO

class WebsocketCommunicatorAchex:
    def __init__(self, parent):
        self.parent = parent
        self.name = "websocket_communicator_achex"
        self.tp_websocket = self.parent.tp_websocket
        self.machine = self.parent.machine
        self.tp_state = self.parent.tp_state
        self.synchronizer = self.parent.synchronizer

    def openConnection(self):
        self.tp_websocket.connect("ws://achex.ca:4010")
        self.tp_websocket.SID = json.loads(self.tp_websocket.recv())['SID']
        self.tp_websocket.send(json.dumps({'setID': self.machine.websocket_username, "passwd": "mypass"}))
        try:
            auth_message = json.loads(self.tp_websocket.recv())
            if auth_message['auth'] == 'ok':
                pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                             pncLibrary.printout_websocket_connection_success_string,
                                                             self.machine.websocket_username)
                self.tp_state.websocket_connected_event.set()
                return True
        except ConnectionAbortedError as websocket_error:
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_websocket_connection_failure_string,
                                                         self.name, websocket_error)
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

    def receiveMessage(self):
        return json.loads(self.tp_websocket.recv())['payload']

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
            self.tp_state.websocket_connected_event.set()
            return True

    def sendMessage(self, **kwargs):
        data_stream = BytesIO()
        #data_stream.seek(0)
        np.savez_compressed(data_stream, **kwargs)
        data_stream.seek(0)
        encoded_string = base64.b64encode(data_stream.read())
        #self.web_socket.send(self.decode_bytes(data_stream))
        self.tp_websocket.send(encoded_string)
        return len(encoded_string)

    def receiveMessage(self):
        return self.tp_websocket.recv()

class WebsocketReceiver(Thread):
    def __init__(self, parent):
        super(WebsocketReceiver, self).__init__()
        self.parent = parent
        self.name = "cloud_trajectory_planner_receiver"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer
        self.tp_websocket = parent.tp_websocket
        self.websocket_communicator = self.parent.websocket_communicator
        self.tp_state = parent.tp_state
        #self.websocket_connected_event = parent.websocket_connected_event

        self.enqueued_sequence_id = 0
        self.current_requested_sequence_id = 0
        self.current_received_sequence_id = 0

        self.assembled_payload_points = np.empty((0,6))
        self.planned_point_output_queue = Queue()
        self.planned_point_payload_buffer = []

        self.startup_event = Event()

    def run(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_subthread_launch_string, self.parent.name,
                                                     self.name)

        self.startup_event.set()

        while self.synchronizer.t_run_cloud_trajectory_planner_receiver_event.is_set():
            if self.tp_state.websocket_connected_event.is_set():
                try:
                    b64payload = self.websocket_communicator.receiveMessage()
                    if not b64payload:
                        raise websocket.WebSocketPayloadException

                    payload, keys = self.decodePayload(b64payload)
                    if 'sid_ack' in keys:
                    #if type[0] == 'POINTSACK':
                        #self.tp_state.sequence_ack_id = payload['sid_ack'].item()
                        if payload['move_type'] == 'SP_trajectory':
                            self.tp_state.cutting_sequence_ack_id = payload['sid_ack'].item()
                            if self.tp_state.SP_sequence_ack_id == self.tp_state.current_requested_SP_sequence_id:
                                self.tp_state.sequence_id_ack_event.set()
                        elif payload['move_type'] == 'rapid':
                            self.tp_state.rapid_sequence_ack_id = payload['sid_ack'].item()
                            if self.tp_state.rapid_sequence_ack_id == self.tp_state.current_requested_rapid_sequence_id:
                                self.tp_state.sequence_id_ack_event.set()
                        # if self.tp_state.sequence_ack_id == self.tp_state.current_requested_sequence_id:
                        #     self.tp_state.sequence_id_ack_event.set()
                    elif 'hello_ack' in keys:
                        self.tp_state.tp_connected_event.set()
                    elif 'metadata_ack' in keys:
                        #print('TP got metadata')
                        self.tp_state.metadata_ack_event.set()
                        pass
                    elif 'error' in keys and payload['error']:
                        if not payload['error'] == 'IOError':
                            print('TP got error')
                    elif 'reset_connection_ack' in keys:
                        pass
                    elif 'planned_points' in keys:
                        payload.planning_time = time.time()-self.parent.send_time
                        self.planned_point_payload_buffer.append(payload)

                        if not (payload['planned_points'] == -1).all() and not payload['planned_points'].size == 1:
                            # if payload['move_type'] == 'SP_trajectory':
                            #     self.synchronizer.tp_first_trajectory_received_event.set()
                            if 'num_sub_seqs' in keys:
                                #self.tp_state.send_next_block_event.set()
                                self.tp_state.incoming_number_of_sub_sequences = payload['num_sub_seqs'].item()
                                self.tp_state.current_sub_sid = payload['sub_sid'].item()
                                #Only plan one move ahead of current position
                                if self.tp_state.current_sub_sid == 0:
                                    self.tp_state.send_next_block_event.set()
                                self.assembled_payload_points = np.vstack((self.assembled_payload_points, payload['planned_points'].T))
                                pncLibrary.printStringToTerminalMessageQueue(
                                    self.synchronizer.q_print_server_message_queue,
                                    pncLibrary.printout_trajectory_planner_subsequence_received_string, self.name, payload.payload_size,
                                    self.tp_state.current_sub_sid, payload['sid'].item(), self.parent.remote_tp_name)

                            if payload['sub_sid'] == payload['num_sub_seqs']-1:
                                #self.tp_state.current_received_sequence_id = payload['sid'].item()
                                #self.tp_state.all_moves_consumed_event.clear()
                                if payload['move_type'] == 'rapid':
                                    self.tp_state.current_received_rapid_sequence_id = payload['sid'].item()
                                    self.synchronizer.tp_reposition_move_received_event.set()
                                    self.synchronizer.q_trajectory_planner_reposition_move_queue.put(pncLibrary.Move(
                                        self.assembled_payload_points[:, 0:self.machine.number_of_joints],
                                        move_type='remotely_planned', sequence_id=payload['sid'].item()))
                                    pncLibrary.printStringToTerminalMessageQueue(
                                        self.synchronizer.q_print_server_message_queue,
                                        pncLibrary.printout_trajectory_planner_rapid_sequence_enqueued_string, self.name,
                                        self.tp_state.current_received_SP_sequence_id)
                                elif payload['move_type'] == 'SP_trajectory':
                                    self.tp_state.current_received_SP_sequence_id = payload['sid'].item()
                                    self.tp_state.all_moves_consumed_event.clear()
                                    self.synchronizer.tp_first_trajectory_received_event.set()
                                    self.synchronizer.q_trajectory_planner_planned_move_queue.put(pncLibrary.Move(
                                        self.assembled_payload_points[:, 0:self.machine.number_of_joints],
                                        move_type='remotely_planned', sequence_id=payload['sid'].item()))
                                    pncLibrary.printStringToTerminalMessageQueue(
                                        self.synchronizer.q_print_server_message_queue,
                                        pncLibrary.printout_trajectory_planner_cutting_sequence_enqueued_string, self.name,
                                        self.tp_state.current_received_SP_sequence_id)

                                self.assembled_payload_points = np.empty((0, 6))

                        else:
                            self.tp_state.current_received_sequence_id = payload['sid'].item()
                            self.assembled_payload_points = np.empty((0, 6))
                            if payload['move_type'] == 'rapid':
                                pncLibrary.printStringToTerminalMessageQueue(
                                    self.synchronizer.q_print_server_message_queue,
                                    pncLibrary.printout_trajectory_planner_bad_rapid_sequence_received_string,
                                    self.parent.remote_tp_name,
                                    self.tp_state.current_received_sequence_id)
                            elif payload['move_type'] == 'SP_trajectory':
                                pncLibrary.printStringToTerminalMessageQueue(
                                    self.synchronizer.q_print_server_message_queue,
                                    pncLibrary.printout_trajectory_planner_bad_cutting_sequence_received_string,
                                    self.parent.remote_tp_name,
                                    self.tp_state.current_received_sequence_id)
                            # pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                            #                                              pncLibrary.printout_bad_sequence_received_string,
                            #                                              self.parent.remote_tp_name,
                            #                                              self.tp_state.current_received_sequence_id)

                        # if self.tp_state.current_requested_SP_sequence_id == self.tp_state.current_received_SP_sequence_id or
                        #     self.tp_state.current_requested_rapid_sequence_id == self.tp_state.current_received_rapid_sequence_id:
                        #     self.tp_state.matching_sequence_received_event.set()
                        #     self.tp_state.send_next_block_event.set()

                except websocket.WebSocketTimeoutException:
                    #print('WEBSOCKET TIMEOUT')
                    pass
                except websocket.WebSocketPayloadException:
                    #Bad transmission
                    print("WEBSOCKET RECEIVER: Received empty transmission")
                    pass
                except Exception as error:
                    print("WEBSOCKET RECEIVER: Had error: ", str(error))

        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_subthread_terminate_string, self.parent.name,
                                                     self.name)

    def decodePayload(self, payload):
        payload_size = len(payload)
        byte_stream = BytesIO(base64.b64decode(payload.encode('utf-8')))
        byte_stream.seek(0)
        try:
            decoded_payload = np.load(byte_stream)
            decoded_payload.payload_size = payload_size
        except np.BadZipFile:
            print('break load')
            self.parent.websocket_communicator.sendMessage(reset_connection='')
        except Exception as error:
            print('error: ' + str(error))
            #self.websocket_communicator.sendMessage(reset_connection='')
        return decoded_payload, decoded_payload.keys()

    def plotPlanningTime(self):
        import matplotlib.pyplot as plt
        plt.plot([self.planned_point_payload_buffer[k]['planned_points'].shape[0] for k in
                  range(0, len(self.planned_point_payload_buffer))],
                 [self.planned_point_payload_buffer[k].planning_time for k in
                  range(0, len(self.planned_point_payload_buffer))], 'ro')
        plt.show()

    def extractStartingPoints(self, payload_index):
        return self.planned_point_payload_buffer[payload_index]['planned_points'][:,0]


class CloudTrajectoryPlannerInterface(Thread):
    def __init__(self, parent):
        super(CloudTrajectoryPlannerInterface, self).__init__()
        self.name = "cloud_trajectory_planner"
        self.machine = parent.machine
        self.toolpath_data = None
        self.remote_tp_name = "MukulLab"
        self.synchronizer = parent.synchronizer
        self.tp_websocket = websocket.WebSocket()
        self.tp_websocket.timeout = self.machine.websocket_timeout

        self.tp_state = pncLibrary.CloudTrajectoryPlannerState()

        if self.machine.websocket_type == 'achex':
            self.websocket_communicator = WebsocketCommunicatorAchex(self)
        elif self.machine.websocket_type == 'wsninja':
            self.websocket_communicator = WebsocketCommunicatorNinja(self)

        self.raw_point_queue = Queue()
        self.planned_point_output_queue = Queue()
        self.startup_event = Event()
        #self.tp_first_trajectory_received_event = Event()
        #self.tp_planning_finished_event = Event()

        self.planned_point_payload_buffer = []
        self.requested_point_payload_buffer = []
        self.planning_time_log = []

        #self.sp_file_name = 'pass4complete'
        self.path_id = 3
        self.joint_data_file = "pass5_machine_short"
        self.tool_data_file = "pass5_tool_short"
        # self.work_transformation_file = 'machine_tableToPart.txt'
        # self.tool_transformation_file = 'machine_toolToHolder.txt'

        self.starting_sequence_id = 0
        self.ending_sequence_id = -1
        self.plan_to_index_delta = 0
        self.conversion_factor_dist = 1
        self.send_time = 0

        #self.loadTransformations()

    def run(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_launch_string, current_process().name,
                                                     self.name)
        pncLibrary.waitForThreadStart(self, WebsocketReceiver)
        self.startup_event.set()
        #return
        while not self.websocket_communicator.openConnection():
            pass

        if self.connectToTP():
            self.initializeToolpathData()
            #self.synchronizer.tp_need_points_event.set()
            #self.enqueueSequencesForPlanning()

            self.synchronizer.tp_toolpath_setup_event.wait()
            self.sendStartupMetadata()

            while self.synchronizer.t_run_cloud_trajectory_planner_event.is_set() and self.tp_state.tp_connected_event.is_set():
                try:
                    self.requestToolpathPointsFromCAM()
                    if self.tp_state.send_next_block_event.wait(self.machine.event_wait_timeout) and self.synchronizer.tp_plan_motion_event.wait(self.machine.event_wait_timeout):
                        if (self.tp_state.current_requested_SP_sequence_id-self.starting_sequence_id) < self.plan_to_index_delta:
                            self.tp_state.sequence_id_ack_event.clear()
                            self.tp_state.matching_sequence_received_event.clear()

                            self.synchronizer.tp_need_points_event.set()


                            points_to_plan = self.raw_point_queue.get(True, pncLibrary.queue_move_queue_wait_timeout)
                            self.synchronizer.tp_planning_finished_event.clear()
                            self.tp_state.data_flushed_to_websocket_event.clear()

                            #self.tp_state.current_requested_sequence_id = points_to_plan[0]
                            if points_to_plan[3] == 'SP_trajectory':
                                self.tp_state.current_requested_SP_sequence_id = points_to_plan[0]
                            elif points_to_plan[3] == 'rapid':
                                self.tp_state.current_requested_rapid_sequence_id = points_to_plan[0]

                            self.send_time = time.time()
                            self.tp_state.send_next_block_event.clear()
                            self.sendPlanningRequest(points_to_plan)
                            self.requested_point_payload_buffer.append(points_to_plan)
                        else:
                            raise Empty

                except Empty:
                    if self.tp_state.current_received_SP_sequence_id == self.tp_state.current_requested_SP_sequence_id:
                        if not self.synchronizer.tp_planning_finished_event.is_set():
                            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                                         pncLibrary.printout_trajectory_planning_finished_string,
                                                                         self.tp_state.current_received_SP_sequence_id)
                            self.synchronizer.tp_planning_finished_event.set()

                        if self.synchronizer.q_trajectory_planner_planned_move_queue.empty():
                            self.tp_state.all_moves_consumed_event.set()
                            #self.synchronizer.db_write_to_websocket_event.set()
                            if not self.tp_state.data_flushed_to_websocket_event.is_set() and self.synchronizer.mc_motion_complete_event.is_set():
                                self.synchronizer.q_database_command_queue_proxy.put(pncLibrary.DatabaseCommand('flush_to_websocket', self.machine.motion_start_time))
                                self.tp_state.data_flushed_to_websocket_event.set()

                finally:
                    self.flushDataFeedbackToTP()

        pncLibrary.waitForThreadStop(self, self.cloud_trajectory_planner_receiver)
        self.tp_websocket.close()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_thread_terminate_string, current_process().name,
                                                     self.name)

    # def openConnection(self):
    #     if self.machine.websocket_type == "wsninja":
    #         self.tp_websocket.connect("wss://node2.wsninja.io")
    #         self.tp_websocket.send(json.dumps({'guid': self.machine.websocket_client_GUID}))
    #         ack_message = json.loads(self.tp_websocket.recv())
    #         if not ack_message['accepted']:
    #             pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
    #                                                          pncLibrary.printout_websocket_connection_failure_string,
    #                                                          self.name, 'Websocket connection not accepted')
    #             return False
    #         else:
    #             self.tp_state.websocket_connected_event.set()
    #             return True
    #     elif self.machine.websocket_type == "achex":
    #         self.tp_websocket.connect("ws://achex.ca:4010")
    #         self.tp_websocket.SID = json.loads(self.tp_websocket.recv())['SID']
    #         self.tp_websocket.send(json.dumps({'setID': "pncapp", "passwd": "mypass"}))
    #         try:
    #             auth_message = json.loads(self.tp_websocket.recv())
    #             if auth_message['auth'] == 'ok':
    #                 pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
    #                                                              pncLibrary.printout_websocket_connection_success_string,
    #                                                              self.machine.websocket_username)
    #                 return True
    #         except ConnectionAbortedError as websocket_error:
    #             pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
    #                                                          pncLibrary.printout_websocket_connection_failure_string,
    #                                                          self.name, websocket_error)
    #             return False


    def connectToTP(self):
        msg_size = self.websocket_communicator.sendMessage(hello='HELLO')
        if self.tp_state.tp_connected_event.wait(self.machine.event_wait_timeout):
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_trajectory_planner_connection_string,
                                                         self.name, self.remote_tp_name)
            self.tp_state.send_next_block_event.set()
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

    def sendPlanningRequest(self, points_to_plan):
        data_size = self.websocket_communicator.sendMessage(sid=points_to_plan[0], tool=points_to_plan[1], joint=points_to_plan[2], move_type=points_to_plan[3])
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_trajectory_plan_request_sent_string,
                                                     self.name, data_size, int(points_to_plan[0]))

    # def sendMessage(self, **kwargs):
    #     data_stream = BytesIO()
    #     #data_stream.seek(0)
    #     np.savez_compressed(data_stream, **kwargs)
    #     data_stream.seek(0)
    #     encoded_string = base64.b64encode(data_stream.read())
    #     #self.web_socket.send(self.decode_bytes(data_stream))
    #     if self.machine.websocket_type == 'wsninja':
    #         self.tp_websocket.send(encoded_string)
    #     elif self.machine.websocket_type == 'achex':
    #         self.tp_websocket.send(json.dumps({"to": self.machine.websocket_tp_name, "payload": encoded_string.decode()}))
    #     return len(encoded_string)

    def resetWebsocketBuffer(self):
        self.websocket_communicator.sendMessage(reset='RESET')
        while 'RESET' not in self.tp_websocket.recv():
            pass

    def requestToolpathPointsFromCAM(self):
        if self.tp_state.sculptprint_point_buffer_length > self.raw_point_queue.qsize():
            self.synchronizer.tp_need_points_event.set()

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
        # if output_queue is None:
        #     output_queue = self.raw_point_queue

            # self.tool_points, self.machine_points, self.contiguous_sequences,
            # self.starting_sequence_id,
            # self.ending_sequence_id if self.ending_sequence_id >= 0 else len(
            #     self.contiguous_sequences))
        #print("TRAJECTORY PLANNER: Enqueueing {} sequences from {}")
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

            #enqueue_point_list.append(points_to_enqueue)

            #self.raw_point_queue.put((np.array([self.tp_state.enqueued_sequence_id]), tool_space_data[:, sequence_slices[k][1]], joint_space_data[:, sequence_slices[k][1]]))
            self.raw_point_queue.put(points_to_enqueue)
            #self.raw_point_queue.put((self.path_serial_number, tool_space_data, joint_space_data))

    def extractLatestPlanRequest(self, requested_sequence_id):
        for request in self.requested_point_payload_buffer:
            if request[0] == requested_sequence_id:
                return request

    def extractStartingVoxelPoints(self):
        #return self.machine_points.T[self.contiguous_sequences[self.starting_sequence_id],:5]
        return pncLibrary.TP.rotaryAxesToDegrees(np.array([self.machine_points.T[self.contiguous_sequences[self.starting_sequence_id]][:self.machine.number_of_joints]]))

    def flushDataFeedbackToTP(self):
        while not self.synchronizer.q_trajectory_planner_data_return_queue.empty():
            data_to_return = self.synchronizer.q_trajectory_planner_data_return_queue.get()
            self.tp_state.metadata_ack_event.clear()
            meta_size = self.websocket_communicator.sendMessage(metadata='', file_name=self.sp_file_name, pass_number=np.array([self.path_id]),
                             feedback_data_type=self.machine.servo_log_types[self.machine.servo_log_type],
                             feedback_period = np.array([int(1000*self.machine.servo_dt*self.machine.servo_log_sub_sample_rate)]),
                             starting_sequence=np.array([self.starting_sequence_id]),
                             ending_sequence=np.array([self.starting_sequence_id+self.plan_to_index_delta]),
                             motion_start_time=np.array([self.machine.motion_start_time]))
            self.tp_state.metadata_ack_event.wait()
            data_size = self.websocket_communicator.sendMessage(planned_points=data_to_return['planned_points'],
                             executed_points=data_to_return['executed_points'],
                             motion_start_time=np.array([self.machine.motion_start_time]),
                             rt_time=data_to_return['rt_time'],
                             nonrt_time=data_to_return['nonrt_time'],
                             buffer_level=data_to_return['buffer_level'])
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_websocket_motion_data_sent_string,
                                                         self.name, data_size, self.starting_sequence_id,
                                                         self.starting_sequence_id+self.plan_to_index_delta-1)

            # self.websocket_communicator.sendMessage(file_name=self.sp_file_name, pass_number=np.array([self.path_id]),
            #                  feedback_data_type=self.machine.servo_log_types[self.machine.servo_log_type],
            #                  feedback_period = np.array([self.machine.servo_dt*self.machine.servo_log_sub_sample_rate]))

    # def loadTransformations(self):
    #     self.work_transformation_matrix = np.loadtxt(self.machine.raw_point_files_path + self.work_transformation_file, skiprows=2).T
    #     self.tool_transformation_matrix = np.loadtxt(self.machine.raw_point_files_path + self.tool_transformation_file, skiprows=2).T

    def sendStartupMetadata(self):
        self.tp_state.metadata_ack_event.clear()
        meta_size = self.websocket_communicator.sendMessage(metadata='', file_name=self.toolpath_data.sculptprint_file_name, path_id=self.toolpath_data.toolpath_id,
                         block_length=np.array([self.machine.websocket_block_length]),
                         velocity_limit=np.asarray(self.machine.tp_max_joint_velocity) * 1,
                         acceleration_limit=np.asarray(self.machine.tp_max_joint_acceleration) * 1,
                         servo_dt=np.array([self.machine.servo_dt]),
                         tool_transformation=self.toolpath_data.tool_transformation_matrix,
                         part_transformation=self.toolpath_data.work_transformation_matrix,
                         mrr_limit=0)
        self.tp_state.metadata_ack_event.wait()
