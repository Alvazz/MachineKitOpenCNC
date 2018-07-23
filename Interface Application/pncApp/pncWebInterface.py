import pncLibrary, time, websocket, json, base64, numpy as np
from multiprocessing import Queue, Event, current_process
from threading import Thread, current_thread
from queue import Empty
from io import BytesIO

class WebsocketReceiver(Thread):
    def __init__(self, parent):
        super(WebsocketReceiver, self).__init__()
        self.parent = parent
        self.name = "cloud_trajectory_planner_receiver"
        self.machine = parent.machine
        self.synchronizer = parent.synchronizer
        self.tp_websocket = parent.tp_websocket
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
                    b64payload = self.tp_websocket.recv()
                    if not b64payload:
                        raise websocket.WebSocketPayloadException

                    payload, keys = self.decodePayload(b64payload)
                    if 'sid_ack' in keys:
                    #if type[0] == 'POINTSACK':
                        self.tp_state.sequence_ack_id = payload['sid_ack'].item()
                        if self.tp_state.sequence_ack_id == self.tp_state.current_requested_sequence_id:
                            self.tp_state.sequence_id_ack_event.set()
                    elif 'hello_ack' in keys:
                        self.tp_state.tp_connected_event.set()
                    elif 'metadata_ack' in keys:
                        #print('TP got metadata')
                        pass
                    elif 'error' in keys:
                        print('TP got error')
                    elif 'reset_connection_ack' in keys:
                        pass
                    elif 'planned_points' in keys:
                        payload.planning_time = time.time()-self.parent.send_time
                        self.planned_point_payload_buffer.append(payload)

                        if not (payload['planned_points'] == -1).all() and not payload['planned_points'].size == 1:
                            self.tp_state.first_trajectory_received_event.set()
                            if 'num_sub_seqs' in keys:
                                #self.tp_state.send_next_block_event.set()
                                self.tp_state.incoming_number_of_sub_sequences = payload['num_sub_seqs'].item()
                                self.tp_state.current_sub_sid = payload['sub_sid'].item()
                                #Only plan one move ahead of current position
                                if self.tp_state.current_sub_sid == 0:
                                    self.tp_state.send_next_block_event.set()
                                self.assembled_payload_points = np.vstack((self.assembled_payload_points, payload['planned_points']))
                                pncLibrary.printStringToTerminalMessageQueue(
                                    self.synchronizer.q_print_server_message_queue,
                                    pncLibrary.printout_trajectory_planner_subsequence_received_string, self.name, self.tp_state.current_sub_sid,
                                    payload['sid'].item(), self.parent.remote_tp_name)

                            else:
                                print('should not be here')
                                self.tp_state.incoming_number_of_sub_sequences = 0
                                self.tp_state.current_sub_sid = 0
                                self.assembled_payload_points = payload['planned_points']


                            if payload['sub_sid'] == payload['num_sub_seqs']-1:
                                #[self.planned_point_payload_buffer[k]['planned_points'].shape[0] / self.planned_point_payload_buffer[k].planning_time for k in range(0, len(self.planned_point_payload_buffer))]
                                #self.planned_point_output_queue.put((payload['pid'], payload['planned_points']))
                                # if payload['planned_points'].size == 1:
                                #     print('bad traj')
                                self.tp_state.current_received_sequence_id = payload['sid'].item()
                                self.synchronizer.q_trajectory_planner_planned_move_queue.put(pncLibrary.Move(self.assembled_payload_points[:, 0:self.machine.number_of_joints],move_type='remotely_planned', sequence_id=payload['sid'].item()))
                                self.assembled_payload_points = np.empty((0, 6))
                                pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                                             pncLibrary.printout_trajectory_planner_sequence_enqueued_string, self.name,
                                                                             self.tp_state.current_received_sequence_id)

                        else:
                            self.tp_state.current_received_sequence_id = payload['sid'].item()
                            self.assembled_payload_points = np.empty((0, 6))
                            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                                         pncLibrary.printout_bad_sequence_received_string,
                                                                         self.parent.remote_tp_name,
                                                                         self.tp_state.current_received_sequence_id)

                        if self.tp_state.current_requested_sequence_id == self.tp_state.current_received_sequence_id:
                            self.tp_state.matching_sequence_received_event.set()
                            self.tp_state.send_next_block_event.set()

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
        byte_stream = BytesIO(base64.b64decode(payload.encode('utf-8')))
        byte_stream.seek(0)
        try:
            decoded_payload = np.load(byte_stream)
        except np.BadZipFile:
            print('break load')
            self.parent.sendMessage(reset_connection='')
        except Exception as error:
            print('error: ' + str(error))
            #self.parent.sendMessage(reset_connection='')
        return decoded_payload, decoded_payload.keys()

    def plotPlanningTime(self):
        import matplotlib.pyplot as plt
        plt.plot([self.planned_point_payload_buffer[k]['planned_points'].shape[0] for k in
                  range(0, len(self.planned_point_payload_buffer))],
                 [self.planned_point_payload_buffer[k].planning_time for k in
                  range(0, len(self.planned_point_payload_buffer))], 'ro')
        plt.show()

    def extractStartingPoints(self, payload_index):
        return self.planned_point_payload_buffer[payload_index]['planned_points'][0,:]


class CloudTrajectoryPlannerInterface(Thread):
    def __init__(self, parent):
        super(CloudTrajectoryPlannerInterface, self).__init__()
        self.name = "cloud_trajectory_planner"
        self.machine = parent.machine
        self.remote_tp_name = "MukulLab"
        self.synchronizer = parent.synchronizer
        self.tp_websocket = websocket.WebSocket()
        self.tp_websocket.timeout = self.machine.websocket_timeout

        self.tp_state = pncLibrary.CloudTrajectoryPlannerState()

        self.raw_point_queue = Queue()
        self.planned_point_output_queue = Queue()
        #self.first_trajectory_received_event = Event()
        #self.planning_finished_event = Event()

        self.planned_point_payload_buffer = []
        self.requested_point_payload_buffer = []
        self.planning_time_log = []
        self.sp_file_name = 'pass4complete.scpr'
        self.path_id = 3
        self.joint_data_file = "pass3_machine"
        self.tool_data_file = "pass3_tool"
        self.starting_sequence_id = 4
        self.ending_sequence_id = 7
        self.conversion_factor_dist = 1
        self.send_time = 0

        self.startup_event = Event()

    def run(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_launch_string, current_process().name,
                                                     self.name)
        pncLibrary.waitForThreadStart(self, WebsocketReceiver)
        self.startup_event.set()

        while not self.openConnection():
            pass

        if self.connectToTP():
            self.tool_points, self.machine_points, self.retraction_flags = pncLibrary.TP.load_sp_data(self.machine.raw_point_files_path, self.joint_data_file, self.tool_data_file)
            self.contiguous_sequences = pncLibrary.TP.obtain_contiguous_sequences(self.retraction_flags.astype(int))
            self.enqueuePointsForPlanning(self.tool_points, self.machine_points, self.contiguous_sequences, self.starting_sequence_id, self.ending_sequence_id if self.ending_sequence_id >= 0 else len(self.contiguous_sequences))
            #self.enqueuePointsForPlanning(self.tool_points, self.machine_points, self.contiguous_sequences, 136, len(self.contiguous_sequences))
            #self.enqueuePointsForPlanning(self.tool_points, self.machine_points, self.contiguous_sequences, 134, 137)
            self.sendMessage(metadata='', file_name=self.sp_file_name, path_id=np.array([self.path_id]),
                             block_length=np.array([self.machine.websocket_block_length]),
                             velocity_limit=np.asarray(self.machine.tp_max_joint_velocity)*1,
                             acceleration_limit=np.asarray(self.machine.tp_max_joint_acceleration)*1)
            while self.synchronizer.t_run_cloud_trajectory_planner_event.is_set() and self.tp_state.tp_connected_event.is_set():
                try:
                    if self.tp_state.send_next_block_event.wait(self.machine.event_wait_timeout) and self.synchronizer.tp_plan_motion_event.wait(self.machine.event_wait_timeout):
                        self.tp_state.sequence_id_ack_event.clear()
                        self.tp_state.matching_sequence_received_event.clear()

                        points_to_plan = self.raw_point_queue.get(True, pncLibrary.queue_move_queue_wait_timeout)
                        self.tp_state.planning_finished_event.clear()

                        self.tp_state.current_requested_sequence_id = points_to_plan[0]
                        self.send_time = time.time()
                        self.tp_state.send_next_block_event.clear()
                        self.sendPlanningRequest(points_to_plan)
                        self.requested_point_payload_buffer.append(points_to_plan)

                        self.flushDataFeedbackToTP()

                except Empty:
                    if self.tp_state.current_received_sequence_id == self.tp_state.current_requested_sequence_id and not self.tp_state.planning_finished_event.is_set():
                        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue, pncLibrary.printout_trajectory_planning_finished_string, self.tp_state.current_received_sequence_id)
                        self.tp_state.planning_finished_event.set()

                    # while not self.synchronizer.q_trajectory_planner_data_return_queue.empty():
                    #     data_to_return = self.synchronizer.q_trajectory_planner_data_return_queue.get()
                    #     self.sendMessage(stepgen='STEPGEN', planned_points=data_to_return['commanded'], executed_points=data_to_return['stepgen'])

        pncLibrary.waitForThreadStop(self, self.cloud_trajectory_planner_receiver)
        self.tp_websocket.close()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     pncLibrary.printout_thread_terminate_string, current_process().name,
                                                     self.name)

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

    def connectToTP(self):
        self.sendMessage(hello='HELLO')
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

    def sendPlanningRequest(self, points_to_plan):
        data_stream = BytesIO()
        np.savez_compressed(data_stream, sid=points_to_plan[0],
                            tool=points_to_plan[1], joint=points_to_plan[2])
        data_stream.seek(0)
        encoded_string = base64.b64encode(data_stream.read())
        self.tp_websocket.send(encoded_string)

    def sendMessage(self, **kwargs):
        data_stream = BytesIO()
        #data_stream.seek(0)
        np.savez_compressed(data_stream, **kwargs)
        data_stream.seek(0)
        encoded_string = base64.b64encode(data_stream.read())
        #self.web_socket.send(self.decode_bytes(data_stream))
        self.tp_websocket.send(encoded_string)

    # def sendExecutedPointData(self, message, planned_points, executed_points):
    #     data_stream = BytesIO()
    #     np.savez_compressed(data_stream, message=message, planned_points=planned_points, executed_points=executed_points)
    #     data_stream.seek(0)
    #     encoded_string = base64.b64encode(data_stream.read())
    #     self.tp_websocket.send(encoded_string)

    def resetWebsocketBuffer(self):
        self.tp_websocket.send('RESET')
        while 'RESET' not in self.tp_websocket.recv():
            pass

    def sortAndOutputPoints(self):
        for p in range(0,len(self.planned_point_payload_buffer)):
            planned_point_payload = self.planned_point_payload_buffer[p]
            if planned_point_payload['sid'] == self.current_requested_sequence_id:
                self.current_requested_sequence_id += 1
                self.planned_point_output_queue.put(self.planned_point_payload_buffer.pop(p))
                break
        self.sortAndOutputPoints()

    def enqueuePointsForPlanning(self, tool_space_data, joint_space_data, sequence_slices, begin_sequence, end_sequence):
        #enqueue_point_list = []
        for k in range(begin_sequence, end_sequence):
            move_type = sequence_slices[k][0]
            #self.tp_state.enqueued_sequence_id += 1
            self.tp_state.enqueued_sequence_id = k
            points_to_enqueue = [np.array([self.tp_state.enqueued_sequence_id]), tool_space_data[:, sequence_slices[k][1]], joint_space_data[:, sequence_slices[k][1]]]

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

    def flushDataFeedbackToTP(self):
        while not self.synchronizer.q_trajectory_planner_data_return_queue.empty():
            data_to_return = self.synchronizer.q_trajectory_planner_data_return_queue.get()
            self.sendMessage(stepgen='STEPGEN', planned_points=data_to_return['commanded'], executed_points=data_to_return['stepgen'])