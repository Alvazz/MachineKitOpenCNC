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
                    payload, keys = self.decodePayload(b64payload)
                    if 'pid_ack' in keys:
                    #if type[0] == 'POINTSACK':
                        self.tp_state.point_ack_id = payload['pid_ack'].item()
                        if self.tp_state.point_ack_id == self.tp_state.current_requested_sequence_id:
                            self.tp_state.point_id_ack_event.set()
                    elif 'hello_ack' in keys:
                        self.tp_state.tp_connected_event.set()
                    elif 'planned_points' in keys:
                        self.planned_point_payload_buffer.append(payload)
                        self.planned_point_output_queue.put((payload['pid'], payload['planned_points']))
                        self.synchronizer.q_trajectory_planner_planned_move_queue.put(pncLibrary.Move(payload['planned_points'][:,0:self.machine.number_of_joints], 'remotely_planned'))
                        self.tp_state.current_received_sequence_id = payload['pid'].item()
                        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                                     pncLibrary.printout_subsequence_enqueued_string, self.name,
                                                                     self.tp_state.current_received_sequence_id)

                        if self.tp_state.current_requested_sequence_id == self.tp_state.current_received_sequence_id:
                            self.tp_state.matching_sequence_received_event.set()
                            self.tp_state.send_next_block_event.set()

                except websocket.WebSocketTimeoutException:
                    pass

    def decodePayload(self, payload):
        byte_stream = BytesIO(base64.b64decode(payload.encode('utf-8')))
        byte_stream.seek(0)
        decoded_payload = np.load(byte_stream)
        return decoded_payload, decoded_payload.keys()

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

        self.planned_point_payload_buffer = []
        self.joint_data_file = "machine"
        self.tool_data_file = "tool"
        self.conversion_factor_dist = 1

        self.startup_event = Event()

    def run(self):
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_launch_string, current_process().name,
                                                     self.name)
        pncLibrary.waitForThreadStart(self, WebsocketReceiver)
        self.startup_event.set()

        while not self.openConnection():
            pass

        self.connectToTP()

        self.tool_points, self.machine_points, self.retraction_flags = pncLibrary.TP.load_sp_data(self.machine.raw_point_files_path, self.joint_data_file, self.tool_data_file)
        self.contiguous_sequences = pncLibrary.TP.obtain_contiguous_sequences(self.retraction_flags.astype(int))
        self.enqueuePointsForPlanning(self.tool_points, self.machine_points, self.contiguous_sequences, 0, 10)

        while self.synchronizer.t_run_cloud_trajectory_planner_event.is_set() and self.tp_state.tp_connected_event.is_set():
            if self.tp_state.send_next_block_event.wait(self.machine.event_wait_timeout) and self.synchronizer.tp_plan_motion_event.wait(self.machine.event_wait_timeout):
                self.tp_state.point_id_ack_event.clear()
                self.tp_state.matching_sequence_received_event.clear()

                points_to_plan = self.raw_point_queue.get()

                self.tp_state.current_requested_sequence_id = points_to_plan[0]
                self.tp_state.send_next_block_event.clear()
                self.sendPlanningRequest(points_to_plan)

                self.flushDataFeedbackToTP()
                # while not self.synchronizer.q_trajectory_planner_data_return_queue.empty():
                #     data_to_return = self.synchronizer.q_trajectory_planner_data_return_queue.get()
                #     self.sendMessage(stepgen='STEPGEN', planned_points=data_to_return['commanded'], executed_points=data_to_return['stepgen'])

        self.tp_websocket.close()
        pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                     self.machine.thread_terminate_string, current_process().name,
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
        else:
            pncLibrary.printStringToTerminalMessageQueue(self.synchronizer.q_print_server_message_queue,
                                                         pncLibrary.printout_trajectory_planner_connection_failure_string,
                                                         self.name, self.remote_tp_name)

    def sendPlanningRequest(self, points_to_plan):
        data_stream = BytesIO()
        np.savez_compressed(data_stream, pid=points_to_plan[0],
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
        for k in range(begin_sequence, end_sequence):
            self.tp_state.enqueued_sequence_id += 1
            self.raw_point_queue.put((np.array([self.tp_state.enqueued_sequence_id]), tool_space_data[:, sequence_slices[k][1]], joint_space_data[:, sequence_slices[k][1]]))
            #self.raw_point_queue.put((self.path_serial_number, tool_space_data, joint_space_data))

    def flushDataFeedbackToTP(self):
        while not self.synchronizer.q_trajectory_planner_data_return_queue.empty():
            data_to_return = self.synchronizer.q_trajectory_planner_data_return_queue.get()
            self.sendMessage(stepgen='STEPGEN', planned_points=data_to_return['commanded'], executed_points=data_to_return['stepgen'])