import numpy as np
import pickle, csv
import socket, time

data_path = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Experimental Data\\'
csv_output_path = data_path + 'CSV Output\\'

file_analysis = True
time_analysis = True
if file_analysis:
    #Head top
    head_top_database = pickle.load(open(data_path + 'Head Data\\head_pass3_short_top_try2.scpr_Tool Pass #3_2018.12.14-09.49.59_database','rb'))
    head_bottom_database = pickle.load(open(data_path + 'Head Data\\head_pass3_short_bottom.scpr_Tool Pass #3_2018.12.14-10.05.17_database','rb'))
    candleholder_top_database = pickle.load(open(data_path + 'Candleholder Data\\candleholder_top.scpr_Tool Pass #2_2018.12.14-10.33.17_database','rb'))
    candleholder_bottom_database = pickle.load(open(data_path + 'Candleholder Data\\candleholder_bottom_good.scpr_Tool Pass #2_2018.12.14-10.50.10_database','rb'))



    db = head_bottom_database
    commands = np.vstack((db.ARCHIVED_COMMANDED_SERVO_POSITIONS, db.COMMANDED_SERVO_POSITIONS))
    command_times = np.vstack((db.ARCHIVED_INTERPOLATED_POLYLINE_TRANSMISSION_TIMES, db.POLYLINE_TRANSMISSION_TIMES))
    stepgens = np.vstack((db.ARCHIVED_STEPGEN_FEEDBACK_POSITIONS, db.STEPGEN_FEEDBACK_POSITIONS))
    stepgen_times = np.vstack((db.ARCHIVED_RTAPI_CLOCK_TIMES, db.RTAPI_CLOCK_TIMES))
    try:
        getattr(db, 'ARCHIVED_ENCODER_FEEDBACK_POSITIONS')
        encoders = np.vstack((db.ARCHIVED_ENCODER_FEEDBACK_POSITIONS, db.ENCODER_FEEDBACK_POSITIONS))
        encoder_times = np.vstack((db.ARCHIVED_SERIAL_RECEIVED_TIMES, db.SERIAL_RECEIVED_TIMES))
    except AttributeError:
        encoders = db.ENCODER_FEEDBACK_POSITIONS
        encoder_times = db.SERIAL_RECEIVED_TIMES

    np.savetxt(csv_output_path + "sp_tp_commands.csv", commands, delimiter=',')
    np.savetxt(csv_output_path + "sp_tp_command_times.csv", command_times, delimiter=',')
    np.savetxt(csv_output_path + "sp_stepgens.csv", stepgens, delimiter=',')
    np.savetxt(csv_output_path + "sp_stepgen_times.csv", stepgen_times, delimiter=',')
    np.savetxt(csv_output_path + "sp_encoders.csv", encoders, delimiter=',')
    np.savetxt(csv_output_path + "sp_encoder_times.csv", encoder_times, delimiter=',')

    #Export relevant data to csv
    pull_times = db.PULL_TIMES
    push_times = db.PUSH_TIMES
    #state_update_pull_times = db.PULL_TIMES
    state_update_pull_times = [pull_times[i,0] if len(pull_times[i,1]) == 1 else -1 for i in range(0, np.shape(pull_times)[0])]
    state_update_pull_times = np.array(state_update_pull_times)[np.where(np.array(state_update_pull_times)>-1)]

    cam_pull_times = [pull_times[i,0] if len(pull_times[i,1]) > 1 else -1 for i in range(0, np.shape(pull_times)[0])]
    cam_pull_times = np.array(cam_pull_times)[np.where(np.array(cam_pull_times)>-1)]

    np.savetxt(csv_output_path + "raw_pull_times.csv", pull_times[:,0], delimiter=',')
    np.savetxt(csv_output_path + "state_update_pull_times.csv", state_update_pull_times, delimiter=',')
    np.savetxt(csv_output_path + "cam_pull_times.csv", cam_pull_times, delimiter=',')
    np.savetxt(csv_output_path + "raw_push_times.csv", push_times[:,0], delimiter=',')

if time_analysis:
    # For timing GCode files
    rsh_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #rsh_socket.connect(("127.0.0.1", 5007))
    rsh_socket.connect(("129.1.15.5", 5007))

    start_time = time.clock()
    run_flag = True
    program_started = False

    rsh_socket.send('hello EMC roby 1\n'.encode())
    rsh_socket.send('set enable EMCTOO\n'.encode())
    recvd = rsh_socket.recv(2048)
    while run_flag:
        rsh_socket.send('get program_status\n'.encode())
        recvd = rsh_socket.recv(2048).decode().strip()

        if recvd == "PROGRAM_STATUS IDLE" and program_started:
            run_flag = False
        elif recvd == "PROGRAM_STATUS RUNNING" and not program_started:
            print('program started')
            start_time = time.clock()
            program_started = True
        time.sleep(0.05)

print('Toolpath time is ' + str(time.clock()-start_time))

