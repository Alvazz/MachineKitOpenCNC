import numpy as np
import pickle, csv

data_path = 'C:\\Users\\robyl_000\\Documents\\Projects\\PocketNC\\Experimental Data\\'
csv_output_path = data_path + 'CSV Output\\'

#Head top
head_top_database = pickle.load(open(data_path + 'Head Data\\head_pass3_short_top_try2.scpr_Tool Pass #3_2018.12.14-09.49.59_database','rb'))
head_bottom_database = pickle.load(open(data_path + 'Head Data\\head_pass3_short_bottom.scpr_Tool Pass #3_2018.12.14-10.05.17_database','rb'))
candleholder_top_database = pickle.load(open(data_path + 'Candleholder Data\\candleholder_top.scpr_Tool Pass #2_2018.12.14-10.33.17_database','rb'))
candleholder_bottom_database = pickle.load(open(data_path + 'Candleholder Data\\candleholder_bottom_good.scpr_Tool Pass #2_2018.12.14-10.50.10_database','rb'))



db = candleholder_bottom_database
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