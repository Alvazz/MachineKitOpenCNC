######################## Trajectory Generation ########################
import math, csv, numpy as np

def checkMoveOvertravel(point_samples, limits):
    if not (point_samples < limits[0]).any() or (point_samples > limits[1]).any():
        return (False, None, None)
    else:
        negative_overtravel = np.where(point_samples < limits[0])
        positive_overtravel = np.where(point_samples > limits[1])
        #Should be (bool, (line, axis), (line, axis))
        return (True,
                (negative_overtravel[0][0], negative_overtravel[1][0]) if negative_overtravel[0].size else None,
                (positive_overtravel[0][0], positive_overtravel[1][0]) if positive_overtravel.size else None)
        #return (False, (np.where(move.point_samples < limits[0]), np.where(move.point_samples < limits[0])))
    # for k in range(0, move.point_samples.shape[0]):
    #     if any(move.point_samples[k] < limits[0]) or any(move.point_samples[k] > limits[1]):
    #         return (False, k)
    # return (True, None)

def convertMotionCS(machine, mode, points):
    if mode == 'table center':
        return points + machine.machine_table_center_zero
    elif mode == 'absolute':
        return points - machine.machine_table_center_zero

def importPoints(machine, file):
    ##FIXME check for overtravel
    points = convertMotionCS(machine, 'absolute', np.array(list(csv.reader(open(file, "rt"), delimiter=" "))).astype("float")[:,:machine.number_of_joints])
    return points

def padAndShapeAxisPoints(points, polylines, blocklength):
    pad_points = np.lib.pad(points, ((0, blocklength - (np.size(points, 0) % blocklength)), (0, 0)), 'constant',
                            constant_values=points[-1])
    shape_points = pad_points.reshape((-1, blocklength), order='C')
    return np.pad(shape_points, ((0, polylines - (np.size(shape_points, 0) % polylines)), (0, 0)), 'constant',
                  constant_values=shape_points[-1, -1])

def formatPoints(points, polylines, block_length):
    axis_coords = []
    #FIXME fix if not divisible by polylines*blocklength
    for axis in range(points.shape[1]):
        axis_coords.append(padAndShapeAxisPoints(np.asarray([points[:, axis]]).T, polylines, block_length))
    return np.asarray(axis_coords).transpose(1, 2, 0)

def _units_transform(data, dist_slice=slice(None, 3, None), rot_slice=slice(3, None, None),
                     correct_dist=True, correct_rot=True, conversion_factor_dist = 1):
    # Convert to radian if rotational axis
    if correct_rot:
        data[rot_slice, :] = (data[rot_slice, :] * np.pi) / 180
    if correct_dist:
        data[dist_slice, :] = data[dist_slice, :] * conversion_factor_dist
    return data

# Returns desired slices of tool and joint data files.
# Tool file order - x,y,z,volume,a,b
# Joint file order - X,Y,Z,A,B,S
def load_sp_data(data_folder, joint_data_file, tool_data_file,
                 data_slice=slice(None, None, None),
                 subsample_slice=slice(None, None, None)):
    tool_data = np.loadtxt(data_folder + tool_data_file, skiprows=1)
    joint_data = np.loadtxt(data_folder + joint_data_file, skiprows=1)

    return load_sp_data_from_arrays(joint_data, tool_data, subsample_slice)

# Load SculptPrint data from numpy arrays
def load_sp_data_from_arrays(joint_data, tool_data,
                             subsample_slice=slice(None, None, None)):
    # Joint file indexing order is a mapping from n+1 -> n+1 that maps coordinate order -> index in file
    # The last index is for the spindle coordinate.
    joint_file_indexing_order = [1, 4, 2, 5, 6, 3]
    # Joint file indexing order is a mapping from n -> n that maps
    # coordinate order -> index in file -- in the file written out by
    # SculptPrint, the 6th index is the volumes, and the 5th is if this is
    # a flag or not.
    tool_file_indexing_order = [0, 1, 2, 6, 3, 4, 5]

    tool_temp_data = tool_data.T
    joint_temp_data = joint_data.T

    tool_analysis_data = np.zeros((len(tool_file_indexing_order), tool_temp_data.shape[1]))
    for index in np.arange(len(tool_file_indexing_order)):
        tool_analysis_data[index] = tool_temp_data[tool_file_indexing_order[index]]
    tool_analysis_data = _units_transform(tool_analysis_data,
                                               dist_slice=slice(None, 4, None),
                                               rot_slice=slice(4, 5, None), correct_rot=False)

    joint_analysis_data = np.zeros((len(joint_file_indexing_order), joint_temp_data.shape[1]))
    for index in np.arange(len(joint_file_indexing_order)):
        joint_analysis_data[index] = joint_temp_data[joint_file_indexing_order[index]]
    joint_analysis_data = _units_transform(joint_analysis_data)

    return (tool_analysis_data[:, subsample_slice],
            joint_analysis_data[:, subsample_slice],
            tool_analysis_data[-1, subsample_slice].astype(dtype=bool))

# Given data (:, n) and flag(1, n) of bools, split into
# contiguous set of slices based on flag
def obtain_contiguous_sequences(flag):
    last_start_index = 0
    switch_indices = np.where((flag[1:] - flag[:-1]) != 0)[0]
    switch_indices = np.hstack((np.array([-1]), switch_indices))
    if (switch_indices.shape != 0):
        last_start_index = switch_indices[-1]+1

    contiguous_lists = []
    all_same = []
    for index in np.arange(switch_indices.size-1):
        contiguous_lists.append((flag[switch_indices[index]+1],
                                 slice(switch_indices[index]+1,
                                       switch_indices[index+1]+1, None)))
        if __debug__:
           all_same.append(np.any(flag[switch_indices[index]+1: switch_indices[index+1]+1] - flag[switch_indices[index]+1]))
    # if __debug__:
    #        print("All contiguous flags are same", not np.any(np.asarray(all_same)))
    contiguous_lists.append((flag[last_start_index],
                             slice(last_start_index, None, None)))
    return contiguous_lists

def get_combined_data(joint_data, tool_data):
    return (np.vstack((tool_data[:3], tool_data[4:6],
                       tool_data[3], joint_data[:6],
                       tool_data[6])))

def write_combined_data(joint_data, tool_data, filename):
    combined_analysis_data = get_combined_data(joint_data, tool_data).T
    np.savetxt(filename, combined_analysis_data)

################### TRAJECTORY GENERATION ###################

def generateHoldPositionPoints(machine, hold_time=1, position = [np.nan]):
    if any(np.isnan(position)):
        position_to_hold = machine.current_stepgen_position
    else:
        position_to_hold = np.array(position)

    joint_position_samples = np.zeros([int(np.clip(np.round(hold_time/machine.servo_dt),1,np.inf)), int(machine.number_of_joints)]) + position_to_hold
    return joint_position_samples

def generateMovePoints(machine, end_points, start_points = [np.nan], move_velocity = 0, max_joint_velocities = -1, max_joint_accelerations = -1, move_type = 'trapezoidal'):
    #fallthrough_points = generateHoldPositionPoints(machine, 0, start_points)
    #FIXME check for out of limit move generation
    if checkMoveOvertravel(end_points, machine.absolute_axis_travel_limits)[0]:
    #if any(end_points[k] < machine.table_center_axis_travel_limits[k][0] for k in range(0,len(end_points))) or any(end_points[k] > machine.table_center_axis_travel_limits[k][1] for k in range(0,len(end_points))):
        print('move exceeds bounds')
        return generateHoldPositionPoints(machine, 0, start_points)

    #Handle arguments
    if any(np.isnan(start_points)):
        start_points = machine.current_stepgen_position
    if max_joint_velocities == -1:
        max_joint_velocities = machine.max_joint_velocity
    if max_joint_accelerations == -1:
        max_joint_accelerations = machine.max_joint_acceleration
    #print('starting move from ')
    #print(start_points)
    #FIXME add capability to scale move velocity
    #Trapezoidal velocity profile generation
    #USAGE:
    #   move_type: string
    #   start_points: 1 x num_axes list
    #   end_points: 1 x num_axes list
    #   move_velocity: float
    #   max_joint_velocities: 1 x num_axes list
    #   max_joint_accelerations: 1 x num_axes list

    if move_type == 'trapezoidal':
        move_vector = (np.asarray(end_points) - np.asarray(start_points))
        if not np.count_nonzero(move_vector):
            # FIXME return 0 if move_vector == 0
            #Null move
            print('move is null')
            return generateHoldPositionPoints(machine, 0, start_points)

        move_direction = np.sign(move_vector)
        #Check if we can reach cruise for each joint
        max_move_velocity = np.zeros([1,machine.number_of_joints])[0]
        for joint in range(0,machine.number_of_joints):
            no_cruise_time_to_center = math.sqrt(
                math.fabs(move_vector[joint]) / max_joint_accelerations[joint])
            if move_vector[joint] == 0:
                #Do not move this joint
                #print('joint ' + str(joint) + ' holding position')
                pass
            elif (max_joint_accelerations[joint] * no_cruise_time_to_center) >= max_joint_velocities[joint]:
                #We can reach cruise phase
                max_move_velocity[joint] = max_joint_velocities[joint]
            else:
                #Cannot reach cruise phase, move is triangular
                max_move_velocity[joint] = max_joint_accelerations[joint] * no_cruise_time_to_center

        #print('move direction')
        #print(move_direction)

        #Calculate move profile for each joint
        time_points = np.zeros([3, machine.number_of_joints])
        for joint in range(0, machine.number_of_joints):
            if max_move_velocity[joint] < max_joint_velocities[joint]:
                #Move too short for cruise phase, acceleration and deceleration intersect at t1=t2
                time_points[0][joint] = max_move_velocity[joint] / max_joint_accelerations[joint]
                time_points[1][joint] = time_points[0][joint]
                time_points[2][joint] = 2*time_points[0][joint]
            else:
                #Time and distance to cruise velocity, t1
                time_points[0][joint] = max_move_velocity[joint] / max_joint_accelerations[joint]
                #print('joint ' + str(joint) + ' can reach cruise')
                #End of move
                time_points[2][joint] = (np.fabs(move_vector[joint]) + (max_move_velocity[joint]*time_points[0][joint]))/max_move_velocity[joint]
                #Start of deceleration phase
                time_points[1][joint] = time_points[2][joint]-time_points[0][joint]
                #sanity check
                #dist_points[2][joint] = dist_points[1][joint] + dist_points[0][joint]

        #Find limiting joint
        slowest_joint = np.where(time_points[-1,:] == np.max(time_points[-1,:]))[0][0]
        #print('slowest joint is ' + str(slowest_joint))
        move_time = time_points[2][slowest_joint]
        #print('move time is ' + str(move_time))

        motion_scale_factors = (np.asarray(time_points[2][:])/move_time)
        # Scale waypoint times for slowest axis
        time_points = np.multiply(np.asarray(time_points), np.divide(1,motion_scale_factors,out = np.zeros_like(motion_scale_factors),where = motion_scale_factors != 0)).tolist()

        #Sample time points to plan trajectories
        servo_times = np.arange(0,math.ceil(move_time/machine.servo_dt)*machine.servo_dt,machine.servo_dt)
        joint_position_samples = np.zeros([np.size(servo_times),machine.number_of_joints])
        #Force initial position
        joint_position_samples[0][:] = start_points

        #State machine for integration
        last_joint_positions = np.zeros([3,machine.number_of_joints]).tolist()
        phase_switch_times = np.zeros([3,machine.number_of_joints]).tolist()

        max_move_velocity = np.multiply(motion_scale_factors,max_move_velocity)
        max_joint_accelerations = np.multiply(np.power(motion_scale_factors,2), max_joint_accelerations)

        for ndx in range(0,np.size(servo_times)):
            t = servo_times[ndx]
            for joint in range(0, machine.number_of_joints):
                #Phase -1: Joint hold position
                if time_points[0][joint] == 0 and time_points[1][joint] == 0 and time_points[2][joint] == 0:
                    joint_position_samples[ndx][joint] = start_points[joint]
                #Phase 1: Acceleration
                elif t < time_points[0][joint]:
                    #We are in phase one (acceleration) for this joint
                    joint_position_samples[ndx][joint] = start_points[joint] + 0.5*move_direction[joint]*max_joint_accelerations[joint]*np.power(t,2)
                    last_joint_positions[0][joint] = joint_position_samples[ndx][joint]
                    phase_switch_times[0][joint] = t
                #Phase 2: either cruise or deceleration
                elif t >= time_points[0][joint] and (t < time_points[1][joint] or time_points[0][joint] == time_points[1][joint]):
                    if time_points[0][joint] != time_points[1][joint]:
                        #We are in cruise phase
                        joint_position_samples[ndx][joint] = last_joint_positions[0][joint] + move_direction[joint]*(t-phase_switch_times[0][joint])*max_move_velocity[joint]
                        last_joint_positions[1][joint] = joint_position_samples[ndx][joint]
                        phase_switch_times[1][joint] = t
                    else:
                        #Move too short for cruise, begin deceleration
                        joint_position_samples[ndx][joint] = last_joint_positions[0][joint] + move_direction[joint]*\
                                                             ((t-phase_switch_times[0][joint])*max_move_velocity[joint] - 0.5*max_joint_accelerations[joint]*np.power(t-phase_switch_times[0][joint],2))
                        last_joint_positions[1][joint] = joint_position_samples[ndx][joint]
                        phase_switch_times[1][joint] = t
                        last_joint_positions[1][joint] = joint_position_samples[ndx][joint]
                        phase_switch_times[1][joint] = t
                #Phase 3: Deceleration
                elif t >= time_points[1][joint]:
                    #print('decelerating joint ' + str(joint))
                    joint_position_samples[ndx][joint] = last_joint_positions[1][joint] + move_direction[joint]*\
                                                         ((t-phase_switch_times[1][joint])*max_move_velocity[joint] -
                                                          0.5 * max_joint_accelerations[joint] * np.power(t-phase_switch_times[1][joint],2))
                    last_joint_positions[2][joint] = joint_position_samples[ndx][joint]
                    phase_switch_times[2][joint] = t


        #Force last position
        joint_position_samples[-1,:] = end_points

        return joint_position_samples