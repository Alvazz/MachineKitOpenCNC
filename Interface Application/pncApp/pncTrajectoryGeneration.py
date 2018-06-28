######################## Trajectory Generation ########################
import math, csv, numpy as np

def importPoints(machine, file):
    ##FIXME check for overtravel
    points = np.array(list(csv.reader(open(file, "rt"), delimiter=" "))).astype("float")[:,:machine.number_of_joints]
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

def generateHoldPositionPoints(machine, hold_time=1, position = [np.nan]):
    if any(np.isnan(position)):
        position_to_hold = machine.current_position
    else:
        position_to_hold = np.array(position)

    joint_position_samples = np.zeros([int(np.clip(np.round(hold_time/machine.servo_dt),1,np.inf)), int(machine.number_of_joints)]) + position_to_hold
    return joint_position_samples

def generateMovePoints(machine, end_points, start_points = [np.nan], move_velocity = 0, max_joint_velocities = -1, max_joint_accelerations = -1, move_type = 'trapezoidal'):
    fallthrough_points = generateHoldPositionPoints(machine, 0, start_points)
    #FIXME check for out of limit move generation
    if any(end_points[k] < machine.limits[k][0] for k in range(0,len(end_points))) or any(end_points[k] > machine.limits[k][1] for k in range(0,len(end_points))):
        print('move exceeds bounds')
        return fallthrough_points

    #Handle arguments
    if any(np.isnan(start_points)):
        start_points = machine.current_position
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
            return fallthrough_points

        move_direction = np.sign(move_vector)
        #Check if we can reach cruise for each joint
        max_move_velocity = np.zeros([1,machine.number_of_joints])[0]
        for joint in range(0,machine.number_of_joints):
            no_cruise_time_to_center = math.sqrt(
                math.fabs(move_vector[joint]) / max_joint_accelerations[joint])
            if move_vector[joint] == 0:
                #Do not move this joint
                print('joint ' + str(joint) + ' holding position')
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
                print('joint ' + str(joint) + ' can reach cruise')
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