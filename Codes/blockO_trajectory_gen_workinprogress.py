import numpy as np
import math
import sys

# TODO need to figure -0 out, EE orientation, ROS communication with 200 Hz
"""
ISSUES: 
1) floating-point imprecision leads to:
    (ex: see matrix log function)
    - need to round for any boolean comparisons
    - need to round for arccos functions to be within range
    - negative (-) zero to handle - equality comparisons challenging with +0, must use np.(allclose or isclose, NOT isequal) & math.isclose
    - negative 0 (-0.0) in rotation matrices 
"""

# ***** USER-INPUT PARAMETERS (Global Vars) *****
# Block "O"
ORIGIN_OFFSET = np.array([0, 0, -1])
SCALE = 1  # Block "O" scaling, baseline is such that the bottom horizontal line is 10 cm
ORIENTATION = np.array([  # frame orientation of Block "O"
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]])
# Trapezoidal Time Scaling, s(t), NOTE: v^2/a <= 1
# pure translation speeds
VEL_TRAVEL = 0.1  # m/sec
ACCEL_TRAVEL = 0.1  # m/sec^2
# pure rotation speeds
VEL_ROT = 0.5  # rad/sec
ACCEL_ROT = 1   # rad/sec^2


# ***** PATH GENERATION PROGRAM *****


def main():
    """Run as main function to enact all trajectory generation. Returns modified waypoint data and the trajectory."""
    # ***** BLOCK "O" MODIFICATION *****
    modified_waypoints_ordered = modify_blockO()
    # ***** GENERATE CORNER-WAYPOINT TRANSFORMATIONS *****
    waypoint_transformations = generate_waypoint_transformations(modified_waypoints_ordered)
    # ***** GENERATE ALL TRAPEZOIDAL TIME-SCALED TRANSFORMATIONS *****
    trajectory = generate_trapezoidal_tranformations(waypoint_transformations)
    # ***** OUTPUT TRAJECTORY TO dVRK ROBOT *****
    run_dvrk(trajectory)
    return modified_waypoints_ordered, waypoint_transformations, trajectory


def run_dvrk(trajectory):
    """Iteratively writes the list of time-scaled trajectories, along each path section, to the dVRK ROS robot."""
    for Transform in trajectory:
        # TODO add wait() cmd, and cp command
        pass


def modify_blockO():
    """
    Modifies the basleine Block "O" coordinates, measured w.r.t the horizontal bottom section being 10cm. A scaling,
    origin offset, and reorientation can be performed to collectively modulate the coordinates, and resulting path.
    """
    # Block "O" default coordinate measurements, defined with 0.1m base, coordinates in [m]
    baseline_waypoints_ordered = np.array([
        [0.048, 0, 0],  # w0
        [0, 0.048, 0],  # w1
        [0, 0.208, 0],  # w2
        [0.048, 0.256, 0],  # w3
        [0.148, 0.256, 0],  # w4
        [0.196, 0.208, 0],  # w5
        [0.196, 0.048, 0],  # w6
        [0.148, 0, 0],  # w7
        [0.048, 0, 0]], dtype='double')  # w8/w0
    # modulation calculation
    modulated_waypoints = SCALE * np.dot(ORIENTATION, baseline_waypoints_ordered.T).T + ORIGIN_OFFSET
    print("\nModulated Block \"O\" Coordinates:")
    for i, waypoint in enumerate(modulated_waypoints):
        print(f'\tw{i}: {waypoint}')
    return modulated_waypoints


def generate_waypoint_transformations(modified_waypoints_ordered):
    """
    Given the corner-waypoints coordinates of the Block "O", the transformations for the path are generated. The path is
    defined such that pure translation happens between corners, and pure rotation happens at each corner, between pure
    translations. During the pure translations, the EE frame is oriented such that one axis is always parallel to the
    straight-line path. For all paths, the EE frame is oriented such that another axis is always perpendicular to the
    plan of the Block "O" defined by w0 x w1.
    From start to finish, this results in 18 total transformations, with 2 per corner.
    """
    # normal vector to Block "O" plane
    r_normal_vec = np.cross(modified_waypoints_ordered[0], modified_waypoints_ordered[1])  # w0 x w1
    r_normal_unit = r_normal_vec / np.linalg.norm(r_normal_vec)  # get unit vector

    # Precompute ordered list of homogenous transforms at discrete waypoint coordinates
    T_list = []
    print('\nWaypoint Transformations Generated:')
    for i, _ in enumerate(modified_waypoints_ordered):
        try:
            w_start = modified_waypoints_ordered[i]
            w_end = modified_waypoints_ordered[i + 1]
            # print(i, i+1)
            # print(w_start, w_end)

            # unit vector parallel to travel path
            r_parallel_unit = (w_start - w_end) / np.linalg.norm(w_start - w_end)  # (wi - wi+1)/||wi - wi+1||

            # third, dependent unit vector of rotation
            # RViz simulation orientation
            # r_x = -r_parallel_unit
            # r_z = -r_normal_unit
            # r_y = np.cross(r_x, r_z) / np.linalg.norm(np.cross(r_x, r_z))
            # dVRK Physical Robot orientation
            r_x = -r_parallel_unit
            r_z = -r_normal_unit
            r_y = np.cross(r_x, r_z) / np.linalg.norm(np.cross(r_x, r_z))

            # build common rotation matrix
            R = np.column_stack((r_x, r_y, r_z))

            # build transformations, T = (R,w) in SO(3)
            T_start = np.vstack([np.column_stack((R, w_start)), np.array([0, 0, 0, 1])])
            T_end = np.vstack([np.column_stack((R, w_end)), np.array([0, 0, 0, 1])])

            # add to list, must append to avoid overwrite of window indices
            T_list.append(T_start)
            T_list.append(T_end)

            print(f'\n\tT{i}{i}:\n{T_start}\n\n\tT{i}{i+1}:\n{T_end}')
        except IndexError:  # windowing 2 full waypoints at a time, indices beyond at end
            break
    return T_list


def generate_trapezoidal_tranformations(T_list):
    """
    Given the list of corner-waypoint transformations, the straight-line interpolated transformations are computed
    from a trapezoidal time-scaling. The ordered list of the generated time-scaled trajectory is returned.
    """
    # Time-Scaling
    if VEL_TRAVEL ** 2 > ACCEL_TRAVEL or VEL_ROT ** 2 > ACCEL_ROT:  # verify input
        raise Exception('Trapezoidal Time-Scaling must satisfy v^2 <= a')

    rosrate_Hz = 200  # [Hz]
    ts = 1 / rosrate_Hz  # [sec]
    trajectory = []  # init list of time-scaled transformations
    for i, _ in enumerate(T_list):
        try:
            # TODO add go to origin, and return home transformations to start and end path ????
            T_start = T_list[i]
            T_end = T_list[i + 1]

            # parse R,p from T
            R_start = T_start[0:3, 0:3]
            p_start = T_start[0:3, 3]
            R_end = T_end[0:3, 0:3]
            p_end = T_end[0:3, 3]

            if np.array_equal(R_start, R_end):  # pure travel movement
                msg = 'Pure Translation'
                v = VEL_TRAVEL
                a = ACCEL_TRAVEL
            elif np.array_equal(p_start, p_end):  # pure rotation movement
                msg = 'Pure Rotation'
                v = VEL_ROT
                a = ACCEL_ROT
            else:
                sys.exit('Path Error')
            print(f'\n{msg}\n')

            # Total time for straight-line path in task-space
            T = (a + v ** 2) / (v * a)
            t = 0
            j = 0
            # Time-Scaling @ 200Hz
            while t <= T:
                # Current time given rosrate
                t = j * ts  # [0, ts, 2*ts, 3*ts, ..., T], step through @ 200 Hz
                j += 1  # index timestep

                # trapezoidal scaling
                if 0 <= t <= v / a:
                    s = 0.5 * a * t ** 2
                elif v / a < t <= T - v / a:
                    s = v * t - v ** 2 / (2 * a)
                elif T - v / a < t <= T:
                    s = (2 * a * v * T - 2 * v ** 2 - a ** 2 * (t - T) ** 2) / (2 * a)
                else:  # handle when mod(T, ts) != 0, at end of path
                    print(f'\nNOTE: Path section {i} is {round(t - T),4} seconds beyond desired length')
                    s = 1  # finish path

                # calculate R(s), p(s) of T=(R,p)
                p_s = p_start + s * (p_end - p_start)
                w, theta = matrix_log(np.matmul(R_start.T, R_end))
                print(np.matmul(R_start.T, R_end))
                print(w,theta, s)
                R_func = matrix_exp(w, theta * s)  # exp(log(R_startT*R_end)*s))
                R_s = np.matmul(R_start, R_func)
                print(R_s)
                R_s[R_s == 0.] = 0.  # TODO does this get rid of -0.0 properly?
                print(R_s)
                # create transform from (R,p) & add to trajectory list
                T_s = np.vstack([np.column_stack((R_s, p_s)), np.array([0, 0, 0, 1])])
                trajectory.append(T_s)
                print(f'\n{msg} from w{i//2} to w{i//2 + (i+1)%2}:\ns({round(t,3)}) = {round(s,3)}, of T = {round(T,3)} sec:\n T=\n{T_s}')

        except IndexError:  # windowing 2 full transformations at a time, indices beyond at end
            break
    return trajectory


def matrix_exp(w, theta) -> np.ndarray:
    """Rodrigues' formula for rotation matrix given axis-angle, (w,theta), so(3) --> SO(3)."""
    return np.identity(3) + math.sin(theta) * skew_sym_to_matrix(w) + (1 - math.cos(theta)) * np.linalg.matrix_power(skew_sym_to_matrix(w), 2)


def matrix_log(r: np.ndarray) -> tuple:
    """Matrix Logarithm algorithm for rotation matrix, SO(3) --> so(3). Returns tuple (w, theta), theta in [0, pi]."""
    if np.allclose(r, np.identity(3)):  # TODO verify proper usage
        print('no rotation***')
        w = np.array([0, 0, 0])  # w, undefined
        theta = 0
    elif math.isclose(np.trace(r), -1.0):  # Since given pure rot and pure trans trajectory, this should NOT be used
        print('trace')
        const = [0, 0, 0]
        if r[2, 2] >= -1:  # r33
            i = 2
        elif r[1, 1] >= -1:  # r22
            i = 1
        elif r[0, 0] >= -1:  # r11
            i = 0
        else:
            sys.exit('Matrix Log Error - R poorly defined')
        const[i] = 1
        w = np.array([const[1] + r[0, i], const[1] + r[1, i], const[2] + r[2, i]]) / math.sqrt(2 * (1 + r[i, i]))
        theta = math.pi
    elif not math.isclose(np.trace(r), -1.0):  # TODO verify proper usage
        print('no trace')
        part = 0.5 * (np.trace(r) - 1)
        # handle floating-point imprecision error, enforce arcos domain is [-1, 1]
        if math.isclose(part, 1) or math.isclose(part, -1):  # TODO verify proper usage
            part = round(part, 2)
        theta = math.acos(part)
        w = skew_sym_to_vec((r - r.T) / (2 * math.sin(theta)))
    else:
        sys.exit('Matrix Log Error')
    return w, theta


def skew_sym_to_matrix(w: np.ndarray) -> np.ndarray:
    """Create 3x3 skew symmetric matrix from axis vector."""
    return np.array([[0, -1*w[2], w[1]], [w[2], 0, -1*w[0]], [-1*w[1], w[0], 0]])


def skew_sym_to_vec(m: np.ndarray) -> np.ndarray:
    """Create 3x1 vector form skew symmetric matrix."""
    return np.array([m[2, 1], m[0, 2], m[1, 0]])


# run script
modified_waypoints_ordered, waypoint_transformations, trajectory = main()