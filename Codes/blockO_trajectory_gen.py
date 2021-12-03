import numpy as np
import math
import sys

# ***** INPUT PARAMETERS *****

# Block "O"
ORIGIN_OFFSET = np.array([0, 0, 0])
SCALE = 1  # Block "O" scaling
ORIENTATION = np.array([  # frame orientation of Block "O"
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]])
# Trapezoidal Time Scaling, s(t), NOTE: v^2/a < 1
# pure translation
VEL_TRAVEL = 1  # ?/sec  # TODO units of dVRK
ACCEL_TRAVEL = 1  # ?/sec^2
# pure rotation
VEL_ROT = 1  # rad/sec
ACCEL_ROT = 1  # rad/sec^2

# ***** PATH GENERATION PROGRAM *****

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

modified_waypoints_ordered = SCALE * np.dot(ORIENTATION, baseline_waypoints_ordered.T).T + ORIGIN_OFFSET
print(modified_waypoints_ordered)

# normal vector to Block "O" plane
r_normal_vec = np.cross(modified_waypoints_ordered[0], modified_waypoints_ordered[1])  # w0 x w1
r_normal_unit = r_normal_vec / np.linalg.norm(r_normal_vec)  # get unit vector

# Precompute ordered list of homogenous transformas at discrete waypoint coordinates
T_list = []
for i, _ in enumerate(modified_waypoints_ordered):
    try:
        w_start = modified_waypoints_ordered[i]
        w_end = modified_waypoints_ordered[i + 1]
        # print(i, i+1)
        # print(w_start, w_end)

        # unit vector parallel to travel path
        r_parallel_unit = (w_start - w_end) / np.linalg.norm(w_start - w_end)  # (wi - wi+1)/||wi - wi+1||

        # third, dependent unit vector of rotation
        # TODO order matters here, must align with {EE} with (+) or multiply (-)
        r_x = r_parallel_unit  # TODO verify orientation
        r_z = r_normal_unit  # TODO verify orientation
        r_y = np.cross(r_x, r_z) / np.linalg.norm(np.cross(r_x, r_z))

        # build common rotation matrix
        R = np.column_stack((r_x, r_y, r_z))

        # build transformations, T = (R,w)
        T_start = np.vstack([np.column_stack((R, w_start)), np.array([0, 0, 0, 1])])
        T_end = np.vstack([np.column_stack((R, w_end)), np.array([0, 0, 0, 1])])

        # add to list, must append to avoid overwrite of window indices
        T_list.append(T_start)
        T_list.append(T_end)

    except IndexError:  # windowing 2 full waypoints at a time, indices beyond at end
        print('\nWaypoint Transformations Generated\n')
        print(T_list)
        break


# ***** TRAJECTORY GENERATION PROGRAM *****


def matrix_exp(w, theta) -> np.ndarray:
    """Rodrigues' formula for rotation matrix given axis-angle, (w,theta), so(3) --> SO(3)."""
    return np.identity(3) + math.sin(theta) * skew_sym_to_matrix(w) + (1 - math.cos(theta)) * skew_sym_to_matrix(w) ** 2


def matrix_log(r: np.ndarray) -> tuple:
    """Matrix Logarithm algorithm for rotation matrix, SO(3) --> so(3). Returns tuple (w, theta), theta in [0, pi]."""
    if np.array_equal(r, np.identity(3)):
        return None, 0
    elif np.trace(r) == -1:
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

        return w, math.pi
    elif np.trace(r) != -1:
        theta = math.acos(0.5 * (np.trace(r) - 1))
        return skew_sym_to_vec((r - r.T) / (2 * math.sin(theta))), theta
    else:
        sys.exit('Matrix Log Error')


def skew_sym_to_matrix(w: np.ndarray) -> np.ndarray:
    """Create 3x3 skew symmetric matrix from axis vector."""
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def skew_sym_to_vec(m: np.ndarray) -> np.ndarray:
    """Create 3x1 vector form skew symmetric matrix."""
    return np.array([m[2, 1], m[0, 2], [1, 0]])


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
            print('Pure Translation')
            v = VEL_TRAVEL
            a = ACCEL_TRAVEL
        elif np.array_equal(p_start, p_end):  # pure rotation movement
            print('Pure Rotation')
            v = VEL_ROT
            a = ACCEL_ROT
        else:
            sys.exit('Path Error')

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
            if 0 <= t or t <= v:
                s = 0.5 * a * t ** 2
            elif v / a < t or t <= T - v / a:
                s = v * t - v ** 2 / (2 * a)
            elif T - v / a < t or t <= T:
                s = (2 * a * v * T - 2 * v ** 2 - a ** 2 * (t - T) ** 2) / (2 * a)
            else:  # handle when mod(T, ts) != 0, at end of path
                print(f'\nPath section {i} is {t - T} seconds beyond desired length\n')
                s = 1  # finish path
                sys.exit('Time-Scaling Error')  # TODO remove once satisfied with handling

            # calculate R(s), p(s) of T=(R,p)
            p_s = p_start + s * (p_end - p_start)
            R_s = R_start * matrix_exp(*matrix_log(R_start.T * R_end) * s)  # TODO fix arg bugs, tuple * multiple
            # create transform and add to trajectory
            T_s = np.vstack([np.column_stack((R_s, p_s)), np.array([0, 0, 0, 1])])
            trajectory.append(T_s)
    except IndexError:  # windowing 2 full transformations at a time, indices beyond at end
        break

# ***** OUTPUT TRAJECTORY TO dVRK ROBOT *****

for Transform in trajectory:
    pass
    # TODO add wait() cmd, and cp command

