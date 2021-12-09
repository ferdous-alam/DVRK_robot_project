"""
This program imports an np trajectory, and iterates it through it, communicating the trajectory transformations. This
script can be run directly, or called from "run_dvrk_terminal.py"
"""


import numpy as np
import dvrk
import tf_conversions.posemath as pm


def run_dvrk(p, filename='tau.npy', movement_type='all'):
    """
    This imports an np trajectory, and iterates it through it, communicating the trajectory transformations.
    :param p: dVRK ROS instance
    :param filename: name of np trajectory to import and then execute
    :param movement_type: desired enforcement movement of dVRK, either
        (1) purely positional as = 'p',
        (2) purely rotational as = 'r',
     or (3) both position and rotation as = 'all' (default)
    """

    tau = np.load(filename)
    start = p.setpoint_cp()
    mcp = []
    for numpy_transformation in tau:
        kdl_transformation = pm.fromMatrix(numpy_transformation)  # convert from np to kdl transformation
        pos = kdl_transformation.p  # extract position vector
        rot = kdl_transformation.M  # extract rotation matrix
        # communicate to robot given movement type
        if movement_type == 'p':
            start.p = pos
            print(pos)
        elif movement_type == 'r':
            start.M = rot
            print(rot)
        elif movement_type == 'all':
            start.p = pos
            start.M = rot
            print(rot, pos)

        else:
            print('Movement Type input error, must be either "p","r", or "all"')

        # move robot
        p.move_cp(start)

        # for saving and exporting measured results from physical dVRK
        # cp_val = p.measured_cp()
        # print(cp_val)
        # mcp.append(p.measured_cp())
        # rate.sleep()
        # np.save('mcp.npy', mcp)
    print('Block /"O/" Trajectory Completed!')


if __name__ == '__main__':
    # generate PSM instance
    p = dvrk.psm("PSM1")
    p.enable()  # check, must be True
    p.home()  # check
    # run trajectory communication fxn that moves robot
    run_dvrk(p, filename='tau.npy', movement_type='all')
