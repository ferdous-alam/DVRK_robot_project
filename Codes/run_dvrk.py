import numpy as np
from blockO_trajectory_gen import main


# def run_dvrk(trajectory):
#     """Iteratively writes the list of time-scaled trajectories, along each path section, to the dVRK ROS robot."""
#     for Transform in trajectory:
#         # TODO add wait() cmd, and cp command
#         pass
#
#

_, _, tau = main()
np.save('tau.npy', tau)
print('completed!')
