import numpy as np
import rospy

import tf_conversions.posemath as pm
import PyKDL


def test_dvrk(p):
    tau = np.load('tau.npy')
    start = p.setpoint_cp()
    mcp = []
    for numpy_transformation in tau:
        kdl_transformation = pm.fromMatrix(numpy_transformation)  # convert from np to kdl transformation
	pos = kdl_transformation.p  # extract position vector
	rot = kdl_transformation.M  # extract rotation matrix
	print(rot,pos)
	# communicate to robot	
	start.p = pos
	start.M = rot
        p.move_cp(start)


	#cp_val = p.measured_cp()	
	#print(cp_val)
	#mcp.append(p.measured_cp())	
	# rate.sleep()

    #np.save('mcp.npy', mcp)
