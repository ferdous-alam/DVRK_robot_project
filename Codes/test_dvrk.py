import numpy as np


def test_dvrk(p):
    tau = np.load('tau.npy')
    start = p.measured_cp()
    for i in range(len(tau)):
        start.p[0] = tau[i][0, 3]
        start.p[1] = tau[i][1, 3]
        start.p[2] = tau[i][2, 3] - 0.060

        p.move_cp(start).wait()
