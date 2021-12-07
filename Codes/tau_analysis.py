import numpy as np
import pandas as pd

tau_hardware = np.load('tau_hardware.npy')
# tau_generated = np.load('tau_generated.npy')

hardware_xyz = np.empty(len(tau_hardware), 3)
for i in range(tau_hardware):
    hardware_xyz[i] = np.array([tau_hardware[i][0,3],tau_hardware[i][1,3],tau_hardware[i][2,3]])

hardware_df = pd.DataFrame(hardware_xyz, columns=['x','y','z'])
hardware_df.to_pickle('tau_hardware_df.csv')

# generated_x = tau_generated[:, 0, 3]
# generated_y = tau_generated[:, 1, 3]
# generated_z = tau_generated[:, 2, 3]


