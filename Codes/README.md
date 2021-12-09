#dVRK Software
This directory contains all files & codes related to the DVRK robot project.

This involves inverse kinematic MATLAB script.

And a series of Python scripts to generate/export Block "O" trajectory and then communicate it the dVRK PSM robot
- "blockO_trajectory_generation.py" generates and exports a trapezoidal time-scaled trajectory given Block "O" coordinate definitions and time-scaling velocity and acceleration parameters
- "run_dvrk.py" imports a given trajectory and communicates it to the physical dVRK PSM
- "gen_trajectory.py" & "run_dvrk_terminal.py" are just scripts to help simplify running these programs from the terminal, but they are not necessary.

