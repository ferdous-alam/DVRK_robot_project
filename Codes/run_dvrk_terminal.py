"""
Runs robot trajectory on physical PSM simply by running "run_dvrk", called from terminal.
"""


import dvrk
from run_dvrk import run_dvrk


# generate PSM instance
p = dvrk.psm("PSM1")
p.enable()  # check, must be True
p.home()  # check

# run trajectory communication script that moves robot
run_dvrk(p, filename='tau.npy', movement_type='all')
