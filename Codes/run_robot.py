import numpy as np
#from test_dvrk import test_dvrk
from run_dvrk_pos import test_dvrk


import dvrk

p = dvrk.psm("PSM1")
p.enable()
p.home()

test_dvrk(p)

print('completed!')
