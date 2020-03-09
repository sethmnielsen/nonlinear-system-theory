import numpy as np
import sys

sys.path.append("..")
from midterm.phase_portrait import PhasePlotter

def calc_xdot(x, t0, mu):
    mu = 5
    x1 = x[0]
    x2 = x[1]
    
    x1d = x2 
    x2d = -x1 + x2*mu - x2*x1**2 - x2**3
    return [x1d, x2d] 
    
ph = PhasePlotter([0,0], calc_xdot)
ph.set_properties(width=3)
ph.plot_phases()

ph.draw_plots()

