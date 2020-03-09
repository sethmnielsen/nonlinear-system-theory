import numpy as np
import sys

sys.path.append("..")
from phase_portrait import PhasePlotter

def calc_xdot(x, t0=0, mu=0.5):
    x1 = x[0]
    x2 = x[1]
   
    x1d = x2 
    x2d = -x1 + x2*mu - x2*x1**2 - x2**3
    return [x1d, x2d] 
    
ph = PhasePlotter([0,0], calc_xdot)

props = {'width': 4,
         'mgrid_sz': 42,
         'figsize': [10,10],
         'style': 'darkgrid',
         'rc': {'axes.facecolor': '#343436',
                'axes.black'}}
ph.set_properties(props)

ph.plot_phases()
ph.plot_trajectories()

ph.draw_plots()

