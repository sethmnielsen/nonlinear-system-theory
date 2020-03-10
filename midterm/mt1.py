import numpy as np
import sys

sys.path.append("..")
from phase_portrait import PhasePlotter

def calc_xdot(x, t0=0, mu=-1.4):
    x1 = x[0]
    x2 = x[1]
   
    x1d = x2 
    x2d = -x1 + x2*mu - x2*x1**2 - x2**3
    return [x1d, x2d] 
    
props = {
    'width': 4.0,
    'mgrid_sz': 40,
    'figsize': [10,10],
    'style': None,
    'rc': {'axes.facecolor': '#252538'}
}
ph = PhasePlotter([0,0], calc_xdot, props)

phase_params = {
    # 'units': 'inches',
    # 'scale': 10,
    'cmap': 'ylorrd'
}
ph.plot_phases(params=phase_params)

traj_params = {
    'color': None 
}
ph.plot_trajectories(params=traj_params)

ph.draw_plots()

