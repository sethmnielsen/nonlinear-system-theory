import numpy as np
import sys

sys.path.append("..")
from phase_portrait import PhasePlotter

def calc_xdot(x, t0=0):
    x1 = x[0]
    x2 = x[1]
    x3 = x[2]
   
    rho = 2
    b = 1.5
    c1 = 1
    c2 = 2
    c3 = 3
    Ef = 7
    I = 12
    tau = 0.6
    
    x1d = x2 
    x2d = (1/I) * ( rho -b*x2 - c1*x3*np.sin(x1) )
    x3d = (1/tau) * ( -c2*x3 + c3*np.cos(x1) + Ef )
    return [x1d, x2d, x3d] 
    
props = {
    'ndim': 3,
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

