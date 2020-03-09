import numpy as np

from midterm.phase_portrait import PhasePlotter

def calc_xdot(x, t0):
    x1d = -(x[0] + x[0]**3) + 2*x[1]
    x2d = 2*x[0] - (x[1] + x[1]**3)
    return [x1d, x2d] 
    
def calc_V(x1, x2):
    P = np.array([[1/6, 1/12], [1/12, 1/6]])
    return (1/6)*(x1**2 + x1*x2 + x2**2)
    # return x1 @ P @ x2
    
def calc_Vdot(z1, z2):
    Vdot = -(z1**2 + z2**2) - (1/6)*(2*z1 + z2)*(3+z1)*z1**2 - (1/6)*(z1+2*z2)*(3+z2)*z2**2
    return Vdot
    

ph = PhasePlotter([[1,1],[-1,-1]], calc_xdot)
ph.set_properties(width=3)
ph.plot_phases()
ph.plot_trajectories()
ph.plot_level_curves(calc_V, calc_Vdot)

ph.draw_plots()

