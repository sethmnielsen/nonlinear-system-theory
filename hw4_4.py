import numpy as np
import numpy.linalg as npl
from scipy import integrate

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
import seaborn as sns
sns.set_style('darkgrid')

def calc_x_dot(x, t0):
    x1d = -(x[0] + x[0]**3) + 2*x[1]
    x2d = 2*x[0] - (x[1] + x[1]**3)
    return [x1d, x2d] 
    
def calc_zdot(z):
    z1dot = -z[0]**3 - 3*z[0]**2 - 4*z[0] + 2*z[1]
    z2dot = 2*z[0] - 2*z[1]**3 - 3*z[1]**2 - 4*z[1]
    return np.array([z1dot, z2dot])
    
def calc_V(x1, x2):
    # P = np.array([[1/6, 1/12], [1/12, 1/6]])
    return (1/6)*(x1**2 + x1*x2 + x2**2)
    
def calc_Vdot(z1, z2):
    # Vzdot = (1/6)*(2*z[0]*zdot[0] + zdot[0]*z[1] + z[0]*zdot[1] + 2*z[1]*zdot[1])
    Vdot = -(z1**2 + z2**2) - (1/6)*(2*z1 + z2)*(3+z1)*z1**2 - (1/6)*(z1+2*z2)*(3+z2)*z2**2
    return Vdot
    

cmap = 'Oranges_r'
w = 4
a = 0 # offset
m = 42
x_arr = np.vstack(( np.linspace(-abs(w+a), w+a, m), np.linspace(-abs(w+a), w+a, m) ))
# x_arr = x_arr[np.nonzero(x_arr)[0]]
x1, x2 = np.meshgrid(x_arr[0], x_arr[1])

x1d = -(x1 + x1**3) + 2*x2
x2d = 2*x1 - (x2 + x2**3)

n = 400
t = np.linspace(0, 10, n)

a = 0 # offset
mz = 80 
z_arr = np.linspace(-w, w, mz)
z1, z2 = np.meshgrid(z_arr, z_arr)

z = np.stack((z1,z2))
# zdot = calc_zdot(z)
Vz = calc_V(z1, z2)
Vzdot = calc_Vdot(z1, z2)
# Vzdot = Vzdot

rng: np.random.Generator = np.random.default_rng(5)
x0 = rng.uniform(-3.0, 3.0, size=(10,2))

traj = np.array([integrate.odeint(calc_x_dot, x0i, t).T for x0i in x0])

fig1, ax1 = plt.subplots()
ax1: Axes
x1d_ulen = x1d / np.sqrt(x1d**2 + x2d**2) 
x2d_ulen = x2d / np.sqrt(x1d**2 + x2d**2)
M = np.hypot(x1d, x2d)

params = {'units': 'xy',
          'angles': 'xy',
          'scale': 400.0/m,
          'scale_units': 'xy',
          'headwidth': 4.0,
          'headlength': 4.0,
          'headaxislength': 4.0,
          'width': 0.004,
          'cmap': matplotlib.cm.get_cmap(cmap),
          'alpha': 0.9}
Q = ax1.quiver(x1, x2, x1d_ulen, x2d_ulen, M, **params)

trj_params = {'color': 'blue',
              'marker': 'o',
              'linestyle': 'dashed',
              'linewidth': 0.9,
              'markersize': 3.0,
              'markevery': n}
for trj in traj:
    ax1.plot(trj[0], trj[1], **trj_params)

# lvls = range(int(np.min(Vzdot)), int(np.max(Vzdot)), 2)
ii = Vzdot>0
c = np.min(Vz[Vzdot>0])
# lvls = np.arange(-2,2, 0.2)
lvls = [0,c]
Vdot_params = {'levels': lvls,
               'alpha': 0.9,
               'cmap': matplotlib.cm.get_cmap('bone_r')}
CS = ax1.contour(z1-1, z2-1, Vz, **Vdot_params)
CS = ax1.contour(z1+1, z2+1, Vz, **Vdot_params)
ax1.clabel(CS, inline=1, fontsize=10)
    
    
# qk = ax1.quiverkey(Q, X=0, Y=0, U=1, label=r'$1 \frac{m}{s}$', labelpos='E',
                #    coordinates='figure')
ax1.plot([-w,w], [0,0], color='0', linewidth='0.7')
ax1.plot([0,0], [-w,w], color='0', linewidth='0.7')
ax1.set_aspect('equal')
ax1.set_xlim(-w,w)
ax1.set_ylim(-w,w)
# ax1.set_title("pivot='tip'; scales with x view")  

plt.show()