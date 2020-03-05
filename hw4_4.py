import numpy as np
import numpy.linalg as npl
from scipy import integrate

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
import seaborn as sns
sns.set_style('darkgrid')

def x_dot(x, t0):
    x1d = -(x[0] + x[0]**3) + 2*x[1]
    x2d = 2*x[0] - (x[1] + x[1]**3)
    return [x1d, x2d] 
    
def V(x):
    P = np.array([[1/6, 1/12], [1/12, 1/6]])
    return x @ P @ x
    
def V_dot()

cmap = 'Oranges_r'
w = 3
a = 0
m = 42
x_arr = np.vstack(( np.linspace(-abs(w+a), w+a, m), np.linspace(-abs(w+a), w+a, m) ))
# x_arr = x_arr[np.nonzero(x_arr)[0]]
x1, x2 = np.meshgrid(x_arr[0], x_arr[1])

x1d = -(x1 + x1**3) + 2*x2
x2d = 2*x1 - (x2 + x2**3)

n = 400
t = np.linspace(0, 10, n)

rng: np.random.Generator = np.random.default_rng(5)
x0 = rng.uniform(-3.0, 3.0, size=(10,2))
# x0 = np.array([[1.2,-1.8],
#                [2.3,1.7],
#                [-0.3, -2.3]])

traj = np.array([integrate.odeint(x_dot, x0i, t).T for x0i in x0])

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

trj_params = {'linewidth': 0.9,
              'markersize': 2.0,
              'markevery': n}
for trj in traj:
    ax1.plot(trj[0], trj[1], '-o', **trj_params)
    
# qk = ax1.quiverkey(Q, X=0, Y=0, U=1, label=r'$1 \frac{m}{s}$', labelpos='E',
                #    coordinates='figure')
ax1.plot([-w,w], [0,0], color='0', linewidth='0.7')
ax1.plot([0,0], [-w,w], color='0', linewidth='0.7')
ax1.set_aspect('equal')
ax1.set_xlim(-w,w)
ax1.set_ylim(-w,w)
# ax1.set_title("pivot='tip'; scales with x view")  

plt.show()