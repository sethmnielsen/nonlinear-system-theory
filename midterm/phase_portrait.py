import numpy as np
from numpy import ndarray
import numpy.linalg as npl
from scipy import integrate

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
# from mpl_toolkits.mplot3d import Axes3D
from matplotlib.figure import Figure
import seaborn as sns

from typing import Dict

class PhasePlotter:
    
    def __init__(self, eqpts, calc_xdot: callable, props:Dict=None):
        self.set_properties(props)
        proj = None
        if self.NDIM == 3:
            proj = '3d'
        self.fig1, self.ax1 = plt.subplots(subplot_kw=dict(projection=proj))
        
        self.ax1: Axes
        
        self.eqpts = np.array(eqpts)
        self.calc_xdot = calc_xdot
        
    def states_meshgrid(self):
        w = self.PLOT_WIDTH 
        a = self.PLOT_OFFSET 
        m = self.MGRID_SIZE 
        x_arr = np.vstack([np.linspace(-abs(w+a), w+a, m) for i in range(self.NDIM)])
        self.xmesh = np.meshgrid(*x_arr)
        
        self.xdot = self.calc_xdot(self.xmesh)
        self.xdot = np.array(self.xdot)
        
        # self.x1, self.x2 = self.xmesh
        # self.x1d, self.x2d = self.xdot
        
    def plot_axes(self):
        w = self.PLOT_WIDTH
        
        self.ax1.set_xlim(-w,w)
        self.ax1.set_ylim(-w,w)
        if self.NDIM == 3: 
            self.ax1.set_zlim(-w,w)
        else:
            self.ax1.plot([-w,w], [0,0], color='0', linewidth='0.7')
            self.ax1.plot([0,0], [-w,w], color='0', linewidth='0.7')
            self.ax1.set_aspect('equal')
        
    
    def plot_phases(self, params:Dict=None):
        self.states_meshgrid()
        self.plot_axes()        

        # x1d_ulen = self.x1d / np.sqrt(self.x1d**2 + self.x2d**2) 
        # x2d_ulen = self.x2d / np.sqrt(self.x1d**2 + self.x2d**2)
        xdot_ulen = np.zeros(self.xdot.shape) 
        norm2 = np.sqrt(np.sum(self.xdot**2, axis=0))
        for i, xd in enumerate(self.xdot):
            xdot_ulen[i] = xd / norm2 
            
        # x1d_ulen = self.x1d / np.sqrt(self.x1d**2 + self.x2d**2) 
        # x2d_ulen = self.x2d / np.sqrt(self.x1d**2 + self.x2d**2)
        # M = np.hypot(self.x1d, self.x2d)
        
        if self.NDIM == 2:
            params_ = {
                'units': 'xy',
                'angles': 'xy',
                'scale': 400.0/self.MGRID_SIZE,
                'headwidth': 4.0,
                'headlength': 4.0,
                'headaxislength': 4.0,
                'width': 0.004,
                'cmap': None,
                'alpha': 0.9
            }
        else:
            params_ = {
                'length': 0.2,
                'arrow_length_ratio': 0.3,
                'normalize': False,
                'linewidths': None,
                'cmap': None,
                'norm': xdot_ulen
            }
        if params is not None:
            params_.update(params)
            cmap = params_['cmap'] 
            if cmap is not None and cmap.casefold() in map(str.casefold, matplotlib.cm.cmap_d.keys()):
                for key in matplotlib.cm.cmap_d.keys():
                    cmap = key if cmap.casefold() == key.casefold() else cmap
            params_['cmap'] = cmap
        # Q = self.ax1.quiver(self.x1, self.x2, self.xd1, norm2, **params_)
        if self.NDIM == 2:
            Q = self.ax1.quiver(*self.xmesh, *xdot_ulen, norm2, **params_)
        else:
            Q = self.ax1.quiver(*self.xmesh, *self.xdot, **params_)
        

    def plot_trajectories(self, n_traj:int=10, n_tsteps=400, tf=10, params=None):
        rng: np.random.Generator = np.random.default_rng(self.RNG_SEED)
        x0 = rng.uniform(-self.PLOT_WIDTH, self.PLOT_WIDTH, size=(n_traj,2))
        
        t = np.linspace(0, tf, n_tsteps)
        
        traj = np.array([integrate.odeint(self.calc_xdot, x0i, t).T for x0i in x0])
        
        params_ = {'color': 'blue',
                   'marker': 'o',
                   'linestyle': 'dashed',
                   'linewidth': 0.9,
                   'markersize': 3.0,
                   'markevery': n_tsteps}
        if params is not None and len(params) != 0:
            params_.update(params)
            
        for trj in traj:
            self.ax1.plot(trj[0], trj[1], **params_)
        
    
    def plot_level_curves(self, calc_V, calc_Vdot):
        mz = 300 
        w = self.PLOT_WIDTH
        z_arr = np.linspace(-w, w, mz)
        z1, z2 = np.meshgrid(z_arr, z_arr)
        
        z = np.stack((z1, z2))
        Vz = calc_V(z1, z2)
        Vzdot = calc_Vdot(z1, z2)
        
        c = np.min(Vz[Vzdot>0])
        lvls = [0,c]
        Vdot_params = {'levels': lvls,
                       'alpha': 0.9,
                       'cmap': matplotlib.cm.get_cmap('viridis')}
                    #    'cmap': matplotlib.cm.get_cmap('bone_r')}
        for pt in self.eqpts:
            CS = self.ax1.contour(z1+pt[0], z2+pt[1], Vz, **Vdot_params)
            
        self.ax1.clabel(CS, inline=1, fontsize=10)
    
    def set_properties(self, props:Dict=None):
        props_ = {
            'ndim': 2,
            'figsize': [10,10],
            'style': None, 
            'width': 4,
            'offset': 0,
            'mgrid_sz': 42,
            'seed': None,
            'rc': None
        }
        
        if props is not None:
            for key in props.keys():
                if not key in props_.keys():
                    raise KeyError('Not a valid property')
            props_.update(props)
            
        sns.set_style(props_['style'], props_['rc'])
        plt.rcParams['figure.figsize'] = props_['figsize'] 
        self.PLOT_WIDTH = props_['width']
        self.PLOT_OFFSET = props_['offset']
        self.MGRID_SIZE = props_['mgrid_sz']
        
        self.RNG_SEED = props_['seed']
        
        self.NDIM = props_['ndim']
        
    def draw_plots(self):
        plt.show()