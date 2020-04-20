import numpy as np

import matplotlib.pyplot as plt
import seaborn as sns
sns.set_style('darkgrid')

from spacecraft_sim_interface import SpacecraftSim
from rotations import Euler2Quaternion, Quaternion2Euler, Rotation2Euler

class spacecraft_sim_py():
    
    def __init__(self):
        dt = 0.001
        sig0 = np.array([0.6, -0.4, 0.2])
        omg0 = np.array([0.7, 0.2, -0.15])
        # omg0 = np.array([0.9, 0., 0.])
        P = np.array([18.67, 2.67, 10.67])
        K = 7.11
        x0 = np.zeros(16)
        x0[3:6] = sig0
        x0[9:12] = omg0
        self.sim = SpacecraftSim(x0, dt)
        
        self.t_end = 15.0  # sec
        self.n = int(self.t_end // dt)
        
        self.x = np.zeros(16)
        self.x_hist = np.zeros((16, self.n))
        
    def run(self):
        for i in range(self.n):
            self.sim.run()
            self.sim.getState(self.x)
            self.x_hist[:,i] = np.copy(self.x) 
        
        x = self.x_hist
        sig = x[3:6]
        omg = x[9:12]
        
        t = np.linspace(0, self.t_end, self.n)
        fig, ax = plt.subplots(2,1, sharex=True)
        ax[0].set_title('States')
        ax[0].plot(t, sig[0], label=r'$\sigma_1$')
        ax[0].plot(t, sig[1], label=r'$\sigma_2$')
        ax[0].plot(t, sig[2], label=r'$\sigma_3$')
        ax[0].set_ylabel(r'$\sigma$')
        ax[1].plot(t, omg[0], label=r'$\omega_1$')
        ax[1].plot(t, omg[1], label=r'$\omega_2$')
        ax[1].plot(t, omg[2], label=r'$\omega_3$')
        ax[1].set_ylabel(r'$\omega$ (rad/s)')
        ax[1].set_xlabel('Time (s)')
        ax[0].legend()
        ax[1].legend()
        
        plt.show()
        

if __name__ == '__main__':
    ss = spacecraft_sim_py()
    ss.run()
