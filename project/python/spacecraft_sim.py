#!/usr/bin/env python3

import numpy as np

import matplotlib.pyplot as plt
import seaborn as sns
sns.set_style('darkgrid')

from spacecraft_sim_interface import SpacecraftSim
from rotations import Euler2Quaternion, Quaternion2Euler, Rotation2Euler

class spacecraft_sim_py():
    def __init__(self):
        
        sig0 = np.array([0.6, -0.4, 0.2])
        omg0 = np.array([0.7, 0.2, -0.15])
        x0 = np.array([sig0, omg0])
        self.sim = SpacecraftSim(x0)

    def run(self):
        n = 100
        for i in range(n):
            state = self.sim.run()

if __name__ == '__main__':
    ss = spacecraft_sim_py()
    while True:
        ss.run()
