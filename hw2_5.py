import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
sns.set_style('white')


fig1, ax1 = plt.subplots()
fig2, ax2 = plt.subplots()
fig3, ax3 = plt.subplots()

x1 = np.arange(-2, 2, 0.1)
x2 = np.arange(-2, 2, 0.1)
n = len(x1)
X1, X2 = np.meshgrid(x1, x2)

u1 = 0
u2 = 0.9*X1-3.2*X2
u3 = np.clip(0.9*X1-3.2*X2,-1.0, 1.0)

x1d = X2
x2d = -0.5*X1 + 1.5*X2 + 0.5*u1

ax1.quiver(X1, X2, x1d, x2d, scale=17)
ax1.set_aspect('equal')

x2d = -0.5*X1 + 1.5*X2 + 0.5*u2

ax2.quiver(X1, X2, x1d, x2d, scale=17)
ax2.set_aspect('equal')

x2d = -0.5*X1 + 1.5*X2 + 0.5*u3

ax3.quiver(X1, X2, x1d, x2d, scale=17)
ax3.set_aspect('equal')

plt.show()