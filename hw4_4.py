import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
import numpy as np
# import sys
# from sage.all import *

# if len(sys.argv) != 2:
#     print("Usage: %s <n>" % sys.argv[0])
#     print("Outputs the prime factorization of n.")
#     sys.exit(1)

# print(factor(sage_eval(sys.argv[1])))

# x1, x2 = var('x1','x2')
# plot_vector_field(( x1*cos(x2),-x2*cos(x1) ), (x1,-pi,pi), (x2,-pi,pi))


# sys.exit(0)

w = 5
m = 20
X1 = np.linspace(-w, w, m)
X2 = np.linspace(-w, w, m)
n = len(X1)
x1, x2 = np.meshgrid(X1, X2)

x1d = -(x1 + x1**3) + 2*x2
x2d = 2*x1 - (x2 + x2**3)

# ax1.quiver(x1, x2, x1d, x2d, scale=0.3)

# ax1.set_aspect('equal')

# plt.show()

fig1, ax1 = plt.subplots()
ax1: Axes
ax1.set_title("pivot='tip'; scales with x view")  
M = np.hypot(x1d, x2d)
Q = ax1.quiver(x1, x2, x1d, x2d, M, units='dots', pivot='mid', width=0.22,
               scale=0.90)
qk = ax1.quiverkey(Q, X=0, Y=0, U=1, label=r'$1 \frac{m}{s}$', labelpos='E',
                   coordinates='figure')
# ax1.scatter(x1, x2, color='0.5', s=1)

plt.show()