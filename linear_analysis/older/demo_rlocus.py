import numpy as np
import matplotlib.pyplot as plt
from control.matlab import * # MATLAB-like control toolbox functionality

plt.close('all')

# Plant
s = tf('s')
P = 1/(s**2+1)

# 1
G = P
plt.figure(1)
rlocus(G)

# 2
G = (1 + s) * P
plt.figure(2)
rlocus(G)

# 3
G = (1/(0.1*s + 1)) * P
plt.figure(3)
rlocus(G)