import numpy as np
import matplotlib.pyplot as plt
from control.matlab import * # MATLAB-like control toolbox functionality

plt.close('all')

tau = 1
K = 1

# System matrices
A = -1/tau
B = K / tau
C = 1
D = 0
sys = ss(A, B, C, D)
P = tf(sys)
print(P)

# Step response for the system
plt.figure(1)
yout, T = step(sys)
plt.plot(T.T, yout.T)


plt.figure(2)
rlocus(sys)

# Bode plot for the system
plt.figure(2)
mag, phase, om = bode(sys, logspace(-2, 2), Plot=True)

G = tf(sys)
print(G)
