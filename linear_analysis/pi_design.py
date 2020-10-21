import numpy as np
import matplotlib.pyplot as plt
from control.matlab import * # MATLAB-like control toolbox functionality


K = 4.9
tau = 0.085
zeta = 0.7
umax = 12
emax = K*umax/2
kp = umax/emax

plt.close('all')

print('kp= ', kp)
wn = (1 + K*kp)/(2*tau*zeta)
print('wn= ', wn)
ki = wn**2 * tau / K
print('ki =', ki)

s = tf('s')
Gyr = (K/tau)*(ki + kp*s)/(s**2 + ((1 + K*kp)/tau)*s + K*ki/tau)

# Step response for the system
yout, T = step(Gyr)
plt.figure()
plt.plot(T.T, yout.T)
plt.show()