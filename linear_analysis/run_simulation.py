import numpy as np
from aircraft_dynamics import *
import integrators as intg
import matplotlib.pyplot as plt
import aerosonde_parameters as MAV

#establish initial state
pn = 0
pe = 0
pd = 0
u = 1
v = 0
w = 0
e0 = 1
e1 = 0
e2 = 0
e3 = 0
p = 0
q = 0
r = 0
x = np.array([pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r])


wind = np.array([[0], [0], [0], [0], [0], [0]])

#establish input
delta = np.array([0, 0, 0, 0])

#establish aircraft for simulation
drone = Aircraft(MAV.J, MAV.mass, x, wind)

u1 = Aerodynamics.forces_moments(drone, delta, MAV) #Aerodynamic enviornment
#u1 = np.zeros((6,1))

#----------INTEGRATION
t = 0; 
dt = 0.001; n = 1000
x = np.reshape(x, (13, 1))

integrator = intg.RK4(dt, drone.f)

t_history = [0]
x_history = [x]

# While force is applied
for i in range(n):
    x = integrator.step(t, x, u1)
    t = (i+1) * dt

    t_history.append(t)
    x_history.append(x)

# While no force is applied
for i in range(n,10*n):
    x = integrator.step(t_history[-1], x, u1)
    t = (i+1) * dt

    t_history.append(t)
    x_history.append(x)
    
# Convert to numpy arrays, fix dimensions    
x_his = np.array(x_history)
x_his = x_his.reshape((x_his.shape[0],x_his.shape[1]))
t_his = np.array(t_history)

#----------PLOTS

# Positions
fig1 = plt.figure(figsize=(12,9))
plt.plot(t_his, x_his[:,0], t_his, x_his[:,1], t_his, x_his[:,2],linewidth=3)
plt.legend(['X-Position','Y-Position','Z-Position'],fontsize=14)
plt.xlabel('Time',fontsize=14)
plt.ylabel('Positions',fontsize=14)
plt.grid(which='both')
plt.title('Position to Input Forces and Moments',fontsize=18)
plt.show()

# Linear Velocities
fig2 = plt.figure(figsize=(12,9))
plt.plot(t_his, x_his[:,3], t_his, x_his[:,4], t_his, x_his[:,5],linewidth=3)
plt.legend(['X-Component','Y-Component','Z-Component'],fontsize=14)
plt.xlabel('Time',fontsize=14)
plt.ylabel('Linear Velocity',fontsize=14)
plt.grid(which='both')
plt.title('Velocities Due to Input Forces and Moments',fontsize=18)
plt.show()

# Angular Velocities
fig3 = plt.figure(figsize=(12,9))
plt.plot(t_his, x_his[:,10], t_his, x_his[:,11], t_his, x_his[:,12],linewidth=3)
plt.legend(['X-Component','Y-Component','Z-Component'],fontsize=14)
plt.xlabel('Time',fontsize=14)
plt.ylabel('Angular Velocity',fontsize=14)
plt.grid(which='both')
plt.title('Angular Velocity Due to Input Forces and Moments',fontsize=18)
plt.show()