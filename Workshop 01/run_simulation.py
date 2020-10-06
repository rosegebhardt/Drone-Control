#----------IMPORT PACKAGES

import numpy as np
import workshop01 as ws1
import integrators as intg
import matplotlib.pyplot as plt

#----------INITIAL INPUTS

# Initial position
x_1 = np.zeros((3,1))
# Initial velocity
x_2 = np.zeros((3,1))
# Initial pose
# note --> q = cos(th/2) + u*sin(th/2), u = axis, th = angle
x_3 = np.array([[1],[0],[0],[0]])
# Initial ang. vel.
x_4 = np.zeros((3,1))
# Full state vector
x = np.concatenate([x_1,x_2,x_3,x_4])

# Input force
# First part
u1 = np.zeros((6,1))
u1[0] = 2
u1[1] = 1
u1[2] = 0
u1[3] = 1
u1[4] = 0
u1[5] = 0
# Turn off input
u2 = np.zeros((6,1))

#----------INTEGRATION

model = ws1.RigidBody()

t = 0; 
dt = 0.001; n = 1000

integrator = intg.RK4(dt, model.f)

t_history = [0]
x_history = [x]

# While force is applied
for i in range(n):
    x = integrator.step(t, x, u1)
    t = (i+1) * dt

    t_history.append(t)
    x_history.append(x)

x2 = np.array(x_history)
x2 = x2[-1,:]

# While no force is applied
for i in range(n,2*n):
    x = integrator.step(t_history[-1], x2, u2)
    t = (i+1) * dt

    t_history.append(t)
    x_history.append(x)
    
# Convert to numpy arrays, fix dimensions    
x_his = np.array(x_history)
x_his = x_his.reshape((x_his.shape[0],x_his.shape[1]))
t_his = np.array(t_history)

#----------PLOTS

# Linear Velocities
fig1 = plt.figure(figsize=(12,9))
plt.plot(t_his, x_his[:,3], t_his, x_his[:,4], t_his, x_his[:,5],linewidth=3)
plt.legend(['X-Component','Y-Component','Z-Component'],fontsize=14)
plt.xlabel('Time',fontsize=14)
plt.ylabel('Linear Velocity',fontsize=14)
plt.grid(which='both')
plt.title('Velocities Due to Input Forces and Moments',fontsize=18)
plt.show()

# Angular Velocities
fig2 = plt.figure(figsize=(12,9))
plt.plot(t_his, x_his[:,10], t_his, x_his[:,11], t_his, x_his[:,12],linewidth=3)
plt.legend(['X-Component','Y-Component','Z-Component'],fontsize=14)
plt.xlabel('Time',fontsize=14)
plt.ylabel('Angular Velocity',fontsize=14)
plt.grid(which='both')
plt.title('Angular Velocity Due to Input Forces and Moments',fontsize=18)
plt.show()
