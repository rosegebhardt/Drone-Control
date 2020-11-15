import numpy as np
from mav_dynamics import mavDynamics
import integrators as intg
import matplotlib.pyplot as plt
import aerosonde_parameters as MAV
import mavsim_python_chap5_model_coef as ch5

x = ch5.x_trim.flatten()
wind = np.array([[0], [0], [0], [0], [0], [0]])
delta = ch5.u_trim.flatten()

#establish aircraft for simulation
drone = mavDynamics(ch5.Ts)
u1 = drone._forces_moments(delta) #Aerodynamic enviornment
#u2 = Aerodynamics.forces_moments(drone, np.array([0, 0, 0, 0]), MAV)

#----------INTEGRATION
t = 0; 
dt = 0.0001; n = 500

for i in range(n): 
    drone.update(delta,wind)
    
# =============================================================================
# # Convert to numpy arrays, fix dimensions    
# x_his = np.array(x_history)
# x_his = x_his.reshape((x_his.shape[0],x_his.shape[1]))
# t_his = np.array(t_history)
# 
# #----------PLOTS
# 
# # Positions
# fig1 = plt.figure(figsize=(12,9))
# plt.plot(t_his, x_his[:,0], t_his, x_his[:,1], t_his, x_his[:,2],linewidth=3)
# plt.legend(['X-Position','Y-Position','Z-Position'],fontsize=14)
# plt.xlabel('Time',fontsize=14)
# plt.ylabel('Positions',fontsize=14)
# plt.grid(which='both')
# plt.title('Position to Input Forces and Moments',fontsize=18)
# plt.show()
# 
# # Linear Velocities
# fig2 = plt.figure(figsize=(12,9))
# plt.plot(t_his, x_his[:,3], t_his, x_his[:,4], t_his, x_his[:,5],linewidth=3)
# plt.legend(['X-Component','Y-Component','Z-Component'],fontsize=14)
# plt.xlabel('Time',fontsize=14)
# plt.ylabel('Linear Velocity',fontsize=14)
# plt.grid(which='both')
# plt.title('Velocities Due to Input Forces and Moments',fontsize=18)
# plt.show()
# 
# # Angular Velocities
# fig3 = plt.figure(figsize=(12,9))
# plt.plot(t_his, x_his[:,10], t_his, x_his[:,11], t_his, x_his[:,12],linewidth=3)
# plt.legend(['X-Component','Y-Component','Z-Component'],fontsize=14)
# plt.xlabel('Time',fontsize=14)
# plt.ylabel('Angular Velocity',fontsize=14)
# plt.grid(which='both')
# plt.title('Angular Velocity Due to Input Forces and Moments',fontsize=18)
# plt.show()
# =============================================================================
