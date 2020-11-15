import numpy as np
import parameters as P
import matplotlib.pyplot as plt
from integrators import get_integrator
from pid import PIDControl

def model_DCMotor(t,x,u):
    return (P.K*u/P.tau) - (x/P.tau)

def model_Seesaw(t,x,u):
    return x 

class Controller:
    def __init__(self):
        self.ctrl = PIDControl(P.kp, P.ki, P.kd, P.umax, P.sigma, P.Ts)
        
    def update(self, r, y):
        return self.ctrl.PID(r,y)
    
class System:
    def __init__(self):
        self.integ = get_integrator(P.Ts,model_DCMotor)
        
    def update(self, y, u):
        return self.integ.step(t,y,u)
    
    def update_velocity(self, y, u):
        return self.integ.step(t,y,u)
   
    def update_angle(self, y, u):
        step1 = self.integ.step(t,y,u)
        return self.integ.step(t,step1,u)
        
# Init system and feedback controller
system = System()
controller = Controller()

# Simulate step response
t_history = [0]
angle_history = [0]
velocity_history = [0]
#u_history = [0]

r = 1
angle = 0
velocity = 0
t = 0

for i in range(P.nsteps):
    u_angle = controller.update(r, angle)
    u_velocity = controller.update(r, velocity)
    angle = system.update_angle(velocity, u_angle)
    velocity = system.update_velocity(velocity, u_velocity)
    t += P.Ts

    t_history.append(t)
    angle_history.append(angle)
    velocity_history.append(velocity)
    #u_history.append(u)

# Plot response y due to step change in r

fig1 = plt.figure(figsize=(12,9))
plt.plot(t_history, angle_history, linewidth=2.5)
plt.grid(which='both',axis='both')
#plt.plot(t_history, velocity_history, '--')
#plt.legend(['Angular Position','Angular Velocity'])
plt.xlabel('Time (sec)', fontsize=16)
plt.ylabel('Angular Position (rad)', fontsize=16)
plt.title('Angular Position Response to Step Input', fontsize=20)
plt.show()

fig2 = plt.figure(figsize=(12,9))
plt.plot(t_history, velocity_history, linewidth=2.5)
plt.grid(which='both',axis='both')
#plt.plot(t_history, velocity_history, '--')
#plt.legend(['Angular Position','Angular Velocity'])
plt.xlabel('Time (sec)', fontsize=16)
plt.ylabel('Angular Velocity (rad/s)', fontsize=16)
plt.title('Angular Velocity Response to Step Input', fontsize=20)
plt.show()
