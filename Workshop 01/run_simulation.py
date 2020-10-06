#----------IMPORT PACKAGES
import numpy as np
import workshop01 as ws1
import integrators as intg
import matplotlib.pyplot as plt


# initial position
x_1 = np.zeros((3,1))
# initial velocity
x_2 = np.zeros((3,1))
# initial pose
# note --> q = cos(th/2) + u*sin(th/2), u = axis, th = angle
x_3 = np.array([[1],[0],[0],[0]])
# initial ang. vel.
x_4 = np.zeros((3,1))
# full state vector
x = np.concatenate([x_1,x_2,x_3,x_4])

# input force
u1 = np.zeros((6,1))
u1[3] = 1
u1[4] = 0.5
u1[5] = 10
u2 = np.zeros((6,1))

model = ws1.RigidBody()

#def f(t, x, u):
    #return np.array([x[1], (-b/m)*x[1]-(k/m)*x[0]])

t = 0; #x = np.array([0, 1]); u = 0
dt = 0.001; n = 1000

integrator = intg.RK4(dt, model.f)

t_history = [0]
x_history = [x]

for i in range(n):
    x = integrator.step(t, x, u1)
    t = (i+1) * dt

    t_history.append(t)
    x_history.append(x)

x2 = np.array(x_history)
x2 = x2[-1,:]

for i in range(n,2*n):
    x = integrator.step(t_history[-1], x2, u2)
    t = (i+1) * dt

    t_history.append(t)
    x_history.append(x)
    
x_his = np.array(x_history)
x_his = x_his.reshape((x_his.shape[0],x_his.shape[1]))
t_his = np.array(t_history)

fig1 = plt.figure(figsize=(12,9))
plt.plot(t_his, x_his[:,10])
#plt.legend(['Position','Velocity'])
plt.xlabel('Time')
plt.ylabel('??')
plt.title('??')
plt.show()
# =============================================================================
# intg.__doc__
# fig = plt.figure(figsize=(12,9))
# plt.plot(t_history, x_history)
# plt.legend(['Position','Velocity'])
# plt.xlabel('Time')
# plt.ylabel('Magnitude')
# plt.title('Position and Velocity: Euler')
# plt.show()
# 
# t_analytic = np.array(t_history)
# x_his = np.array(x_history)
# x_analytic = np.exp(-t_analytic/8)*(8/(3*np.sqrt(7)))*np.sin(3*np.sqrt(7)*t_analytic/8)
# 
# fig2 = plt.figure(figsize=(12,9))
# plt.plot(t_history, x_his[:,0])
# plt.plot(t_history, x_analytic,'--')
# plt.legend(['Approximate Position (Runge Kutta)','True Position'])
# plt.xlabel('Time')
# plt.ylabel('Magnitude')
# plt.title('Analytic Solution Compared to Runge Kutta Solution')
# plt.show()
# =============================================================================


#print(model.f(0,x,u))