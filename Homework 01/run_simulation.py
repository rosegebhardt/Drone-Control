import numpy as np
import matplotlib.pyplot as plt
import integrators as intg

m = 1
b = 0.25
k = 1

def f(t, x, u):
    return np.array([x[1], (-b/m)*x[1]-(k/m)*x[0]])

t = 0; x = np.array([0, 1]); u = 0
dt = 1; n = 100

integrator = intg.RK4(dt, f)


t_history = [0]
x_history = [x]

for i in range(n):
    x = integrator.step(t, x, u)
    t = (i+1) * dt

    t_history.append(t)
    x_history.append(x)

intg.__doc__
fig = plt.figure(figsize=(12,9))
plt.plot(t_history, x_history)
plt.legend(['Position','Velocity'])
plt.xlabel('Time')
plt.ylabel('Magnitude')
plt.title('Position and Velocity: Euler')
plt.show()

t_analytic = np.array(t_history)
x_his = np.array(x_history)
x_analytic = np.exp(-t_analytic/8)*(8/(3*np.sqrt(7)))*np.sin(3*np.sqrt(7)*t_analytic/8)

fig2 = plt.figure(figsize=(12,9))
plt.plot(t_history, x_his[:,0])
plt.plot(t_history, x_analytic,'--')
plt.legend(['Approximate Position (Runge Kutta)','True Position'])
plt.xlabel('Time')
plt.ylabel('Magnitude')
plt.title('Analytic Solution Compared to Runge Kutta Solution')
plt.show()



