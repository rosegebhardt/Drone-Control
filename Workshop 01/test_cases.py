import numpy as np
import workshop01 as ws1
import integrators as integs

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
u = np.zeros((6,1))
u[3] = 1


model = ws1.RigidBody()
print(model.f(0,x,u))