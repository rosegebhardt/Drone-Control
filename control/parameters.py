# DC motor 
K = 4.9 # rad/s/V 
tau = 0.085 # 1/s
umax = 12 # V

# Simulation
x0 = 0 # rad/s
Ts = 1e-4 # s
nsteps = 100000

# PID controller
kp = 10
ki = 40
kd = 0.1
sigma = 0.1
