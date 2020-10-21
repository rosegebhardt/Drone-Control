import numpy as np

x_trim = np.array([[-0.000000, -0.000000, -100.000000, 24.971443, 0.000000, 1.194576, 0.993827, 0.000000, 0.110938, 0.000000, 0.000000, 0.000000, 0.000000]]).T
u_trim = np.array([[-0.118662, 0.009775, -0.001611, 0.857721]]).T
Va_trim = 25.000000
alpha_trim = 0.047801
theta_trim = 0.222334
a_phi1 = 22.628851
a_phi2 = 130.883678
a_theta1 = 5.294738
a_theta2 = 99.947422
a_theta3 = -36.112390
a_V1 = 0.052559
a_V2 = 10.698657
a_V3 = 9.651117
A_lon = np.array([
    [-0.220972, 0.483662, -1.165980, -9.547814, -0.000000],
    [-0.552148, -4.463072, 24.373686, -2.208720, -0.000000],
    [0.191108, -3.993407, -5.294738, 0.000000, -0.000000],
    [0.000000, 0.000000, 0.999878, 0.000000, -0.000000],
    [0.220506, -0.975386, -0.000000, 24.598080, 0.000000]])
B_lon = np.array([
    [-0.144115, 10.698657],
    [-2.585871, 0.000000],
    [-36.112390, 0.000000],
    [0.000000, 0.000000],
    [-0.000000, -0.000000]])
A_lat = np.array([
    [-0.776773, 1.194576, -24.971443, 9.558619, 0.000000],
    [-3.866705, -22.628851, 10.905041, 0.000000, 0.000000],
    [0.783078, -0.115092, -1.227655, 0.000000, 0.000000],
    [0.000000, 1.000000, 0.226071, 0.000000, 0.000000],
    [0.000000, -0.000000, 1.025235, 0.000000, 0.000000]])
B_lat = np.array([
    [1.486172, 3.764969],
    [130.883678, -1.796374],
    [5.011735, -24.881341],
    [0.000000, 0.000000],
    [0.000000, 0.000000]])
Ts = 0.020000

[lam_alon,x_alon] = np.linalg.eig(A_lon)
#[lam_blon,x_blon] = np.linalg.eig(B_lon)
[lam_alat,x_alat] = np.linalg.eig(A_lat)
#[lam_blat,x_blat] = np.linalg.eig(B_lat)

import matplotlib.pyplot as plt

fig1 = plt.figure(figsize=(12,9))
X_alon = [x.real for x in lam_alon]
Y_alon = [x.imag for x in lam_alon]
plt.scatter(X_alon,Y_alon, marker='x',s=100)
#plt.xline(0)
plt.show()

fig2 = plt.figure(figsize=(12,9))
X_alat = [x.real for x in lam_alat]
Y_alat = [x.imag for x in lam_alat]
plt.scatter(X_alat,Y_alat,marker='x',s=100)
#plt.xline(0)
plt.show()