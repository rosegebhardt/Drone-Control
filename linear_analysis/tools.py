import numpy as np
def get_euler_angles_from_rot(R):
    """Compute Euler angles from rotation matrix.
    yaw, pitch, roll: 3, 2, 1 rot sequence
    Note frame relationship: e^b = e^v R^{vb}
    """
    psi = np.arctan2(R[1, 0], R[0, 0]) # yaw angle
    theta = np.arcsin(-R[2, 0])        # pitch angle
    phi = np.arctan2(R[2, 1], R[2, 2]) # roll angle
    return (psi, theta, phi)
    
 
def skew(a):
    """Returns skew symmetric matrix, given a 3-vector"""
    a = a.flatten() # convert to 3-array
    return np.array([
        [    0, -a[2],  a[1]],
        [ a[2],     0, -a[0]],
        [-a[1],  a[0],     0]
        ])   

def rot_from_quat(q):
    """Compute rotation matrix from quaternion.
    quaternion must be provided in form [q0, q]
    """    
    q = q.flatten()
    q0 = q[0]
    q = q[1:]
    return (q0**2 - np.dot(q, q))*np.eye(3) + 2*np.outer(q,q) + 2*q0*skew(q)
 
def quat_prod(p, q):
    p0 = p[0]; p = p[1:4]
    P = np.zeros((4,4))
    P[0, 0] = p0; P[0, 1:] = -p.T
    P[1:, [0]] = p
    P[1:, 1:] = skew(p) + p0*np.eye(3)
    return P @ q 

def quat_from_ypr(y, p, r):
    psi2 = y/2
    theta2 = p/2
    phi2 = r/2
    return np.array([
        [np.sin(phi2)*np.sin(psi2)*np.sin(theta2) + np.cos(phi2)*np.cos(psi2)*np.cos(theta2)], 
        [np.sin(phi2)*np.cos(psi2)*np.cos(theta2) - np.sin(psi2)*np.sin(theta2)*np.cos(phi2)], 
        [np.sin(phi2)*np.sin(psi2)*np.cos(theta2) + np.sin(theta2)*np.cos(phi2)*np.cos(psi2)], 
        [-np.sin(phi2)*np.sin(theta2)*np.cos(psi2) + np.sin(psi2)*np.cos(phi2)*np.cos(theta2)]
        ])

def ypr_from_quat(q):
    rot = rot_from_quat(q)
    (psi, theta, phi) = get_euler_angles_from_rot(rot)
    return (psi, theta, phi) 