import numpy as np

m = 1;
J = np.eye(3)
# note --> q = cos(th/2) + u*sin(th/2), u = axis, th = angle

class RigidBody:
    """Setup state-space system for integration of rigid body
    """
    def __init__(self):
        self.m = m # mass, kg
        self.J = J # mass moment of inertia wrt COM in body-fixed frame, m^2 kg
        #self.Jinv = Jinv # inverse of J
        #self.x = x # state

    def f(self, t, x, u):
        """Evaluates f in xdot = f(t, x ,u)"""
        # Get force, moment (torque)
        f_b = u[:3]
        m_b = u[3:]
        # Get position, velocity, quaternion (rotation), angular velocity 
        #r_i = x[:3] # wrt to i-frame
        v_b = x[3:6] # wrt to i-frame
        q_ib = x[6:10] # for rotation b to i-frame
        w_b = x[10:] # wrt to b-frame
        
        # Normalize quat. -> rotation
        q_ib = q_ib/np.linalg.norm(q_ib) # normalize
        R_ib = rot_from_quat(q_ib)
        
        # Compute equations of motion
        # d/dt(r_i) 
        rdot_i = R_ib @ v_b
        # d/dt(v_b)
        vdot_b = (-skew(w_b) @ v_b) + (1/m)*f_b
        # d/dt(q_ib)
        wq_ib = np.zeros((4,1))
        wq_ib[1:] = w_b
        qdot_ib = 0.5 * quat_prod(wq_ib, q_ib)
        # d/dt(w_b)
        wdot_b = np.linalg.pinv(self.J) @ (m_b - skew(w_b) @ self.J @ w_b)
        
        #x_dot = np.concatenate(rdot_i,vdot_b,qdot_ib,wdot_b)
        return np.concatenate([rdot_i,vdot_b,qdot_ib,wdot_b])
        
        
# TODO add separate class for this functionality        
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