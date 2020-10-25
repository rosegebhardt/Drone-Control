import numpy as np
from tools import *

class Aircraft:
    def __init__(self, J, m, x, wind):
        self.m = m # mass, kg
        self.J = J # mass moment of inertia wrt COM in body-fixed frame, m^2 kg
        self.x = x # State
        self.Va = np.zeros((3,1))
        self.update_velocity_data(wind)
        self.Va_norm = np.linalg.norm(self.Va)
        self.alpha = 1
        self.beta = 1
    
    def set_state(self, x):
        self.x = x
        
    def update_velocity_data(self, wind=np.zeros((6,1))):
        steady_state = wind[0:3]
        gust = wind[3:6]
        Vw = (rot_from_quat(self.x[6:10]) @ steady_state) + gust #Wind vector
        Vg = self.x[3:6] #Ground speed
        self.Va = Vg - Vw #Air Speed relative to body frame
        self.Va_norm = np.linalg.norm(self.Va) #Norm of Air Speed
        self.alpha = np.arctan2(self.Va[2], self.Va[0]) #Angle of attack 
        self.beta = np.arcsin(self.Va[1] / self.Va_norm) #Slip angle

    def f(self, t, x, u): #x is the state
        """Evaluates f in xdot = f(t, x ,u)"""
        # Get force, moment (torque)
        f_b = u[:3]
        m_b = u[3:]
        # Get position, velocity, quaternion (rotation), angular velocity 
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
        vdot_b = (-skew(w_b) @ v_b) + (1/self.m)*f_b
     
        # d/dt(q_ib)
        wq_ib = np.zeros((4,1))
        wq_ib[1:] = np.reshape(w_b, (3,1))
        qdot_ib = 0.5 * quat_prod(wq_ib, q_ib)
        # d/dt(w_b)
        wdot_b = np.linalg.pinv(self.J) @ (m_b - skew(w_b) @ self.J @ w_b)
     
        
        #x_dot = np.concatenate(rdot_i,vdot_b,qdot_ib,wdot_b)
        return np.concatenate([rdot_i,vdot_b,qdot_ib,wdot_b])
        
class Aerodynamics:
    #Define input forces and moments in body frame 
    def forces_moments(aircraft, delta, aero):
        R_sb = np.array([[np.cos(aircraft.alpha), -1*np.sin(aircraft.alpha)],#2x2 rotation matrix used to get aerodynamic forces into fx and fz (body frame)
                         [np.sin(aircraft.alpha), np.cos(aircraft.alpha)]])
        
        psi, theta, phi = get_euler_angles_from_rot(rot_from_quat(aircraft.x[6:10]))
        p = aircraft.x[10] #roll rate
        q = aircraft.x[11] #pitch rate
        r = aircraft.x[12] #yaw rate
            
        delta_e = delta[0]
        delta_a = delta[1]
        delta_r = delta[2]
        delta_t = delta[3]
            
        F_lift = 0.5 * aero.rho * (aircraft.Va_norm**2) * aero.S_wing * (aero.C_L_0 + aero.C_L_alpha * aircraft.alpha + aero.C_L_q * q * (aero.c / (2 * aircraft.Va_norm)) + aero.C_L_delta_e * delta_e)
        F_drag = 0.5 * aero.rho * (aircraft.Va_norm**2) * aero.S_prop * (aero.C_D_0 + aero.C_D_alpha * aircraft.alpha + aero.C_D_q * q * (aero.c / (2 * aircraft.Va_norm)) + aero.C_D_delta_e * delta_e)
        aero_forces = np.array([[F_drag], 
                                [F_lift]])
        
        #Forces in the body frame
        fx = np.reshape(( R_sb @ (-1 * aero_forces) ), (2, 1))[0]
        fy = 0.5 * aero.rho * (aircraft.Va_norm**2) * aero.S_wing * (aero.C_Y_0 + aero.C_Y_beta * aircraft.beta + aero.C_Y_p * p * (aero.b / (2 * aircraft.Va_norm)) + aero.C_Y_r * r * (aero.b / (2 * aircraft.Va_norm)) + aero.C_Y_delta_a * delta_a + aero.C_Y_delta_r * delta_r)
        fz = np.reshape(( R_sb @ (-1 * aero_forces) ), (2, ))[1]
        Mx = 0.5 * aero.rho * (aircraft.Va_norm**2) * aero.S_wing * aero.b * (aero.C_ell_0 + aero.C_ell_beta * aircraft.beta + aero.C_ell_p * p * (aero.b / (2 * aircraft.Va_norm)) + aero.C_ell_r * r * (aero.b / (2 * aircraft.Va_norm)) + aero.C_ell_delta_a * delta_a + aero.C_ell_delta_r * delta_r)
        My = 0.5 * aero.rho * (aircraft.Va_norm**2) * aero.S_wing * aero.c * (aero.C_m_0 + aero.C_m_alpha * aircraft.alpha + aero.C_m_q * q * (aero.c / (2 * aircraft.Va_norm)) + aero.C_m_delta_e * delta_e)
        Mz = 0.5 * aero.rho * (aircraft.Va_norm**2) * aero.S_wing * aero.b * (aero.C_n_0 + aero.C_n_beta * aircraft.beta + aero.C_n_p * p * (aero.b / (2 * aircraft.Va_norm)) + aero.C_n_r * r * (aero.b / (2 * aircraft.Va_norm)) + aero.C_n_delta_a * delta_a + aero.C_n_delta_r * delta_r)
        
        return np.transpose(np.array([[fx, fy, fz, Mx, My, Mz]]))

     #ask dirk what S to use
     #please ask dirk if Va is a norm or a component

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    