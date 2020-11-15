import sys
sys.path.append('..')
import numpy as np

# load message types
#from message_types.msg_state import msgState

import aerosonde_parameters as MAV
from tools import rot_from_quat, ypr_from_quat, quat_from_ypr
import mavsim_python_chap5_model_coef as ch5

class mavDynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        self._state = np.array([[MAV.pn0],  # (0)
                               [MAV.pe0],   # (1)
                               [MAV.pd0],   # (2)
                               [MAV.u0],    # (3)
                               [MAV.v0],    # (4)
                               [MAV.w0],    # (5)
                               [MAV.e0],    # (6)
                               [MAV.e1],    # (7)
                               [MAV.e2],    # (8)
                               [MAV.e3],    # (9)
                               [MAV.p0],    # (10)
                               [MAV.q0],    # (11)
                               [MAV.r0]])   # (12)
        #self._state = ch5.x_trim
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        self._update_velocity_data()
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = np.array([[0.], [0.], [0.]])
        self.Va_norm = np.linalg.norm(self._Va)
        self._alpha = 1
        self._beta = 1
        # initialize true_state message
        #self.true_state = msgState()
        
    def set_state(self, x):
        self.x = x

    ###################################
    # public functions
    def update(self, delta, wind):
        """
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        """
        print(self._state[6:10])
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step/2.*k1, forces_moments)
        k3 = self._derivatives(self._state + time_step/2.*k2, forces_moments)
        k4 = self._derivatives(self._state + time_step*k3, forces_moments)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6][0] = self._state.item(6)/normE
        self._state[7][0] = self._state.item(7)/normE
        self._state[8][0] = self._state.item(8)/normE
        self._state[9][0] = self._state.item(9)/normE

        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)

        # update the message class for the true state
        self._update_true_state()

    def external_set_state(self, new_state):
        self._state = new_state

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        # pn = state.item(0)
        # pe = state.item(1)
        # pd = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        e0 = state.item(6)
        e1 = state.item(7)
        e2 = state.item(8)
        e3 = state.item(9)
        p = state.item(10)
        q = state.item(11)
        r = state.item(12)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)
        
        (psi,theta,phi) = ypr_from_quat(np.array([[e0],[e1],[e2],[e3]]))
        
        lkin_dot = np.array([[np.cos(theta)*np.cos(phi), np.sin(phi)*np.sin(theta)*np.cos(psi) - np.cos(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.cos(psi) - np.sin(phi)*np.sin(psi)],
                             [np.cos(theta)*np.sin(phi), np.sin(phi)*np.sin(theta)*np.sin(psi) - np.cos(phi)*np.cos(psi), np.cos(phi)*np.sin(theta)*np.sin(psi) - np.sin(phi)*np.cos(psi)],
                             [-np.sin(theta), np.sin(phi)*np.cos(theta), np.cos(phi)*np.cos(theta)]])*np.array([[u],[v],[w]])
    
        ldyn_dot = np.array([r*v - q*w, p*w - r*u, q*u - p*v]) + (1/MAV.mass)*np.array([fx, fy, fz])
        
        rkin_dot_array = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                             [0, np.cos(phi), -np.sin(phi)],
                             [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])*np.array([[p],[q],[r]])
        
        rkin_dot =  quat_from_ypr(rkin_dot_array[0],rkin_dot_array[1],rkin_dot_array[2])
    
        rdyn_dot = np.array([MAV.gamma1*p*q - MAV.gamma2*q*r, 
                            MAV.gamma5*p*r - MAV.gamma6*(p**2 - r**2), 
                            MAV.gamma7*p*q - MAV.gamma1*q*r]) + np.array([MAV.gamma3*l - MAV.gamma4*n, 
                            m/MAV.Jy, 
                            MAV.gamma4*l - MAV.gamma8*n])
        
        # position kinematics
        pn_dot = lkin_dot[0]
        pe_dot = lkin_dot[1]
        pd_dot = lkin_dot[2]

        # position dynamics
        u_dot = ldyn_dot[0]
        v_dot = ldyn_dot[1]
        w_dot = ldyn_dot[2]

        # rotational kinematics
        e0_dot = rkin_dot[0]
        e1_dot = rkin_dot[1]
        e2_dot = rkin_dot[2]
        e3_dot = rkin_dot[3]

        # rotational dynamics
        p_dot = rdyn_dot[0]
        q_dot = rdyn_dot[1]
        r_dot = rdyn_dot[2]

        # collect the derivative of the states
        x_dot = np.array([[pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]]).T
        return x_dot

    def _update_velocity_data(self, wind=np.zeros((6,1))):
        steady_state = wind[0:3]
        gust = wind[3:6]
        Vw = (rot_from_quat(self._state[6:10]) @ steady_state) + gust
        Vg = self._state[3:6]
        self._Va = Vg - Vw
        self.Va_norm = np.linalg.norm(self._Va)
        self._alpha = np.arctan2(self._Va[2], self._Va[0])
        self._beta = np.arcsin(self._Va[1] / self.Va_norm)

    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        phi, theta, psi = ypr_from_quat(self._state[6:10])
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)

        delta_e = delta.item(0)
        delta_a = delta.item(1)
        delta_r = delta.item(2)
        delta_t = delta.item(3)

        F_lift = 0.5 * MAV.rho * (self.Va_norm**2) * MAV.S_wing * (MAV.C_L_0 + MAV.C_L_alpha * self._alpha + MAV.C_L_q * q * (MAV.c / (2 * self.Va_norm)) + MAV.C_L_delta_e * delta_e)
        F_drag = 0.5 * MAV.rho * (self.Va_norm**2) * MAV.S_prop * (MAV.C_D_0 + MAV.C_D_alpha * self._alpha + MAV.C_D_q * q * (MAV.c / (2 * self.Va_norm)) + MAV.C_D_delta_e * delta_e)
        aero_forces = np.array([[F_drag], 
                                [F_lift]])
    
        R_sb = np.array([[np.cos(self._alpha), -1*np.sin(self._alpha)],
                         [np.sin(self._alpha), np.cos(self._alpha)]])
    
        fx = ( R_sb @ (-1 * aero_forces) ).flatten()[0]
        fy = 0.5 * MAV.rho * (self.Va_norm**2) * MAV.S_wing * (MAV.C_Y_0 + MAV.C_Y_beta * self._beta + MAV.C_Y_p * p * (MAV.b / (2 * self.Va_norm)) + MAV.C_Y_r * r * (MAV.b / (2 * self.Va_norm)) + MAV.C_Y_delta_a * delta_a + MAV.C_Y_delta_r * delta_r)
        fz = ( R_sb @ (-1 * aero_forces) ).flatten()[1]
        Mx = 0.5 * MAV.rho * (self.Va_norm**2) * MAV.S_wing * MAV.b * (MAV.C_ell_0 + MAV.C_ell_beta * self._beta + MAV.C_ell_p * p * (MAV.b / (2 * self.Va_norm)) + MAV.C_ell_r * r * (MAV.b / (2 * self.Va_norm)) + MAV.C_ell_delta_a * delta_a + MAV.C_ell_delta_r * delta_r)
        My = 0.5 * MAV.rho * (self.Va_norm**2) * MAV.S_wing * MAV.c * (MAV.C_m_0 + MAV.C_m_alpha * self._alpha + MAV.C_m_q * q * (MAV.c / (2 * self.Va_norm)) + MAV.C_m_delta_e * delta_e)
        Mz = 0.5 * MAV.rho * (self.Va_norm**2) * MAV.S_wing * MAV.b * (MAV.C_n_0 + MAV.C_n_beta * self._beta + MAV.C_n_p * p * (MAV.b / (2 * self.Va_norm)) + MAV.C_n_r * r * (MAV.b / (2 * self.Va_norm)) + MAV.C_n_delta_a * delta_a + MAV.C_n_delta_r * delta_r)
        
        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T

    def _motor_thrust_torque(self, Va, delta_t, aero):
        n = 60 * aero.KV * aero.R_motor * aero.i0
        J = Va/(n*aero.D_prop)
        
        C_T = aero.C_T2*(J**2) + aero.C_T1*J + aero.C_T0
        C_Q = aero.C_Q2*(J**2) + aero.C_Q1*J + aero.C_Q0
        
        T_p = aero.rho*(n**2)*(aero.D_prop**4)*C_T
        Q_p = aero.rho*(n**2)*(aero.D_prop**5)*C_Q
        
        return T_p, Q_p


    def _update_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        phi, theta, psi = ypr_from_quat(self._state[6:10])
        pdot = rot_from_quat(self._state[6:10]) @ self._state[3:6]
        self.true_state.pn = self._state.item(0)
        self.true_state.pe = self._state.item(1)
        self.true_state.h = -self._state.item(2)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)
