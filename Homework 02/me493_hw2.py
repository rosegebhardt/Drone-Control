import numpy as np

class simulator(object):
    def __init__(self):
        self.ang_vel = np.array([p,q,r])
        self.lin_vel = np.array([u,v,w])
        self.f = np.array([f_1,f_2,f_3])
        self.M = np.array([m_1,m_2,m_3])
        self.mass = m
        self.euler_ang = np.array([roll,pitch,yaw])
    def skew(vect):
        return np.array([[0,-vect[2],vect[1]],[vect[2],0,-vect[0]],[-vect[1],vect[0],0]])
    def rotation(self):
        R_roll = np.array([[1,0,0],
                           [0,np.cos(self.euler_ang[0]),np.sin(self.euler_ang[0])],
                           [0,-np.sin(self.euler_ang[0],np.cos(self.euler_ang[0])]])
        R_pitch = np.array([[np.cos(self.euler_ang[1]),0,-np.sin(self.euler_ang[1])],
                           [0,1,0],
                           [np.sin(self.euler_ang[1]),0,np.cos(self.euler_ang[1])]])
        R_yaw = np.array([[np.cos(self.euler_ang[2]),np.sin(self.euler_ang[2]),0],
                          [-np.sin(self.euler_ang[2]),np.cos(self.euler_ang[2]),0],
                          [0,0,1]])
        return R_roll*R_pitch*R_yaw
    def rotation_dot(self):
        return self.rotation*self.skew(self.aang_vel)
    
    def __call__(self,X):
      layer_1 = tf.keras.activations.elu(X @ self.weights['w_1'] + self.biases['b_1'], alpha=1.0)
      layer_2 = tf.keras.activations.elu(layer_1 @ self.weights['w_2'] + self.biases['b_2'], alpha=1.0)
      layer_3 = tf.keras.activations.elu(layer_2 @ self.weights['w_3'] + self.biases['b_3'], alpha=1.0)
      layer_4 = tf.keras.activations.elu(layer_3 @ self.weights['w_4'] + self.biases['b_4'], alpha=1.0)
      y_hat = tf.keras.activations.sigmoid(layer_4)
      return y_hat