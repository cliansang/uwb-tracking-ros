#!/usr/bin/env python3

# Python Implementation of Standard (Discrete) Kalman Filter using numpy
# The implementation of the Fitler is based on the following paper:
# http://www.cs.unc.edu/~tracker/media/pdf/SIGGRAPH2001_CoursePack_08.pdf

import numpy as np 
from numpy.linalg import multi_dot
from numpy.linalg import inv   # pinv may be more generic 

class KalmanFilter():

    def __init__ (self, A = None, H = None, obj_id=None):        
        if(A is None or obj_id is None):
            raise ValueError("State Model(A) and Transition (H) Matrices must be provided in object creation!")            
        self.n_states = A.shape[1]      # no. of column vectors in A (length of the state vector)
#         self.m_outputs = H.shape[0]   # no. of row vectors in H (lenght of the meas. vector)
        
        # Set the place holders of Parameters for KF when object creation 
        self.A = A
        self.H = H
        self.B = 0 
        self.Q = np.eye(self.n_states) 
        self.R = np.eye(self.n_states) 
        
        self.P_p = np.eye(self.n_states)           # prior before meas. update
        self.P_m = np.eye(self.n_states)           # posterior after meas. update
        self.x_p = np.zeros((self.n_states, 1))    # prior 
        self.x_m = np.zeros((self.n_states, 1))    # posterior 

        self.id = 0 if obj_id is None else obj_id  # id to differentiate multiple KF objects in application 
        self.isKalmanInitialized = False           # Set a flag for kalman initialization 


    # Initialize the State vector for pior and posterior
    def initState (self, x_0): 
        self.x_p = x_0      # Prior state 
        self.x_m = x_0      # Current state after measurement update 


    # Initialize the Covariance matrices for prior and posterior
    def initStateCovariance (self, P0):
        self.P_p = P0    # prior cov.
        self.P_m = P0    # current cov. after measurement update
    

    # Initialize the system parameters for KF 
    def assignSystemParameters(self, A = None, B = None, H = None, Q = None, R = None, P=None, x_0=None): 
               
        if(A is None or H is None):
            raise ValueError("State Model(A) and Transition(H) Matrices must be provided in KF!")            
        self.n_states = A.shape[1]      # no. of column vectors in A (length of the state vector)
#         self.m_outputs = H.shape[0]   # no. of row vectors in H (lenght of the meas. vector)
        
        self.A = A
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n_states) if Q is None else Q
        self.R = np.eye(self.n_states) if R is None else R 
        
        self.P_p = np.eye(self.n_states) if P is None else P             # prior before meas. update
        self.P_m = np.eye(self.n_states) if P is None else P             # posterior after meas. update
        self.x_p = np.zeros((self.n_states, 1)) if x_0 is None else x_0  # prior 
        self.x_m = np.zeros((self.n_states, 1)) if x_0 is None else x_0  # posterior 

        
    # Evaluate the KF based on the measurement (z) and intput (u) data
    def performKalmanFilter(self, z, u): 

        # Time Update
        self.x_p = np.dot(self.A, self.x_m) + np.dot(self.B, u)
        self.P_p = multi_dot([self.A, self.P_m, np.transpose(self.A) ]) + self.Q

        # Measurement Update
        S = multi_dot([self.H, self.P_p, np.transpose(self.H)]) + self.R
        K = multi_dot([self.P_p, np.transpose(self.H), inv(S)])   # Kalman Gain
        z_residual = z - np.dot(self.H, self.x_p)                 # Diff b/w measurement and prior
        self.x_m = self.x_p + np.dot(K, z_residual)
        self.P_m = self.P_p - multi_dot([K, self.H, self.P_p]) 