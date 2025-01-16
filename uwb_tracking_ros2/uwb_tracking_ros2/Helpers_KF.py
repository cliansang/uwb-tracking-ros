#!/usr/bin/env python3

# The helper functions regarding motion models for Kalman filter implementation
import numpy as np 


# Initialize the Constant Velocity model for UWB-based Positioning in KF 
def initConstVelocityKF():

    dt = 0.1    # system update rate (10 Hz)
    
    # variance of the measurement noise, supposing the error is 15 cm (0.15 * 0.15) on X- & Y-axes
    # Tune the values based on your set-up and the demanded performance
    v_n = np.array([ 0.0225, 0.0225,  0.08])
    sigma_sq = 0.01    # Process noise value based on 10cm precision
    
    # Initial assumed values of the state which includes pose and their corresponding velocites
    x_0 = np.array([1.2, 1.82 , 0.885 , 1, 2, 1.5])  #  Initial guest
    x_0.shape = (len(x_0),1)      # force to be a column vector (6,1)
    
    # Initial value of State Covariance Matrix or Error Covariance 
    P_0 = np.multiply(2, np.eye(6))
    
    # State Model or Transition Matrix 
    A = np.array([[1, 0, 0, dt, 0, 0],
                  [0, 1, 0, 0, dt, 0],
                  [0, 0, 1, 0, 0, dt],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1]])
    
    B = 0 # No control input
    
    #  Relation b/w the observed Measurement and the State vector
    H = np.array([[1, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0]])
    
    # Measurement Noise Covariance
    R = np.array([[v_n[0], 0, 0],
                  [0, v_n[1], 0],
                  [0, 0, v_n[2]]])
    
    '''
    Process Noise Covariance (6x6)
    Ref1: Survey of Maneuvering Target Tracking. Part I: Dynamic Models bz X. Rong Li and et.
    Ref2: Estimation with applications to Tracking and Navigation by Z. Bar-Shalom and et al. [chap. 6]
    Ref3: Mobile Positioning and Tracking (2nd Edition) by S. Frattasi & F. D. Rosa [sec: 6.4]
    '''
    q_uD = (np.power(dt, 4))/4   # (dt, 3)/3  in some lit.
    q_oD = (np.power(dt, 3))/2   # (dt, 2)/2  in some lit.
    q_lD = np.square(dt)         # dt         in some lit.
    
    Q_0 = np.array([[q_uD, 0, 0, q_oD, 0, 0],
                    [0, q_uD, 0, 0, q_oD, 0],
                    [0, 0, q_uD, 0, 0, q_oD],
                    [q_oD, 0, 0, q_lD, 0, 0],
                    [0, q_oD, 0, 0, q_lD, 0],
                    [0, 0, q_oD, 0, 0, q_lD]])

    Q = Q_0 * sigma_sq
    
    return A, B, H, Q, R, P_0, x_0 


# Constant acceleration Motion Model for UWB-based Positioning in KF 
def initConstAccelerationKF():
    
    dt = 0.1    # system update rate (10Hz)
    
    # variance of the measurement noise, supposing the error is 15 cm (0.15 * 0.15) on X- & Y-axes
    # Tune the values based on your set-up and the demanded performance
    v_n = np.array([ 0.0225, 0.0225,  0.08])
    sigma_sq = 0.01    # Process noise value based on 10cm precision

    # Initial guest for state vector 
    x_0 = np.array([1.2, 1.82 , 0.885 , 1, 2, 1.5, 0.5, 1.0, 0.75])  #  Initial guest
    x_0.shape = (len(x_0),1)      # force to be a column vector  (9, 1)

    # Initial guese of State Covariance Matrix shoudn't be zeros 
    P_0 = np.multiply(2, np.eye(len(x_0)))  # (9x9)


    # State Model or Transition Matrix  (9x9)
    A_oD = (np.power(dt, 2))/2  # coefficient for acceleration (off-diagonal)
    A = np.array([[1, 0, 0, dt, 0, 0, A_oD, 0, 0],
                  [0, 1, 0, 0, dt, 0, 0, A_oD, 0],
                  [0, 0, 1, 0, 0, dt, 0, 0, A_oD],
                  [0, 0, 0, 1, 0, 0, dt, 0, 0],
                  [0, 0, 0, 0, 1, 0, 0, dt, 0],
                  [0, 0, 0, 0, 0, 1, 0, 0, dt],
                  [0, 0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1]])

    B = 0 # No control input
    
    #  Relation b/w the observed Measurement and the State vector (3x9)
    H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0, 0, 0, 0]])
    
    # Measurement Noise Covariance (3x3) 
    R = np.array([[v_n[0], 0, 0],
                  [0, v_n[1], 0],
                  [0, 0, v_n[2]]])
    
    '''
    Process Noise Covariance (9x9)
    Ref1: Survey of Maneuvering Target Tracking. Part I: Dynamic Models bz X. Rong Li and et.
    Ref2: Estimation with applications to Tracking and Navigation by Z. Bar-Shalom and et al. [chap. 6]
    Ref3: Mobile Positioning and Tracking (2nd Edition) by S. Frattasi & F. D. Rosa [sec: 6.4]
    '''
    q_t4 = (np.power(dt, 4))/4   # (dt, 3)/3  in some lit.
    q_t3 = (np.power(dt, 3))/2   # (dt, 2)/2  in some lit.
    q_t2 = np.square(dt)         # dt         in some lit.
    
    Q_0 = np.array([[q_t4, 0, 0, q_t3, 0, 0, q_t2, 0, 0],
                    [0, q_t4, 0, 0, q_t3, 0, 0, q_t2, 0],
                    [0, 0, q_t4, 0, 0, q_t3, 0, 0, q_t2],
                    [q_t3, 0, 0, q_t2, 0, 0, dt, 0, 0],
                    [0, q_t3, 0, 0, q_t2, 0, 0, dt, 0],
                    [0, 0, q_t3, 0, 0, q_t2, 0, 0, dt],
                    [q_t2, 0, 0, dt, 0, 0, 1, 0, 0],
                    [0, q_t2, 0, 0, dt, 0, 0, 1, 0],
                    [0, 0, q_t2, 0, 0, dt, 0, 0, 1]])

    Q = Q_0 * sigma_sq
    
    return A, B, H, Q, R, P_0, x_0 
        
        