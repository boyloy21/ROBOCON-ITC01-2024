import numpy as np
import math

class Extended_kalmanFilter:
    def __init__(self,P,Q,R):
        self.A = np.eye(3)
        self.B = np.array([[np.cos(self.yaw), np.sin(self.yaw), 0.0],
                           [-np.sin(self.yaw), np.cos(self.yaw), 0.0],
                           [0.0, 0.0, 0.0]])
        self.H = np.eye(3)
        self.P = P*np.diag(3)
        self.Q = Q*np.diag(3)
        self.R = R*np.diag(3)
    def ekf_predicted(self,x,y,yaw):
        state_pred = np.array([x],[y],[yaw])
        P_pred = self.A @ self.P @ self.A.T + self.Q

        return state_pred,P_pred
    
    def ekf_update(self,state_pred,P_pred,measurement):

        # Observation model
        Y = self.H @ state_pred

        #Calculate error from sensor
        Y_error = measurement - Y
        
        # Inovation Covariance
        S = self.H @ P_pred @ self.H.T + self.R

        # Kalman Gain
        K = P_pred @ self.H.T @ (np.linalg.pinv(S))

        # State Update
        State_up = state_pred + K @ Y_error

        # Predicted update
        P_update = (np.identity(3) - (K @ self.H)) @ P_pred

        return State_up, P_update


if __name__ == "__main__":
    EKF = Extended_kalmanFilter()



        
        
        