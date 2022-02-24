from __future__ import annotations

from dataclasses import dataclass
import numpy as np

@dataclass
class DHJoint:
    alpha_r : float
    d : float
    a : float
    prev_joint : DHJoint = None

    def __post_init__(self) -> DHJoint:
        if self.prev_joint is None and (self.alpha_r != 0 or self.d !=0 or self.a != 0):
            self.prev_joint = DHJoint(0,0,0)
        elif self.prev_joint is None:
            self._matrix = np.zeros((4,4))
            return 
        theta_r = 0
        line0 = np.array([np.cos(theta_r), -1*np.sin(theta_r), 0, self.prev_joint.a])
        line1 = np.array([np.sin(theta_r)*np.cos(self.prev_joint.alpha_r), 
            np.cos(theta_r)*np.cos(self.prev_joint.alpha_r),
            -1*np.sin(self.prev_joint.alpha_r),
            -1*np.sin(self.prev_joint.alpha_r)*self.d])
        line2 = np.array([np.sin(theta_r)*np.sin(self.prev_joint.alpha_r), 
            np.cos(theta_r)*np.sin(self.alpha_r),
            np.cos(self.prev_joint.alpha_r),
            np.cos(self.prev_joint.alpha_r)*self.d])
        line_3 = [0,0,0,1]
        self._matrix =  np.array([line0,line1,line2,line_3])
        return self


    def matrix(self, theta_r) -> np.ndarray:
        '''Creates and returns the df matrix'''
        self._matrix[0][0] = np.cos(theta_r)
        self._matrix[0][1] = -1*np.sin(theta_r)
        self._matrix[1][0] = np.sin(theta_r)*np.cos(self.prev_joint.alpha_r)
        self._matrix[1][1] = np.cos(theta_r)*np.cos(self.prev_joint.alpha_r)
        self._matrix[2][0] = np.sin(theta_r)*np.sin(self.prev_joint.alpha_r)
        self._matrix[2][1] = np.cos(theta_r)*np.sin(self.alpha_r)
        return self._matrix


class Robot:

    def __init__(self, joints : list) -> None:
        self.joints = joints

    def forward(self, angles_r : list) -> list:
        coordinates = []
        matrix = np.eye(4,4)
        for angle,joint in zip(angles_r,self.joints):
            angle *= (np.pi/180)
            print(joint.matrix(angle))
            matrix = np.matmul(matrix,joint.matrix(angle))
            print(matrix)
            position = (matrix[0][3],matrix[1][3],matrix[2][3])
            coordinates.append(position)
        return coordinates 

    def inverse(self, x, y, z) -> list:
        pass

