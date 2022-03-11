
from dataclasses import dataclass

import numpy as np

@dataclass
class Model:

    A1: float
    A2: float
    A3: float
    D0: float
    Up : bool = True

    def angles(self, x : float, y : float, z: float, wrist_angle : float) -> tuple[float,float,float,float]:
        theta123 = wrist_angle
        theta0 = np.arctan2(y,x)
        C0, S0 = np.cos(theta0), np.sin(theta0)
        C123, S123 = np.cos(theta123), np.sin(theta123)
        a = (x - C0*C123*self.A3) if C0 != 0 else (y - S0*C123*self.A3)
        a /= C0 if C0 != 0 else S0
        b = z - S123*self.A3 - self.D0 
        C2 = ( (pow(a,2)+pow(b,2)) - (pow(self.A1,2) + pow(self.A2,2)))
        C2 /= (2*self.A1*self.A2)
        theta2 = np.arccos(C2)
        if self.Up:
            theta2 *= -1
        theta1 = np.arctan(b/a) - np.arctan(self.A2*np.sin(theta2) / (self.A1 + self.A2*C2))
        # S1 = (b*(self.A1+self.A2*C2) - a*self.A2*pow((1-pow(C2,2)),0.5))
        # S1 /= (pow(a,2)+pow(b,2))
        # theta1 = np.arcsin(S1)
        # # testing
        # C2 = ((pow(a,2)+pow(b,2)))
        # C2 -= (pow(self.A1,2)+pow(self.A2,2))
        # C2 /= 2*(self.A1*self.A2)
        # print("c2",np.arccos(C2)/np.pi*180)
        # C1 = b / 2
        # C1 /= pow((pow(self.A1,2) + pow(self.A2,2) + 2*self.A1*self.A2*C2), 0.5)
        # print("c1",np.arccos(C1)/np.pi*180, theta1/np.pi*180)
        # # end testing 
        # print("theta1 ", theta1/np.pi*180)
        # D = pow((self.A1+self.A2*C2),2)+((1-pow(C2,2))*pow(self.A2,2))
        # D = pow(D,0.5)
        # N1 = self.A1+self.A2*C2
        # Ttheta1 = np.arcsin(a/D) - np.arcsin(N1/D)
        # print("theta1 ", Ttheta1/np.pi*180)
        # theta1 -= theta1 + Ttheta1
        # if (abs(self.A1*np.cos(theta1)+self.A2*np.cos(theta1+theta2)) - abs(a)) > 0.001:
        #     print("Error")
        # if self.Up:
        #    theta1 += theta2
        #    theta2 *= -1
        n = self.A1*np.sin(theta1)
        m = self.A2*np.sin(theta1+theta2)
        theta3 = (theta123 - theta1 - theta2)
        return theta0, theta1, theta2, theta3

    def forward(self, theta0, theta1, theta2, theta3):
        # temp1 = theta1
        # temp2 = theta2
        # theta2 = temp2 * -1
        # theta1 = temp1 - theta1
        C0 = np.cos(theta0)
        C1 = np.cos(theta1)
        C12 = np.cos(theta1+theta2)
        C123 = np.cos(theta1+theta2+theta3)
        S0 = np.sin(theta0)
        S1 = np.sin(theta1)
        S12 = np.sin(theta1+theta2)
        S123 = np.sin(theta1+theta2+theta3)
        x = C0*(C123*self.A3+C12*self.A2+C1*self.A1)
        y = S0*(C123*self.A3+C12*self.A2+C1*self.A1)
        z = S123*self.A3+S12*self.A2+S1*self.A1 + self.D0
        gripper_angle = np.mod(theta1+theta2+theta3+8*np.pi, 2*np.pi) - 2*np.pi
        position = [x,y,z,gripper_angle]
        return position

        #theta0, theta1, theta2, theta3 = self.angles(x,y,z,wrist_angle)
        #matrix_0 = np.array([np.array([1,0,0]),np.array([0,np.cos(theta0), -1*np.sin(theta0)]),np.array([0,np.sin(theta0),np.cos(theta0)])])
        
        
        # matrix_0 = np.array([np.array([np.cos(theta0), -1*np.sin(theta0),0]),np.array([np.sin(theta0),np.cos(theta0),0]),np.array([0,0,1])])
        # matrix_05 = np.array([np.array([1,0,0]),np.array([0,0, -1]),np.array([0,1,0])])
        # matrix_1 = np.array([np.array([np.cos(theta1), -1*np.sin(theta1),0]),np.array([np.sin(theta1),np.cos(theta1),0]),np.array([0,0,1])])
        # matrix_2 = np.array([np.array([np.cos(theta2), -1*np.sin(theta2),0]),np.array([np.sin(theta2),np.cos(theta2),0]),np.array([0,0,1])])
        # matrix_3 = np.array([np.array([np.cos(theta3), -1*np.sin(theta3),0]),np.array([np.sin(theta3),np.cos(theta3),0]),np.array([0,0,1])])
        # p0 = np.array([[0,0,self.D0]]).transpose()
        # p1 = np.array([[self.A1,0,0]]).transpose()
        # p2 = np.array([[self.A2,0,0]]).transpose()
        # p3 = np.array([[self.A3,0,0]]).transpose()
        # pos0 = [0,0,self.D0]
        # pos1 = list(np.matmul(matrix_0,(np.matmul(matrix_05,np.matmul(matrix_1,p1)) + p0)).transpose()[0])
        # pos2 = list(np.matmul(matrix_0,(np.matmul(matrix_05,np.matmul(matrix_1,p1 + np.matmul(matrix_2,p2))) + p0)).transpose()[0])
        # pos3 = list(np.matmul(matrix_0,(np.matmul(matrix_05,np.matmul(matrix_1,p1 + np.matmul(matrix_2,p2 + np.matmul(matrix_3,p3)))) + p0)).transpose()[0])
        # return pos3



    def joint_positions(self, x : float, y : float, z : float, wrist_angle : float) -> tuple[tuple[float,float,float],tuple[float,float,float],tuple[float,float,float],tuple[float,float,float]]:
        theta0, theta1, theta2, theta3 = self.angles(x,y,z,wrist_angle)
        #matrix_0 = np.array([np.array([1,0,0]),np.array([0,np.cos(theta0), -1*np.sin(theta0)]),np.array([0,np.sin(theta0),np.cos(theta0)])])
        matrix_0 = np.array([np.array([np.cos(theta0), -1*np.sin(theta0),0]),np.array([np.sin(theta0),np.cos(theta0),0]),np.array([0,0,1])])
        matrix_05 = np.array([np.array([1,0,0]),np.array([0,0, -1]),np.array([0,1,0])])
        matrix_1 = np.array([np.array([np.cos(theta1), -1*np.sin(theta1),0]),np.array([np.sin(theta1),np.cos(theta1),0]),np.array([0,0,1])])
        matrix_2 = np.array([np.array([np.cos(theta2), -1*np.sin(theta2),0]),np.array([np.sin(theta2),np.cos(theta2),0]),np.array([0,0,1])])
        matrix_3 = np.array([np.array([np.cos(theta3), -1*np.sin(theta3),0]),np.array([np.sin(theta3),np.cos(theta3),0]),np.array([0,0,1])])
        p0 = np.array([[0,0,self.D0]]).transpose()
        p1 = np.array([[self.A1,0,0]]).transpose()
        p2 = np.array([[self.A2,0,0]]).transpose()
        p3 = np.array([[self.A3,0,0]]).transpose()
        pos0 = [0,0,self.D0]
        pos1 = list(np.matmul(matrix_0,(np.matmul(matrix_05,np.matmul(matrix_1,p1)) + p0)).transpose()[0])
        pos2 = list(np.matmul(matrix_0,(np.matmul(matrix_05,np.matmul(matrix_1,p1 + np.matmul(matrix_2,p2))) + p0)).transpose()[0])
        pos3 = list(np.matmul(matrix_0,(np.matmul(matrix_05,np.matmul(matrix_1,p1 + np.matmul(matrix_2,p2 + np.matmul(matrix_3,p3)))) + p0)).transpose()[0])
        return [pos0, pos1, pos2, pos3]

A1,A2,A3,D0 = 13,12.4,12.6,7.7
model = Model(A1,A2,A3,D0)
# print(model.joint_positions(10,0,10,0))
# t0,t1,t2,t3 = model.angles(10,0,10,0)
# x = np.cos(t0)*np.cos(t1+t2+t3)*A3+np.cos(t0)*np.cos(t1+t2)*A2 + np.cos(t0)*np.cos(t1)*A1
# y = np.sin(t0)*np.cos(t1+t2+t3)*A3+np.sin(t0)*np.cos(t1+t2)*A2 + np.sin(t0)*np.cos(t1)*A1
# z = np.sin(t1+t2+t3)*A3 + np.sin(t1+t2)*A2+np.sin(t1)*A1 + D0
# print(x,y,z)

#print(model.joint_positions(10,0,10,np.pi/2))
angles = model.angles(15,0,0,-1*np.pi/2)
#print(angles)
positions = model.forward(angles[0],angles[1],angles[2],angles[3])
print(positions)
#print(model.joint_positions(10,10,20,0))