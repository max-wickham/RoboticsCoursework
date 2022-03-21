# Kinematics

## Forward and Inverse Kinematics

In order to build a kinematic model of the robotic arm it was necessary to start by finding the DH parameters for each of the joint linkages and construct the DH transform. In order to decide upon the parameters for the transform the following rules were used taken from the John Craig textbook. ADD REF

- $a_i = \mbox{the distance from } \hat{Z}_i  \mbox{ to } \hat{Z}_{i+1} \mbox{ measured along } \hat{X}_i$
- $\alpha_i = \mbox{the angle from } \hat{Z}_i  \mbox{ to } \hat{Z}_{i+1} \mbox{ measured about } \hat{X}_i$
- $d_i = \mbox{the distance from } \hat{X}_{i-1}  \mbox{ to } \hat{Z}_{i} \mbox{ measured along } \hat{Z}_i$
- $\theta_i = \mbox{the angle from } \hat{X}_{i-1}  \mbox{ to } \hat{Z}_{i} \mbox{ measured about } \hat{Z}_i$

Using these rules table REF was constructed for the robotic arm with the linkages and axis used labelled in the diagram in figure REF.



| Linkage | $a$   | $\alpha$    | $d$   |
| ------- | ----- | ----------- | ----- |
| 0       | $0$   | $90\degree$ | $0$   |
| 1       | $A_1$ | $0$         | $D_1$ |
| 2       | $A_2$ | $0$         | $0$   |
| 3       | $A_3$ | $0$         | $0$   |
| 4       | $0$   | $0$         | $0$   |

Once these parameters had been decided on a matrix could be constructed for each of the linkages by using the general matrix given in the lectures shown in figure REF.
$$
M_0 = \begin{bmatrix} 
	C0 & -S0 & 0 & 0 \\
	S0 & C0 & 0 & 0\\
	0 & 0 & 1 & 0 \\
	0 & 0 & 0 & 1 \\
\end{bmatrix} \\
M_1 = \begin{bmatrix} 
	C1 & -S1 & 0 & 0 \\
	0 & 0 & -1 & -D_1\\
	S1 & C1 & 0 & 0 \\
	0 & 0 & 0 & 1 \\
\end{bmatrix} \\
M_2 = \begin{bmatrix} 
	C2 & -S2 & 0 & A_1 \\
	S2 & C2 & 0 & 0\\
	0 & 0 & 1 & 0 \\
	0 & 0 & 0 & 1 \\
\end{bmatrix} \\
M_3 = \begin{bmatrix} 
	C3 & -S3 & 0 & A_2 \\
	S3 & C3 & 0 & 0\\
	0 & 0 & 1 & 0 \\
	0 & 0 & 0 & 1 \\
\end{bmatrix} \\
M_4 = \begin{bmatrix} 
	1 & 0 & 0 & A_3 \\
	0 & 1 & 0 & 0\\
	0 & 0 & 1 & 0 \\
	0 & 0 & 0 & 1 \\
\end{bmatrix} \\
$$
Having constructed the individual matrices the DH transform need to calculate the position and orientation of each linkage relative to the first can then be calculated by matrix multiplication.
$$
M_0M_1 = \begin{bmatrix} 
	C0C1 & -C0S1 & S0 & C0C1A_1 \\
	S0C1 & -S0S1 & -C0 & S0C1A_1 \\
	S1 & C1 & 0 & S1A_1+D_1 \\
	0 & 0 & 0 & 1 \\
\end{bmatrix} \\
M_0M_1M_2 = \begin{bmatrix} 
	C0C12 & -C0S12 & S0 & C0C1A_1) \\
	S0C12 & -S0S12 & -C0 & S0C1A_1)\\
	S12 & C12 & 0 & S1A_1+D_1 \\
	0 & 0 & 0 & 1 \\
\end{bmatrix} \\
M_0M_1M_2M_3 = \begin{bmatrix} 
	C0C123 & -C0S123 & S0 & C0(C12A_2 + C1A_1) \\
	S0C123 & -S0S123 & -C0 & S0(C12A_2 + C1A_1)\\
	S123 & C123 & 0 & S12A_2 + S1A_1+D_1 \\
	0 & 0 & 0 & 1 \\
\end{bmatrix} \\
M_0M_1M_2M_3M_4 = \begin{bmatrix} 
	C0C123 & -C0S123 & S0 & C0(C123A_3 + C12A_2 + C1A_1) \\
	S0C123 & -S0S123 & -C0 & S0(C123A_3 + C12A_2 + C1A_1)\\
	S123 & C123 & 0 & S123A_3 + S12A_2 + S1A_1+D_1 \\
	0 & 0 & 0 & 1 \\
\end{bmatrix} \\
$$


The final DH transform gives us the orientation and poition of the end of the robot gripper relative to the base of the robot in the coordinate frame of the first robot. If we consider a general rotation around each of the three axis we can interpret better the rotation part of this transform.
$$
R_z(\alpha)R_y(\beta)R_x(\gamma) = \begin{bmatrix} 
cos(\alpha) cos(\beta) & cos(\alpha) sin(\beta) sin(\gamma) - sin(\alpha) cos(\gamma)& cos(\alpha) sin(\beta) sin(\gamma) + sin(\alpha) cos(\gamma)\\
sin(\alpha) cos(\beta) & sin(\alpha) sin(\beta) sin(\gamma) + cos(\alpha) cos(\gamma)& sin(\alpha) sin(\beta) sin(\gamma) - cos(\alpha) sin(\gamma)\\
	-sinb(\beta) & cos(\beta)sin(\gamma) & cos(\beta)cos(\gamma)\\
\end{bmatrix} \\
= 

\begin{bmatrix} 
	C0C123 & -C0S123 & S0  \\
	S0C123 & -S0S123 & -C0\\
	S123 & C123 & 0 \\
\end{bmatrix}
$$
Sincce we know that the rotation around the $z$ axis is $\alpha$ as this is the only joint that can rotate in this axis we can see from the above identity that $\beta = -(\theta_1+\theta_2 + \theta_3)$, which corresponds to the final gripper angle. This means that in order to choose a final wrist angle we must fix the value of $(\theta_1+\theta_2 + \theta_3)$.

The equations describing the relations between the joint angles and the final position of the gripper are also given by the DH transform.
$$
x = C0(C123A_3 + C12A_2 + C1A_1)\\
y = S0(C123A_3 + C12A_2 + C1A_1)\\
z = S123A_3 + S12A_2 + S1A_1+D_1\\
$$
In order to solve these equations the following trigonometric identities will be used where applicable.
$$
cos(\theta+\phi) = cos(\theta)cos(\phi) - sin(\theta)sin(\phi) \\
sin(\theta+\phi) = cos(\theta)sin(\phi) + cos(\phi)sin(\theta) \\
sin(\theta)^2 + cos(\theta)^2 = 1
$$
These equations make solving the forward kinematics for the gripper positions trivial as the joint angles can simply be plugged into this equation, in order to find the joint angles from the gripper position we must first rearrange these equations. Due to there being 4 unknowns on the right hand side of these equitation we must first fix a value of $(\theta_1+\theta_2 + \theta_3)$ corresponding to the final gripper angle. 
$$
\frac{y}{x} = S0/C0 \\
\frac{y}{x} = tan(\theta_0) \\
\theta_0 = tan^{-1}(\frac{y}{x}) \\
x = C0(C123A_3 + C12A_2 + C1A_1)\\
(x / C0) - C123A_3 = C12A_2 + C1A_1 = a \\
\mbox{ Where a is known as C0 and C123 are known} \\
y = S0(C123A_3 + C12A_2 + C1A_1)\\
(y / S0) - C123A_3 = C12A_2 + C1_A1 = a 
\\\mbox{ This identity can be used if C0 = 0 to avoid division by 0} \\
z = S123A_3 + S12A_2 + S1A_1+D_1\\
z - S123A_3 - D_1 = S12A_2 + S1A_1 = b \\
\mbox{ Where b is known as S123 and D1 are known}
$$
This results in the following equations that must be solved to find $\theta_1$ and $\theta_2$ and $\theta_3$
$$
b = S12A_2 + S1A_1 \\
a = C12A_2 + C1A_1 \\
a^2 = C12^2A_2^2+C1^2A_1^2+2C1C12 \\
b^2 = S12^2A_2^2+S1^2A_1^2+2S1S12 \\
a^2 + b^2 = A_1^2(C1^2+S1^2) + A_2^2(C12^2+S12^2) + 2A_1A_2(C1C12 + S1S12) \\
a^2 + b^2 = (A_1^2+A_2^2) + 2A_1A_2(C1C12 + S1S12) \\
a^2 + b^2 = (A_1^2+A_2^2) + 2A_1A_2C2 \\
cos(\theta_2) = \frac{1}{2A_1A_2}((a^2+b^2)-(A_1^2+A_2^2)) \\
\theta_2 = \pm cos^{-1}(\frac{1}{2A_1A_2}((a^2+b^2)-(A_1^2+A_2^2))) \\
\mbox{The sign chosen here decides whether the up or down elbow solution will be given} \\
M_1 = C2A_2+A_1 \\
M_2 = S2A_2 \\
\mbox{Where } M_1 \mbox{ and } M_2 \mbox{ are known values at this stage as } \theta_2 \mbox{ is known}\\
a = M_1C1 - M_1S1 \\
b = M_1S1 + M_2S1 \\
b = P(cos(-\theta_1+\phi)sin(\theta_1)+sin(-\theta_1+\phi)cos(\theta_1))\\
\mbox{With the value of } \phi \mbox{ being such that} \\
\frac{sin(-\theta_1+\phi)}{cos(-\theta_1+\phi)} = \frac{M_2}{M_1} \\
Psin(-\theta_1+\phi) = M_2 \\
Pcos(-\theta_1+\phi) = M_1 \\
tan(-\theta_1+\phi) = \frac{M_2}{M_1} \\
\mbox{Using the sin identity} \\
b = Psin(\phi) \\
a = P(cos(-\theta_1+\phi)C1-sin(-\theta_1+\phi)S1) \\
\mbox{Using the cos identity} \\
a = Pcos(\phi) \\
tan(\phi) = \frac{b}{a} \\
\theta_1 = tan^{-1}(b/a) - tan^{-1}(M_2/M_1) \\
\theta_1 = tan^{-1}(b/a) - tan^{-1}(A_2sin(\theta_2) / (A_1+A_2cos(\theta_2))) \\
\mbox{Use the pre decided gripper angle } (\theta_1+\theta_2+\theta_3) \\
\theta_3 = (\theta_1+\theta_2+\theta_3) - \theta_1 - \theta_2
$$
The choice of the positive or negative solution to the $cos^{-1}$ in the $\theta_2$ equation decides whether or not the robot will be in an up or down elbow position. This value of $\theta_2$ is then used to find $\theta_1$. We can use the fact that we know $cos(\theta_2)$ when calculating $\theta_1$ to give the final equations.
$$
a = (x / C0) - C123A_3 \\
b = z - S123A_3 - D_1\\
\theta_0 = tan^{-1}(\frac{y}{x}) \\
\theta_2 = \pm cos^{-1}(\frac{1}{2A_1A_2}((a^2+b^2)-(A_1^2+A_2^2))) \\
\theta_1 = tan^{-1}(b/a) - tan^{-1}(\pm A_2\sqrt{1-cos(\theta_2)^2} / (A_1+A_2cos(\theta_2))) \\
\theta_3 = (\theta_1+\theta_2+\theta_3) - \theta_1 - \theta_2
$$


## Finding Robot Servo Values

Once a model has been formed that allows the computation of the theoretical forward and inverse kinematics a framework must be built to move between this model and actual values to send to the servos. First we found the values of each of the parameters shown in the DH transform. For the standard gripper the values we chose are shown in table REF. The value of $A_3$ does change dependent on the gripper attachment being used with the value shown here being the one used for the standard gripper.

| $A_1$ | 13   |
| ----- | ---- |
| $A_2$ | 12.4 |
| $A_3$ | 14.6 |
| $D_1$ | 7.7  |

When converting from radians used in the kinematic model to servo values the offsets needed for each of the servos and the direction scale factor must be found. This is necessary to ensure that a rotation for a linkage in the model corresponds to the correct rotation of the corresponding servo and that the servo moves in the correct direction when the rotation is changed. Between linkage 1 and 2 there is an angle offset caused by linkage 2 not lying on the x-axis of linkage and the negative of the offset must also then be applied to linkage 2. The offsets and direction scales for each servo that were used are shown in table REF. 

| Servo | Angle Offest     | Direction Scale |
| ----- | ---------------- | --------------- |
| 0     | 0                | 1               |
| 1     | $-0.18356-\pi/2$ | -1              |
| 2     | $0.18356+\pi/2$  | -1              |
| 3     | $\pi$            | -1              |

## Extension Kinematics For Plate Control Task 4

In order to be able to place an object in any orientation a spinable plate was added that the arm could control. This introduced two problems, how much to turn the plate to ensure a given location on the plate will be at the correct orientation when an object is placed on it and what the new coordinate of the given locaiton will be after the plate is turned. 

The diagram below is used to describe the problem.

ADD DIAGRAM

ADD MATHS FOR SPIN AMOUNT

ADD MATHS FOR NEW LOCATION

