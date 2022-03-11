from model import Model
# from robot_model import DHJoint, Robot
import numpy as np
# import pygame
# from pygame import Vector2
# import sys
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button

A1,A2,A3,D0 = 13,12.4,14.6,7.7
model = Model(A1,A2,A3,D0)
coordinates = model.joint_positions(20,5,10,-1*np.pi/2*0)

fig = plt.figure(figsize=(4,4))
ax = fig.add_subplot(111, projection='3d')


x = fig.add_subplot(80,1,70)
x_slider = Slider(
    ax=x,
    label='X coordinate',
    valmin=0,
    valmax=40,
    valinit=15,
)
y = fig.add_subplot(80,1,73)
y_slider = Slider(
    ax=y,
    label='Y coordinate',
    valmin=-40,
    valmax=40,
    valinit=0,
)
z = fig.add_subplot(80,1,76)
z_slider = Slider(
    ax=z,
    label='Z coordinate',
    valmin=0,
    valmax=40,
    valinit=2,
)
theta = fig.add_subplot(80,1,80)
theta_slider = Slider(
    ax=theta,
    label='Tilt',
    valmin=-90,
    valmax=90,
    valinit=-1*np.pi/2,
)

# The function to be called anytime a slider's value changes
def update(val):
    angles = model.angles(x_slider.val,y_slider.val,z_slider.val,theta_slider.val/180*np.pi)
    angles = [x for x in angles]
    fig.text(2,4,'Angle 0 ' + str(angles[0]))
    angles.append(angles[1]+angles[2])
    #print(angles)
    coordinates = model.joint_positions(x_slider.val,y_slider.val,z_slider.val,theta_slider.val/180*np.pi)
    print(coordinates)
    x = [i[0] for i in coordinates]
    y = [i[1] for i in coordinates]
    z = [i[2] for i in coordinates]
    ax.clear()
    ax.plot(x,y,z)
    ax.set_xlim(0,40)
    ax.set_ylim(-40,40)
    ax.set_zlim(0,40)
    fig.canvas.draw_idle()

update(0)
# register the update function with each slider
x_slider.on_changed(update)
y_slider.on_changed(update)
z_slider.on_changed(update)
theta_slider.on_changed(update)

plt.show()



# pygame.init()

# size = width, height = 640, 480
# screen = pygame.display.set_mode(size)



# print(coordinates)
# for i in range(len(coordinates)):
#     coordinates[i] = (int(coordinates[i][0] * 10), height - int(coordinates[i][2] * 10))  
# print(coordinates)


# def render(points):
#     black = 0, 0, 0
#     white = 255, 255, 255

#     screen.fill(white)
#     for i in range(1, len(points)):
#         prev = points[i-1]
#         cur = points[i]
#         pygame.draw.aaline(screen, black, prev, cur)
#     for point in points:
#         pygame.draw.circle(screen, black, (int(point[0]), int(point[1])), 5)
#     pygame.display.flip()

# x = 25
# y = 0
# z = 13
# theta = -1*np.pi/2*0
# while 1:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT: sys.exit()
#     render(coordinates)
#     pygame.time.wait(int(1000/60))












# points = list(map(Vector2, [(100, 100), (200, 100), (300, 100), (400, 100), (500, 100)]))
# target = Vector2(450, 300)
# target_speed = Vector2(3, 3)

# rel_points = []
# angles = []

# max_angle = 360 # Adjust for limited angles

# for i in range(1, len(points)):
#     rel_points.append(points[i] - points[i-1])
#     angles.append(0)

# def solve_ik(i, endpoint, target):
#     if i < len(points) - 2:
#         endpoint = solve_ik(i+1, endpoint, target)
#     current_point = points[i]

#     angle = (endpoint-current_point).angle_to(target-current_point)
#     angles[i] += min(max(-3, angle), 3)
#     angles[i] = min(max(180-max_angle, (angles[i]+180)%360), 180+max_angle)-180

#     return current_point + (endpoint-current_point).rotate(angle)

# def render():
#     black = 0, 0, 0
#     white = 255, 255, 255

#     screen.fill(white)
#     for i in range(1, len(points)):
#         prev = points[i-1]
#         cur = points[i]
#         pygame.draw.aaline(screen, black, prev, cur)
#     for point in points:
#         pygame.draw.circle(screen, black, (int(point[0]), int(point[1])), 5)
#     pygame.draw.circle(screen, black, (int(target[0]), int(target[1])), 10)
#     pygame.display.flip()

# while 1:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT: sys.exit()

#     solve_ik(0, points[-1], target)
#     angle = 0
#     for i in range(1, len(points)):
#         angle += angles[i-1]
#         points[i] = points[i-1] + rel_points[i-1].rotate(angle)

#     target += target_speed
#     if target.x <= 0 or target.x >= width:
#         target_speed.x = -target_speed.x
#     if target.y <= 0 or target.y >= height:
#         target_speed.y = -target_speed.y

#     render()

#     pygame.time.wait(int(1000/60))


# joint_0 = DHJoint(0,0,1)
# joint_1 = DHJoint(0,0,1,joint_0)
# robot = Robot([joint_0,joint_1])
# coordinates = robot.forward([45*np.pi/180,0])
# print(coordinates)

