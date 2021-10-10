import shapely
from shapely.geometry import LinearRing, LineString, Point
from numpy import sin, cos, pi, sqrt
from random import random
from matplotlib import pyplot as plt

# A prototype simulation of a differential-drive robot with one sensor

# Constants
###########
R = 0.023  # radius of wheels in meters
L = 0.010  # distance between wheels in meters

W = 2.0  # width of arena
H = 2.0  # height of arena

robot_timestep = 0.1        # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch..)

# the world is a rectangular arena with width W and height H
world = LinearRing([(W/2,H/2),(-W/2,H/2),(-W/2,-H/2),(W/2,-H/2)])

# Variables 
###########

x = 0.0   # robot position in meters - x direction - positive to the right 
y = 0.0   # robot position in meters - y direction - positive up
q = 0.0   # robot heading with respect to x-axis in radians 

left_wheel_velocity = 0.5   # robot left wheel velocity in radians/s
right_wheel_velocity = 0.5  # robot right wheel velocity in radians/s

# robot coordinates
x_coord = []
y_coord = []
Vdir_1 = []
Vdir_2 = []

# Kinematic model
#################
# updates robot position and heading based on velocity of wheels and the elapsed time
# the equations are a forward kinematic model of a two-wheeled robot - don't worry just use it
def simulationstep():
    global x, y, q

    for step in range(int(robot_timestep/simulation_timestep)):     #step model time/timestep times
        v_x = cos(q)*(R*left_wheel_velocity/2 + R*right_wheel_velocity/2) 
        v_y = sin(q)*(R*left_wheel_velocity/2 + R*right_wheel_velocity/2)
        omega = (R*right_wheel_velocity - R*left_wheel_velocity)/(2*L)    
    
        x += v_x * simulation_timestep
        y += v_y * simulation_timestep
        q += omega * simulation_timestep

# Simulation loop
#################
file = open("trajectory.dat", "w")

for cnt in range(15000):
    #simple single-ray sensor
    ray = LineString([(x, y), (x+cos(q)*2*W,-(y+sin(q)*2*H)) ])  # a line from robot to a point outside arena in direction of q
    s = world.intersection(ray)
    distanceWall = sqrt((s.x-x)**2+(s.y-y)**2) # Distance wall
    
    #simple controller - change direction of wheels every 10 seconds (100*robot_timestep) unless close to wall then turn on spot
    if distanceWall < 0.30 or distanceWall > 1.85:
        left_wheel_velocity = -0.9
        right_wheel_velocity = 0.9
    else:                
        left_wheel_velocity = 0.5
        right_wheel_velocity = 0.5
            
    #step simulation
    simulationstep()

    arena_wall = world.distance(Point(x,y))

    #check collision with arena walls 
    if arena_wall<L/2:
        print('Stopped due to collission')
        break
        
    if cnt%50==0:
        x_coord.append(x)
        y_coord.append(y)
        Vdir_1.append(cos(q)*0.05)
        Vdir_2.append(sin(q)*0.05)
        #file.write( str(x) + ", " + str(y) + ", " + str(cos(q)*0.05) + ", " + str(sin(q)*0.05) + "\n")

file.close()

# implementing matplotlib

plt.axis([-1, 1, -1, 1])
for i in range(len(x_coord)):
    plt.quiver(x_coord[i],y_coord[i],Vdir_1[i],Vdir_2[i],headwidth=3.0)
    plt.pause(0.01)
plt.show()
