import math
import shapely
from shapely.geometry import LinearRing, LineString, Point
from numpy import sin, cos, pi, sqrt
from random import random
from matplotlib import pyplot as plt

# A prototype simulation of a differential-drive robot with one sensor

# Constants
###########
R = 0.021  # radius of wheels in meters
# circumference is then 2 * 2.1 * pi = 13.1946 CM
# 0.131946 M
L = 0.095  # distance between wheels in meters

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

left_wheel_velocity = 0.6063   # robot left wheel velocity in radians/s
right_wheel_velocity = 0.6063  # robot right wheel velocity in radians/s
# numbers amount to speed 100 in Thymio

# Scalar value 
scalar = W + H


# robot coordinates
robot_pos = {
    "x_coord": [],
    "y_coord": [],
    "Vdir_1": [],
    "Vdir_2": [],
    "sX_coord": [],
    "sY_coord": [],
    "s0X_coord": [],
    "s0Y_coord": [],
    "s4X_coord": [],
    "s4Y_coord": [],
    "lidar": []
}

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


# Lidar setup
#############

def getLaserScans(resolution=10):
    full_scan = []

    # here we emulate the lidar
    for i in range(resolution+1):
        added_angle = i * ((2* math.pi)/resolution)
        current_angle = q + added_angle
        ray = LineString([(x, y), (x+cos(current_angle)*scalar,(y+sin(current_angle)*scalar)) ])
        s = world.intersection(ray)

        # the individual ray distances is what you would get from your lidar sensors
        distance = sqrt((s.x-x)**2+(s.y-y)**2)

        # each measurement will have an individual angle depending on the amount of rays the lidar throws out
        degrees = current_angle * 180 / math.pi

        #hvad skal det her bruges til???
        # here the intersect coords are calculated using the current angle and the measured distance
        x_coord = math.cos(current_angle) * distance
        y_coord = math.sin(current_angle) * distance

        full_scan.append((float(x_coord), float(y_coord))) #
    return full_scan
    # the openCV library has a method to fit a rectangle on a point collection.
    # It also returns how much the rectangle is rotated in relation to your local x-axis.
    #(In the case of the code snippet, the x-axis extends in the direction the robot is facing)
    #Note: integer may not be the best datatype to use if your local coords lies between 0 and 2.

    center, width_height, angle = cv2.minAreaRect(np.asarray(res).astype(np.int))

    #opencv method to return box corners
    points = cv2.boxPoints(box)


# Simulation loop
#################

for cnt in range(1000):

    #simple single-ray sensor pointing straight forward
    ray = LineString([(x, y), (x+cos(q)*scalar,(y+sin(q)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    s = world.intersection(ray)
    distanceWall = sqrt((s.x-x)**2+(s.y-y)**2) # Distance wall

    #simple single-ray sensor pointing to the left
    ray0 = LineString([(x, y), (x+cos(q+0.4)*scalar,(y+sin(q+0.4)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    s0 = world.intersection(ray0)
    distanceWall0 = sqrt((s0.x-x)**2+(s0.y-y)**2) # Distance wall

    #simple single-ray sensor pointing to the right
    ray4 = LineString([(x, y), (x+cos(q-0.4)*scalar,(y+sin(q-0.4)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    s4 = world.intersection(ray4)
    distanceWall4 = sqrt((s4.x-x)**2+(s4.y-y)**2) # Distance wall

    #simple controller - if sensors detect, turn
    turn_rate = 2.2

    if distanceWall4 < 0.35:
        left_wheel_velocity = -turn_rate
        right_wheel_velocity = turn_rate
        #print("sensor0: " + str(distanceWall0))
    # elif distanceWall4 < 0.35:
    #     left_wheel_velocity = turn_rate
    #     right_wheel_velocity = -turn_rate
    #     #print("sensor4: " + str(distanceWall4))
    # elif distanceWall < 0.25:
    #     left_wheel_velocity = -turn_rate
    #     right_wheel_velocity = turn_rate
    # elif distanceWall0 < 0.35 and distanceWall4 < 0.35:
    #     left_wheel_velocity = -turn_rate
    #     right_wheel_velocity = turn_rate
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
        
    #save robot location and sensor rays and lidar scans for matplotlib
    if cnt%50==0:
        robot_pos["x_coord"].append(x)
        robot_pos["y_coord"].append(y)
        robot_pos["Vdir_1"].append(cos(q)*0.05)
        robot_pos["Vdir_2"].append(sin(q)*0.05)
        robot_pos["sX_coord"].append(s.x)
        robot_pos["sY_coord"].append(s.y)
        robot_pos["s0X_coord"].append(s0.x)
        robot_pos["s0Y_coord"].append(s0.y)
        robot_pos["s4X_coord"].append(s4.x)
        robot_pos["s4Y_coord"].append(s4.y)
        robot_pos["lidar"] = getLaserScans()


# Implementing matplotlib
#########################

# plotting lidar lines for every simulation step
def lidarMatPlot(j, list): 
    for i in robot_pos["lidar"]: 
        x, y = i
        plt.plot([robot_pos["x_coord"][j] , x], [robot_pos["y_coord"][j] , y])


for i in range(len(robot_pos["x_coord"])):
    plt.axis([-1, 1, -1, 1])

    lidarMatPlot(i)

    # robot x,y coordinates
    plt.plot(robot_pos["x_coord"][i],robot_pos["y_coord"][i], marker='.', markersize=10, color="red")   
    
    #robot - change to square 
    plt.quiver(robot_pos["x_coord"][i],robot_pos["y_coord"][i],robot_pos["Vdir_1"][i],robot_pos["Vdir_2"][i],headwidth=3.0)
    
    #mid sensor - 2
    plt.plot([robot_pos["x_coord"][i], robot_pos["sX_coord"][i]], [robot_pos["y_coord"][i], robot_pos["sY_coord"][i]])
    
    #left sensor - 0
    plt.plot([robot_pos["x_coord"][i], robot_pos["s0X_coord"][i]], [robot_pos["y_coord"][i], robot_pos["s0Y_coord"][i]])

    #right sensor - 4
    plt.plot([robot_pos["x_coord"][i], robot_pos["s4X_coord"][i]], [robot_pos["y_coord"][i], robot_pos["s4Y_coord"][i]])


    plt.pause(0.01)
    plt.clf()

plt.show()
