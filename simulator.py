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

# robot coordinates
robot_pos = {
    "x_coord": [],
    "y_coord": [],
    "Vdir_1": [],
    "Vdir_2": [],
    "sX_coord": [],
    "sY_coord": []
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

# Simulation loop
#################

for cnt in range(10000):
    #simple single-ray sensor pointing straight forward
    ray = LineString([(x, y), (x+cos(q)*2*W,(y+sin(q)*2*H)) ])  # a line from robot to a point outside arena in direction of q
    s = world.intersection(ray)
    distanceWall = sqrt((s.x-x)**2+(s.y-y)**2) # Distance wall
#    print(s)
#   print(list(s.coords)[0])

    ray0 = LineString([(x, y), (x+cos(q-0.04)*2*W,(y+sin(q-0.04)*2*H)) ])  # a line from robot to a point outside arena in direction of q
    s0 = world.intersection(ray)
    distanceWall0 = sqrt((s0.x-x)**2+(s0.y-y)**2) # Distance wall

    ray4 = LineString([(x, y), (x+cos(q+0.04)*2*W,(y+sin(q+0.04)*2*H)) ])  # a line from robot to a point outside arena in direction of q
    s4 = world.intersection(ray)
    distanceWall4 = sqrt((s4.x-x)**2+(s4.y-y)**2) # Distance wall

    #simple controller - if close to wall, turn on spot
    if distanceWall0 < 0.30:
        left_wheel_velocity = -0.9
        right_wheel_velocity = 0.9
    elif distanceWall4 < 0.30:
        left_wheel_velocity = 0.9
        right_wheel_velocity = -0.9
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
        
    #save robot location and rays for matplotlib    
    if cnt%50==0:
        robot_pos["x_coord"].append(x)
        robot_pos["y_coord"].append(y)
        robot_pos["Vdir_1"].append(cos(q)*0.05)
        robot_pos["Vdir_2"].append(sin(q)*0.05)
        robot_pos["sX_coord"].append(s.x)
        robot_pos["sY_coord"].append(s.y)
        

# implementing matplotlib
plt.axis([-1, 1, -1, 1])
for i in range(len(robot_pos["x_coord"])):
    #robot
    plt.quiver(robot_pos["x_coord"][i],robot_pos["y_coord"][i],robot_pos["Vdir_1"][i],robot_pos["Vdir_2"][i],headwidth=3.0)
    #ray
    plt.plot([robot_pos["x_coord"][i], robot_pos["sX_coord"][i]], [robot_pos["y_coord"][i], robot_pos["sY_coord"][i]])
    #ray0
    #plt.plot()
    #ray4
    #plt.plot()
    plt.pause(0.01)
plt.show()
