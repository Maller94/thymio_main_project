import numpy as np 

# create map
map = np.zeros(16*10).reshape(10,16)

print("Empty Map")
print(map)

# this is how you print the y,x as a tuple where the swamps are
"""
for i in range(6):
    print(tuple(np.argwhere(map == 1)[i]))
"""

# simple function taking position of robot and setting that to be swamp
def detectedSwamp(x,y): 
    belief(x,y)
    map[y][x] = 2

# orientation
q = 3

# function to update surrounding boxes to belief system
def belief(x,y): 
    # if robot is looking left
    if q == 1:
        for i in range(3): 
            map[(y-1)+i][x] = 1
            map[(y-1)+i][x-1] = 1
    # if robot is looking up
    if q == 2:
        for i in range(3): 
            map[y][(x-1)+i] = 1
            map[y-1][(x-1)+i] = 1
    # if robot is looking right
    if q == 3:
        for i in range(3): 
            map[(y-1)+i][x] = 1
            map[(y-1)+i][x+1] = 1
    # if robot is looking down
    if q == 4:
        for i in range(3): 
            map[y][(x-1)+i] = 1
            map[y+1][(x-1)+i] = 1

detectedSwamp(5,5)
print("New Map")
print(map)