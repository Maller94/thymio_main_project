from bisect import bisect_left
import numpy as np 


# robot position = 11

# create map
map = np.zeros(17*11).reshape(11,17)

#create coordinate lists
xi = 0
yi = 0
x_axis_values = [0]*17
y_axis_values = [0]*11

for i in range(len(x_axis_values)):
    x_axis_values[i] = xi
    xi += 120
for i in range(len(y_axis_values)):
    y_axis_values[i] = yi 
    yi += 113


# update map with robot's position
def robotPos(x,y): 
    old = tuple(np.argwhere(map == 11))
    try: 
        map[old[0][0]][old[0][1]] = 0
    except: 
        pass
    map[return_posY(y)][return_posX(x)] = 11
    return map
    

# update map with robot's position and setting that to be swamp 
def detectedSwamp(x,y, o): 
    belief(return_posX(x), return_posY(y), o)
    map[return_posY(y)][return_posX(x)] = 2
    return map

# function to update surrounding boxes to belief system
def belief(x,y, o): 
    # if robot is looking left
    if o == "L" or o == "UL" or o == "DL":
        for i in range(3): 
            map[(y-1)+i][x] = 1
            map[(y-1)+i][x-1] = 1
    # if robot is looking up
    if o == "U":
        for i in range(3): 
            map[y][(x-1)+i] = 1
            map[y-1][(x-1)+i] = 1
    # if robot is looking right
    if o == "R" or o == "UR" or o == "DR":
        for i in range(3): 
            map[(y-1)+i][x] = 1
            map[(y-1)+i][x+1] = 1
    # if robot is looking down
    if o == "D":
        for i in range(3): 
            map[y][(x-1)+i] = 1
            map[y+1][(x-1)+i] = 1

def deleteSwamp(x,y): 
    map[return_posY(y)][return_posX(x)] = 0
    return map


# returns closest number to values in list - use this to return index/x_coord
def take_closest(myList, myNumber):
    # If two numbers are equally close, return the smallest number.
    pos = bisect_left(myList, myNumber)
    if pos == 0:
        return myList[0]
    if pos == len(myList):
        return myList[-1]
    before = myList[pos - 1]
    after = myList[pos]
    if after - myNumber < myNumber - before:
        return after
    else:
        return before

# both functions returns the closes index, (x,y) coordinate, for the given number
def return_posX(number): 
    return x_axis_values.index(take_closest(x_axis_values,number))

def return_posY(number): 
    return y_axis_values.index(take_closest(y_axis_values,number))

# print map 
def printMap():
    print(map)
