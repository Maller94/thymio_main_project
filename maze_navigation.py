import queue
import numpy as np

class Maze:
    def __init__(self, maze):
        self.maze = maze

    def convertMaze(self,maze, path=""):
        posArray = tuple(np.argwhere(maze == 1.))
        j = posArray[0][0]
        i = posArray[0][1]

        mapInner = np.array(maze)
        l1 = []

        pos = set()
        for move in path:
            if move == "L":
                i -= 1

            elif move == "R":
                i += 1

            elif move == "U":
                j -= 1

            elif move == "D":
                j += 1
            pos.add((j, i))
        
        for j, row in enumerate(maze):
            for i, col in enumerate(row):
                if (j, i) in pos:
                    #print(' + ', end="")
                    mapInner[j][i] = 4
                    l1.append((j,i))
                #else:
                    #print(str(col), end="")
        print(l1)
        return mapInner

    def valid(self,maze, moves):
        posArray = tuple(np.argwhere(maze == 1.))
        j = posArray[0][0]
        i = posArray[0][1]

        for move in moves:
            if move == "L":
                i -= 1

            elif move == "R":
                i += 1

            elif move == "U":
                j -= 1

            elif move == "D":
                j += 1

            if not(0 <= i < len(maze[0]) and 0 <= j < len(maze)):
                return False
            elif (maze[j][i] == 3.):
                return False

        return True

    def findEnd(self,maze, moves):
        posArray = tuple(np.argwhere(maze == 1.))
        j = posArray[0][0]
        i = posArray[0][1]

        for move in moves:
            if move == "L":
                i -= 1

            elif move == "R":
                i += 1

            elif move == "U":
                j -= 1

            elif move == "D":
                j += 1

        if maze[j][i] == 5:
            print("Found: " + moves)
            #self.convertMaze(maze, moves)
            return moves

        return False

    def createPlan(self):
        nums = queue.Queue()
        nums.put("")
        moves_list = ""

        while not self.findEnd(maze, moves_list): 
            moves_list = nums.get()
            for j in ["L", "R", "U", "D"]:
                put = moves_list + j
                if self.valid(maze, put):
                    nums.put(put)
        return moves_list

    def getCoordinates(self):
        moves = self.createPlan()
        mapInner = self.convertMaze(maze,moves)
        #return l1
        


if __name__ == "__main__":
    maze = []
    maze.append([0.,0., 0., 0., 0., 1., 0.])
    maze.append([0.,0., 3., 3., 3., 3., 3.])
    maze.append([0.,0., 0., 0., 0., 3., 0.])
    maze.append([3.,3., 3., 3., 0., 3., 0.])
    maze.append([5.,0., 0., 0., 0., 0., 0.])
    maze = np.array(maze)

    mazeObj = Maze(maze)
    print(mazeObj.getCoordinates())
    