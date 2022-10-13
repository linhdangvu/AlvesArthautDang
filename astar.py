import csv
import math
from os import popen
import numpy as np
# import matplotlib.pyplot as plt
from time import sleep

LEFT = (0,1)
RIGHT = (0,-1)
DOWN = (-1,0)
UP = (1,0)


# ##### PROF #####

# def verify(maze, start, end, path):
#     if path[0][0]!=start[0] or path[0][1]!=start[1] or path[-1][0]!=end[0] or path[-1][1]!=end[1]:
#         print('Start or end incorrect')
#         return False
#     last = None
#     for pos in path:
#         if maze[pos] == 1:
#             print('Path cross a wall')
#             return False
#         if last==None:
#             last = pos
#             continue
#         diffX = abs(pos[0] - last[0])
#         diffY = abs(pos[1] - last[1])
#         if diffX > 1 or diffY > 1 or diffX + diffY > 1:
#             print('Path not consecutive')
#             return False
#         last = pos
#     print('Correct path')
#     return True



# def displayMatrix(mat):
#     plt.matshow(mat)
#     plt.show()

# def displayPath(mat, path):
#     if path != None:
#         for (x, y) in path:
#             mat[x, y] = math.inf
#     plt.matshow(mat)
#     plt.show()

##### ASTAR #####
# create Node
class Node():
    def __init__(self, coord, dist, origin):
        self.coord = coord
        self.dist = dist
        self.origin = origin
    def getX(self):
        return self.coord[0]
    def getY(self):
        return self.coord[1]
    def __repr__(self):
        return "Point ({},{}) - Distance: {}".format(self.getX(), self.getY(), self.dist)

def reconstituesChemin(current):
    path = []
    position = []
    current_node = current
    while current_node is not None:
        point_current = (current_node.getX(), current_node.getY())
        path.append(point_current)
        current_node = current_node.origin
    reversed_path = path[::-1]
    print("Path: {}".format(reversed_path))
    return reversed_path

def validNeighbor(next_pos, maze):
    maze_col_size = len(maze) - 1
    maze_row_size = len(maze[maze_col_size]) - 1
    x = next_pos[0]
    y = next_pos[1]
    return y <= maze_col_size and x >= 0 and x <= maze_row_size and y >= 0 and maze[x][y] == 0
    # pass

def getMinDistance(list):
    min = math.inf
    for i in list:
        if(min > i.dist):
            min = i.dist
    return min

def isInChildren(child, children):
    for i in children:
        if i.getX() == child.getX() and i.getY() == child.getY():
            return True
    return False
    
def isInIndex(maze, start, end):
    row = len(maze)
    col = len(maze[len(maze) -1])
    return start[0] < row and start[1] < col and end[0] < row and end[1] < col and maze[start[0]][start[1]] != 1 and maze[end[0]][end[1]] != 1 and maze[start[0]][start[1]] != 2 and maze[end[0]][end[1]] != 2

def astar(maze, start, end):
    ## TODO: Implement A start algorithm
    if not isInIndex(maze, start, end):
        print("Index is out of dimension [{},{}]".format(len(maze)-1, len(maze[len(maze)-1])-1 ))
        return [(0,0)]
    else:
        print("Index is in of dimension [{},{}]".format(len(maze)-1, len(maze[len(maze)-1])-1 ))
        st_node = Node (start, 0, None)
        end_node = Node (end, math.inf, None)
        closed_list = []
        opened_list = [st_node]
        while len(opened_list) > 0:
            current = opened_list.pop(0)
            if current.getX() == end_node.getX() and current.getY() == end_node.getY():
                return reconstituesChemin(current)
            children = []
            # check if there are no neighbor
            for pos in [UP, RIGHT, DOWN, LEFT]: # (-1,0) => UP, (0,1) => RIGHT, (0,-1) => LEFT, (1,0) => DOWN
                next_pos = (current.getX() + pos[0],current.getY() + pos[1])
                if not validNeighbor(next_pos, maze):
                    continue # stop the for loop
                child = Node (next_pos, current.dist+1, current)
                children.append(child)

            closed_list.append(current)
            for child in children:
                if isInChildren(child, closed_list) or (isInChildren(child, opened_list) and child.dist > getMinDistance(opened_list)): # compare avec open list, not children
                    continue
                opened_list.append(child)

##### CSV #####
def convertPolarToCatesian(angle, dist):
    theta = angle * math.pi/180.0
    x = dist * math.cos(theta)
    y = dist * math.sin(theta)
    return (x, y)

#csv
def readCSV(filename):
    path = []
    with open(filename, 'r') as file:
        csvreader = csv.reader(file, delimiter=';')
        for row in csvreader:
            x, y = convertPolarToCatesian(int(float(row[0])), int(float(row[1]))) 
            if  0 < -x < 2000 and -1000 < y < 1000:
                path.append((-x,y+1000))       
    return path

def dataLidar(path): # angle dist => lidar
    lidar_data = []
    for row in path:
        x, y = convertPolarToCatesian(int(float(row[0])), int(float(row[1]))) 
        if  0 < -x < 2000 and -1000 < y < 1000:
            lidar_data.append((-x,y+1000))
    return lidar_data

# def displayCSV(path):
#     if path != None:
#         xs = [float(x[0]) for x in path]
#         ys = [float(x[1]) for x in path]
#         #plt.plot(xs, ys)
#         plt.clf()
#         plt.scatter(xs, ys)
#         plt.ylim(0, 2000)
#         plt.xlim(0, 2000)
#         plt.grid()
#         plt.pause(.1)
#     plt.show()

def increaseObstacle(mat):
    for i in range(len(mat)):
        for j in range(len(mat[i])):
            #print("{},{}".format(i,j))
            if mat[i][j] == 1:
                #print("{},{}".format(i,j))
                # print("{},{}".format(i,j))
                if mat[i][j-1] != 1:
                    mat[i][j-1] = 2.
                if j < (len(mat[i])-1):
                    if mat[i][j+1] != 1:
                        mat[i][j+1] = 2.
    return mat

def hasObstacleBefore(mat, start):
    startX = start[0]
    startY = start[1]
    return mat[startX][startY] == 1 or mat[startX][startY]==2 or mat[startX][startY+1]==1 or mat[startX][startY+1]==2 or mat[startX][startY-1]==1 or mat[startX][startY-1]==2

def addZero(mat):
    row_mat = len(mat[len(mat)-1])
    zero_list = np.asarray([[0.]*row_mat])
    new_mat = np.concatenate((zero_list, mat), axis=0)
    return new_mat

def convertToMatrix(path):
    matrix = np.asarray([[0.]*10]*10) 
    start = (0,5)
    for i in path:
        x = int(i[0]/200) # 10 cases => 0 -> 9
        y = int(i[1]/200) # 10 cases => 0 -> 9
        matrix[x][y] = 1.
    if hasObstacleBefore(matrix, start):
        matrix = addZero(matrix)
    matrix = increaseObstacle(matrix)
    # print(matrix)
    return matrix

##### ROBOT #####
def sendInstruction(char, ser):
    ser.write(char.encode())

# movement robot
def goUp(ser): # or move forward
    sleep(0.1)
    for i in range(5):
        sleep(0.1)
        sendInstruction('z', ser)

def goDown(ser):
    sleep(0.1)
    for i in range(5):
        sleep(0.1)
        sendInstruction('s', ser)

def goLeft(ser):
    sleep(0.1)
    for i in range(15):
        sleep(0.1)
        sendInstruction('q', ser)

def goRight(ser):
    sleep(0.1)
    for i in range(15):
        sleep(0.1)
        sendInstruction('d', ser)

def pathRobot(pathAstar):
    path = []
    for (index, item) in enumerate(pathAstar):
        # print("{} et {}".format(index, item))
        if index < 1:
            continue
        pos = (item[0] - pathAstar[index-1][0] ,item[1] - pathAstar[index-1][1])
        path.append(pos)
    return path

def solutionRobot(pathAstar, ser):
    path_robot = pathRobot(pathAstar)
    # print("Path robot: {}".format(path_robot))
    for (index, pos) in enumerate(path_robot[0:1]):
        # print(1/(index+1))
        sleep(0)
        if pos == DOWN: 
            goDown(ser)
            print("Go down")      
        if pos == UP:
            goUp(ser)
            print("Go up")
        if pos == LEFT:
            goLeft(ser)
            print("Go left")
        if pos == RIGHT:
            goRight(ser)
            print("Go right")

##### LIDAR + ROBOT
def runLidarWithRobot(point, ser):
    # transform Lidar to matrix and create A star
    pathLidar = dataLidar(point)
    matrix0 = convertToMatrix(pathLidar)
    start = (0, 5)
    end = (9, 5)
    print(matrix0)
    pathAstar = astar(matrix0, start, end)
    print("NEW Astar path: {}".format(pathAstar))
    solutionRobot(pathAstar, ser)

##################



##### TEST #####
# ## MATRIX 1
# matrix = np.matrix([[0., 0., 0., 0., 0.], 
#                     [0., 0., 1., 0., 0.], 
#                     [0., 1., 1., 0., 0.], 
#                     [0., 0., 1., 0., 0.], 
#                     [0., 0., 0., 0., 0.]])


# displayMatrix(matrix)
# maze = np.asarray(matrix)
# start = (2 ,0)
# end = (3, 4)

# path = astar(maze, start, end)
# displayPath(matrix, path)
# print(verify(maze, start, end, path))


# ## MATRIX 2
# matrix = np.matrix([[1.,1.,1.,0.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.],
#                     [1.,0.,0.,0.,1.,0.,0.,0.,0.,0.,1.,0.,0.,0.,1.,0.,1.,0.,0.,0.,1.],
#                     [1.,0.,1.,0.,1.,1.,1.,0.,1.,1.,1.,0.,1.,0.,1.,0.,1.,0.,1.,0.,1.],
#                     [1.,0.,1.,0.,0.,0.,0.,0.,1.,0.,1.,0.,1.,0.,1.,0.,1.,0.,1.,0.,1.],
#                     [1.,0.,1.,1.,1.,0.,1.,1.,1.,0.,1.,0.,1.,1.,1.,0.,1.,0.,1.,0.,1.],
#                     [1.,0.,0.,0.,1.,0.,1.,0.,1.,0.,1.,0.,0.,0.,1.,0.,0.,0.,1.,0.,1.],
#                     [1.,1.,1.,0.,1.,1.,1.,0.,1.,0.,1.,1.,1.,0.,1.,1.,1.,1.,1.,0.,1.],
#                     [1.,0.,1.,0.,0.,0.,0.,0.,1.,0.,0.,0.,1.,0.,0.,0.,1.,0.,0.,0.,1.],
#                     [1.,0.,1.,1.,1.,0.,1.,1.,1.,1.,1.,0.,1.,1.,1.,0.,1.,1.,1.,0.,1.],
#                     [1.,0.,0.,0.,0.,0.,0.,0.,1.,0.,0.,0.,1.,0.,1.,0.,1.,0.,0.,0.,1.],
#                     [1.,0.,1.,1.,1.,1.,1.,0.,1.,0.,1.,1.,1.,0.,1.,0.,1.,1.,1.,0.,1.],
#                     [1.,0.,1.,0.,1.,0.,1.,0.,0.,0.,1.,0.,1.,0.,1.,0.,0.,0.,0.,0.,1.],
#                     [1.,1.,1.,0.,1.,0.,1.,1.,1.,0.,1.,0.,1.,0.,1.,0.,1.,0.,1.,1.,1.],
#                     [1.,0.,1.,0.,0.,0.,0.,0.,0.,0.,1.,0.,0.,0.,1.,0.,1.,0.,1.,0.,1.],
#                     [1.,0.,1.,1.,1.,1.,1.,1.,1.,0.,1.,1.,1.,0.,1.,1.,1.,0.,1.,0.,1.],
#                     [1.,0.,1.,0.,1.,0.,0.,0.,0.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.,0.,1.],
#                     [1.,0.,1.,0.,1.,1.,1.,0.,1.,0.,1.,1.,1.,0.,1.,1.,1.,0.,1.,0.,1.],
#                     [1.,0.,1.,0.,0.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.,0.,1.,0.,1.,0.,1.],
#                     [1.,0.,1.,0.,1.,0.,1.,0.,1.,1.,1.,1.,1.,1.,1.,0.,1.,0.,1.,1.,1.],
#                     [1.,0.,0.,0.,1.,0.,1.,0.,0.,0.,0.,0.,0.,0.,1.,0.,1.,0.,0.,0.,0.],
#                     [1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.]])

# start = (0, 3)
# end = (19, 20)
# displayMatrix(matrix)
# maze = np.asarray(matrix)

# path = astar(maze, start, end)
# displayPath(matrix, path)
# print(verify(maze, start, end, path))

##### TEST CSV #####

# ### data 0
# start = (0, 5)
# end = (11, 5)
# path0 = readCSV('../csv/data0.csv')
# # displayCSV(path0)
# matrix0 = convertToMatrix(path0)
# print(matrix0)
# # displayMatrix(matrix0)
# path = astar(matrix0, start, end)
# # displayPath(matrix0, path)
# # print(verify(matrix0, start, end, path))

# start = (0, 10)
# end = (19, 11)
# rotate_1 = rotateLeftMatrix(matrix0)
# print(rotate_1)
# displayMatrix(rotate_1)
# path = astar(rotate_1, start, end)
# displayPath(rotate_1, path)
# print(verify(rotate_1, start, end, path))

# #### data 1
# start = (0, 5)
# end = (10, 5)
# path1 = readCSV('../csv/data1.csv')
# # displayCSV(path1)
# matrix1 = convertToMatrix(path1)
# print(matrix1)
# # displayMatrix(matrix1)
# path = astar(matrix1, start, end)
# # displayPath(matrix1, path)
# # print(verify(matrix1, start, end, path))


# #### data 2
# start = (0, 5)
# end = (9, 5)
# path2 = readCSV('../csv/data2.csv')
# # displayCSV(path2)
# matrix2 = convertToMatrix(path2)
# print(matrix2)
# # displayMatrix(matrix2)
# path = astar(matrix2, start, end)
# # displayPath(matrix2, path)
# # print(verify(matrix2, start, end, path))


# #### data 3
# start = (0, 5)
# end = (9, 5)
# path3 = readCSV('../csv/data3.csv')
# # displayCSV(path3)
# matrix3 = convertToMatrix(path3)
# print(matrix3)
# # displayMatrix(matrix3)
# path = astar(matrix3, start, end)
# # displayPath(matrix3, path)
# # print(verify(matrix3, start, end, path))


# #### data 4
# start = (0, 10)
# end = (19, 10)
# path4 = readCSV('../csv/data4.csv')
# displayCSV(path4)
# matrix4 = convertToMatrix(path4)
# displayMatrix(matrix4)
# path = astar(matrix4, start, end)
# displayPath(matrix4, path)
# print(verify(matrix4, start, end, path))


# #### data 5
# start = (0, 5)
# end = (9, 5)
# path5 = readCSV('../csv/data5.csv')
# # displayCSV(path5)
# matrix5 = convertToMatrix(path5) 
# # rotate_1 = rotateLeftMatrix(matrix5)
# # rotate_2 = rotateLeftMatrix(rotate_1)
# # rotate_3 = rotateLeftMatrix(rotate_2)
# displayMatrix(matrix5)
# # displayMatrix(rotate_1)
# # displayMatrix(rotate_2)
# # displayMatrix(rotate_3)
# path = astar(matrix5, start, end)
# displayPath(matrix5, path)
# print(verify(matrix5, start, end, path))


# #### TEST Lidar
# matrix = np.matrix([[0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,],
#  [0., 0., 0., 0., 1., 1., 0., 0., 0., 0.,],
#  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,],
#  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,],
#  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,],
#  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,],
#  [0., 0., 0., 0., 0., 0., 0., 0., 0., 1.,],
#  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,],
#  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,],
#  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,]])


# displayMatrix(matrix)
# maze = np.asarray(matrix)
# start = (0, 5)
# end = (9, 5)

# path = astar(maze, start, end)
# displayPath(matrix, path)
# print(verify(maze, start, end, path))