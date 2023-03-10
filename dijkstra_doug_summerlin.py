## ========================================
# ENPM661 Spring 2023: Robotic Path Planning 
# Project #2 
# Maze Search with Obstacles with Dijkstra's Algorithm
#
# Author: Doug Summerlin (dsumm1001@gmail.com, dsummerl@umd.edu)
# UID: 114760753
# Directory ID: dsummerl
# ========================================
# Run as 'python3 Dij_maze_search.py'
# Press ESC for exit

import numpy as np
import matplotlib.pyplot as plt
import cv2
import math
from queue import PriorityQueue
import time
# from collections import deque

def getValidInput(type, maze):
    thresh = 5
    while True:
        try:
            coordInput = input("Enter " + type + " node coordinates in x, y format, separated by a comma: ")
            coords = tuple(int(item) for item in coordInput.split(','))
        except ValueError:
            print("Sorry, results invalid. Please try again, entering two integer inputs between 5-595 and 5-245, respectively. ")
            continue
        if coords[0] < 0 + thresh or coords[0] > 600 - thresh or coords[1] < 0 + thresh or coords[1] > 250 - thresh:
            print("Sorry, results invalid. Please try again, entering two integer inputs between 5-595 and 5-245, respectively. ")
            continue
        if checkObstacle(coords, maze) == True:
            print("Sorry, results invalid. Please try again, making sure to not place the start or goal in an obstacle space.")
            continue
        else:
            break
    return coords
    
def checkObstacle(xyCoords, maze):
    if all(maze[xyCoords[1], xyCoords[0]] == [255,255,255]):
        return False
    else:
        return True

def actMoveRight(curr, maze):
    if (checkObstacle((curr[0]+1,curr[1]), maze) == False):
        newRight = (1, None, (curr[0]+1, curr[1]))
        return newRight
    else:
        return None
    
def actMoveLeft(curr, maze):
    if (checkObstacle((curr[0]-1,curr[1]), maze) == False):
        newLeft = (1, None, (curr[0]-1, curr[1]))
        return newLeft
    else:
        return None
    
def actMoveUp(curr, maze):
    if (checkObstacle((curr[0],curr[1]+1), maze) == False):
        newUp = (1, None, (curr[0], curr[1]+1))
        return newUp
    else:
        return None
    
def actMoveDown(curr, maze):
    if (checkObstacle((curr[0],curr[1]-1), maze) == False):
        newDown = (1, None, (curr[0], curr[1]-1))
        return newDown
    else:
        return None
    
def actMoveUpRight(curr, maze):
    if (checkObstacle((curr[0]+1,curr[1]+1), maze) == False):
        newUpRight = (1.4, None, (curr[0]+1, curr[1]+1))
        return newUpRight
    else:
        return None
    
def actMoveUpLeft(curr, maze):
    if (checkObstacle((curr[0]-1,curr[1]+1), maze) == False):
        newUpLeft = (1.4, None, (curr[0]-1, curr[1]+1))
        return newUpLeft
    else:
        return None
    
def actMoveDownRight(curr, maze):
    if (checkObstacle((curr[0]+1,curr[1]-1), maze) == False):
        newDownRight = (1.4, None, (curr[0]+1, curr[1]-1))
        return newDownRight
    else:
        return None
    
def actMoveDownLeft(curr, maze):
    if (checkObstacle((curr[0]-1,curr[1]-1), maze) == False):
        newDownLeft = (1.4, None, (curr[0]-1, curr[1]-1))
        return newDownLeft
    else:
        return None

def searchNode(nodeCoords, maze):
    right = actMoveRight(nodeCoords, maze)
    left = actMoveLeft(nodeCoords, maze)
    up = actMoveUp(nodeCoords, maze)
    down = actMoveDown(nodeCoords, maze)
    upRight = actMoveUpRight(nodeCoords, maze)
    upLeft = actMoveUpLeft(nodeCoords, maze)
    downRight = actMoveDownRight(nodeCoords, maze)
    downLeft = actMoveDownLeft(nodeCoords, maze)

    results = []

    if right is not None:
        results.append(right)
    if left is not None:
        results.append(left)
    if up is not None:
        results.append(up)
    if down is not None:
        results.append(down)
    if upRight is not None:
        results.append(upRight)
    if upLeft is not None:
        results.append(upLeft)
    if downRight is not None:
        results.append(downRight)
    if downLeft is not None:
        results.append(downLeft)

    #print("results", results)
    return results

def generatePath(nodeIndex, nodeCoords, maze):
    pathIndices = []
    pathCoords = []

    while nodeIndex is not None:
        pathIndices.append(nodeIndex)
        pathCoords.append(nodeCoords)
        cv2.circle(maze, (int(nodeCoords[0]),int(nodeCoords[1])), 1, color=(0,255,255), thickness=-1)
        nodeCoords = valDict[nodeIndex]
        nodeIndex = parentDict[nodeIndex]
        outVid.write(cv2.flip(maze,0))

    return pathIndices, pathCoords

#================================================================================================================
#================================================================================================================
#================================================================================================================

print("\nWelcome to the Dijkstra Maze Finder Program! \n")

mazeSize = (250,600)

outVid = cv2.VideoWriter('output.avi',cv2.VideoWriter_fourcc(*'MJPG'), 60, (600,250))

# Create blank maze
maze = np.zeros((mazeSize[0], mazeSize[1], 3), dtype = np.uint8)
maze[:] = (0, 255, 0)
cv2.rectangle(maze, pt1=(5,5), pt2=(595,245), color=(255,255,255), thickness= -1)

# draw rectangle obstacles
cv2.rectangle(maze, pt1=(95,0), pt2=(155,105), color=(0,255,0), thickness= -1)
cv2.rectangle(maze, pt1=(95,145), pt2=(155,mazeSize[1]), color=(0,255,0), thickness= -1)

cv2.rectangle(maze, pt1=(100,0), pt2=(150,100), color=(0,0,255), thickness= -1)
cv2.rectangle(maze, pt1=(100,150), pt2=(150,mazeSize[1]), color=(0,0,255), thickness= -1)

# draw hexagonal obstacle and boundary
hexBoundPts = np.array([[300, 44], [370, 84],
                         [370, 166], [300, 206], 
                         [230, 166], [230,84]])
cv2.fillConvexPoly(maze, hexBoundPts, color=(0, 255, 0))

hexPts = np.array([[300, 125-75], [365, math.ceil(125-37.5)],
                         [365, math.ceil(125+37.5)], [300, 125+75], 
                         [235, math.ceil(125+37.5)], [235, math.ceil(125-37.5)]])
cv2.fillConvexPoly(maze, hexPts, color=(0, 0, 255))

# draw triangular obstacle
cv2.circle(maze, [460, 25], 5, color=(0, 255, 0), thickness=-1)
cv2.circle(maze, [460, 225], 5, color=(0, 255, 0), thickness=-1)
cv2.circle(maze, [510, 125], 5, color=(0, 255, 0), thickness=-1)

cv2.rectangle(maze, pt1=(455,25), pt2=(460,225), color=(0,255,0), thickness= -1)
triUpperBoundPts = np.array([[460, 25], [465, 22], [516, 125], [510, 125]])
cv2.fillConvexPoly(maze, triUpperBoundPts, color=(0, 255, 0))
triLowerBoundPts = np.array([[510, 125], [516, 125], [465, 228], [460, 225]])
cv2.fillConvexPoly(maze, triLowerBoundPts, color=(0, 255, 0))

triPts = np.array([[460, 25], [460, 225], [510, 125]])
cv2.fillConvexPoly(maze, triPts, color=(0, 0, 255))

# get start and goal nodes
start = getValidInput("start", maze)
goal = getValidInput("goal", maze)

startTime = time.time()

solved = False
openList = PriorityQueue()
#closedList = []

parentDict = {1: None}
valDict = {1: start}
closedDict = {}
openDict = {1: start}

startNode = (0, 1, start)
index = startNode[1]
print('github test')

openList.put(startNode)

while not openList.empty() and solved == False:
    first = openList.get()
    del openDict[first[1]]
    #closedList.append(first)
    closedDict[first[1]] = first[2]

    if ((first[2] == goal)):
        elapsedTime = time.time() - startTime
        print ("Yay! Goal node located... Operation took ", elapsedTime, " seconds.")
        print("Current node index: ", first[1], " and cost: ", first[0])
        solved = True

        index, coords = generatePath(first[1], first[2], maze)
        # pathNodes.reverse()
        # pathStates.reverse()
        break

    results = searchNode(first[2], maze)
 
    for i in results:
        if not i[2] in closedDict.values():
        #if not any(i[2] in j for j in closedList):
            maze[i[2][1], i[2][0]] = (255,0,0)
            if not i[2] in openDict.values():
            #if not any(i[2] in k for k in openList.queue):
                temp = list(i)
                index += 1
                temp[1] = index
                temp[0] = round(i[0] + first[0], 2)
                parentDict[temp[1]] = first[1]
                valDict[temp[1]] = temp[2]
                i = tuple(temp)
                openList.put(i)
                openDict[i[1]] = i[2]
        else:
            if i[0] > first[0] + 1:
                temp = list(i)
                temp[0] = i[0] + first[0]
                i = tuple(list)

    outVid.write(cv2.flip(maze,0))

    # print("No. of visited nodes: ", closedList.qsize(), 
    #       "No. of identified nodes: ", index,  
    #       "Length of Stack: ", openList.qsize(), "\n")

    # iter = input("Iterate?")
    # print()

if solved == False:
    print ("Failure! Goal node not found")

# display the image using opencv
maze = cv2.flip(maze, 0)
#maze = cv2.resize(maze, (1800,750), interpolation = cv2.INTER_AREA)
cv2.imshow('Maze', maze)
cv2.waitKey(0)

outVid.release()
cv2.destroyAllWindows()

# Resources