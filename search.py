from collections import deque as queue
import copy
import numpy
from queue import PriorityQueue
import math
import heapq
# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node



# Function to check if mat[row][col]
# is unvisited and lies within the
# boundary of the given matrix

def isValid(row,col,grid, vis):
    #Function to see if the current cell is out of bounds or not
    #To see the bounds of the grid
    ROW = len(grid)
    COL = len(grid[0])

    # Direction vectors
    dRow = [1, 0, 0, -1]
    dCol = [0, 1, -1, 0]

    # If cell is out of bounds
    if (row < 0 or col < 0 or row >= ROW or col >= COL):
        return False

    # If the cell is already visited
    if (vis[(row * ROW) + col]):
        return False

    # Otherwise, it can be visited
    return True


def BFSsolve(grid, start, goal, vis, prev):
    #This function does most of the computation part for the BFS algorithm
    row = 0
    col = 0
    ROW = len(grid)
    COL = len(grid[0])

    # Direction vectors
    dRow = [1, 0, 0, -1]
    dCol = [0, 1, -1, 0]

    stepsCount = 0
    # Stores indices of the matrix cells
    q = queue()

    # Mark the starting cell as visited
    # and push it into the queue
    q.append((row, col))
    vis[(row * ROW) + col] = True
    # Iterate while the queue
    # is not empty
    while (len(q) > 0):
        stepsCount = stepsCount + 1
        cell = q.popleft()
        x = cell[0]
        y = cell[1]
        grid[x][y] = 2 #mark 2 for every visted node. obstacles still stay as 1

        if (x == goal[0] and y == goal[1]):
            break
        # Go to the adjacent cells
        for i in range(4):
            adjx = x + dRow[i]
            adjy = y + dCol[i]
            if (isValid(adjx, adjy, grid, vis) and grid[adjx][adjy] != 1):
                q.append((adjx, adjy))
                vis[(adjx * ROW) + adjy] = True
                prev[(adjx * ROW) + adjy] = (x * ROW) + y
    return prev, stepsCount
def minDist(dist,q,vis,grid):
    ROW=len(grid)
    min  = numpy.inf
    for i,j in q:
        # print(i,j)
        if dist[i][j]<min and vis[(i*ROW)+j]!=True:
            min = dist[i][j]
            minIndexi = i
            minIndexj = j
    return minIndexi, minIndexj

# shortest path algorithm for a graph represented
def dijkstraSolve(grid,start,goal,vis,prev,dist):
    #This function does most of the computation for the dijkstras algorithm
    srow=start[0]
    scol=start[1]
    stepsCount = 0
    dist[srow][scol] = 0  # Hardcoded source cell as (0,0)
    # print(dist)
    # q = queue()
    # q.append((srow, scol))
    ROW = len(grid)
    COL = len(grid[0])
    dRow = [1, 0, 0, -1]
    dCol = [0, 1, -1, 0]
    q = PriorityQueue()
    q.put((dist[srow][scol], (srow, scol)))
    # Iterate while the queue
    # is not empty
    while not q.empty():
        stepsCount = stepsCount + 1

        gotx = q.get()

        minx = gotx[1][0]
        miny = gotx[1][1]
        vis[(minx * ROW) + miny] = True
        grid[minx][miny] = 2
        #checking to see if goal
        if (minx == goal[0] and miny == goal[1]):
            break
        # q.popleft()
        for i in range(4):
            adjx = minx + dRow[i]
            adjy = miny + dCol[i]

            if (isValid(adjx, adjy,grid,vis) and grid[adjx][adjy] != 1 and dist[adjx][adjy] > dist[minx][miny] + 1):
                dist[adjx][adjy] = (dist[minx][miny] + 1)
                # f[adjx][adjy]= dist[adjx,adjy] + heuristic(grid,start,goal)
                #                 print(str(adjx)+" "+str(adjy)+" "+str(dist[adjx][adjy]))
                q.put((dist[adjx][adjy], (adjx, adjy)))
                prev[(adjx * ROW) + adjy] = (minx * ROW) + miny
    return dist, prev, stepsCount
def adjustGrid(grid):
    #Because we used to mark 2 as the goal, the grid became immutaable, marking 
    #certain free nodes as obstacles from the previous algorithm. This provides a fix for that
    for row in range(len(grid)):
        for col in range(len(grid[0])):
            if(grid[row][col]==2):
                grid[row][col]=0

def reconstructPath(allPaths, grid, goal):
    #This function reconstructs the paths for BFS and Dijkstras.
    ROW = len(grid)
    COL = len(grid[0])
    finalPath = []
    optpath = []
    i = goal[0] * len(grid) + goal[1]
    finalPath.append(i)
    # print(i)
    while (i != False):
        i = allPaths[i]
        finalPath.append(i)
    finalPath.reverse()
    if (finalPath[0] == 0):
        for node in finalPath:
            x = int(numpy.floor(node / ROW))
            y = int(node - (x * ROW))
            optpath.append([x, y])
            grid[x][y] = 3
        adjustGrid(grid)
        return optpath
    adjustGrid(grid)
    return []

def make_graph(total_row, total_col, grid, initial):
    #This is used to make the graph of the whole grid.
    graph = []
    for m in range(total_row):
        nodes = []
        for n in range(total_col):
            curr = Node(m, n, False, False)
            #obtaining the start position
            if curr.row == initial.row and curr.col == initial.col:
                curr.g = 0
                curr.cost = 0
            else:
                curr.g = math.inf
                curr.cost = math.inf
            # obstacle check
            if grid[m][n] > 0:
                curr.is_obs = True
            nodes.append(curr)
        graph.append(nodes)

    return graph
def is_visited(n, visited):
    #This function is used to check if a particular node is visited or not
    p = False
    for i in visited:
        if i.col == n.col and i.row == n.row:
            p = True
            break
    return p

def heuristics(graph, goal, total_rows, total_cols):
    #This function is used to find the heuristic value between a particular node to the given goal.
    for m in range(total_rows):
        for n in range(total_cols):
            row = graph[m][n].row
            col = graph[m][n].col
            graph[m][n].h = abs(row - goal.row) + abs(col - goal.col)
    return graph

def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###

    found = False
    ROW = len(grid)
    COL = len(grid[0])
    # print("entered")
    #initializing vis and prev
    vis = [False for i in range(ROW * COL)]
    prev = [False for i in range(ROW * COL)]
    #creating a deepcopy so the graph is not immutable
    gridBFS = copy.deepcopy(grid)
    allPaths, steps = BFSsolve(gridBFS, start, goal, vis, prev)
    # print(allPaths)
    # print(" steps : "+str(steps))
    path = reconstructPath(allPaths, gridBFS, goal)
    if len(path) == 0:
        found = False
    else:
        found = True
    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps

def dir_dfs(dir, graph, curr, possible_nodes, visited, total_row, total_col):
    #this function is used to find all the neighbours from a given current node
    for i in dir:
        new_row = curr.row + i[0]
        new_col = curr.col + i[1]
        poss_rows = range(total_row)
        poss_cols = range(total_col)

        if new_row in poss_rows and new_col in poss_cols:
            #if the current node is not an oject and if it has not been visited before
            if graph[new_row][new_col].is_obs == False:
                if(is_visited(graph[new_row][new_col], visited)==False):
                    #it passes through the loop and gets appended to the posssible nodes list as a possible node in the path
                    node = graph[new_row][new_col]
                    node.parent = curr
                    possible_nodes.append(node)
    return possible_nodes

def dfsPath(final_node,graph,start,path,reached):
    #This function is used to reconstruct the path that was explored using DFS algorithm
    if reached == True:
        while final_node != graph[start.row][start.col]:
            path.append([final_node.row, final_node.col])
            final_node = final_node.parent
        path.append([start.row, start.col])
        path = path[::-1]
        return path
def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###


    # Recursively tracing path to the start
    start = Node(start[0], start[1], False, False)
    goal = Node(goal[0], goal[1], False, False)
    total_row = len(grid)
    total_col = len(grid[0])

    path = []
    total_steps = 0
    reached = False
    visited = []
    #Direction vectors in the order right,down,left,up
    dir = [[0, 1], [1, 0], [0, -1], [-1, 0]]
    # Creating a graph
    graph = make_graph(total_row, total_col, grid, start)
    #Creates all the possible nodes, from the make graph function explained above
    possible_nodes = [graph[start.row][start.col]]
    #While not empty, it iterates
    while len(possible_nodes) > 0:
        curr = possible_nodes.pop()

        if is_visited(curr, visited) == True:
            continue

        total_steps = total_steps + 1

        if goal.row == curr.row and goal.col == curr.col:
            reached = True
            break

        visited.append(curr)
        #The possible nodes is obtained by passing through the dir_dfs function
        possible_nodes = dir_dfs(dir[::-1], graph, curr, possible_nodes, visited, total_row, total_col)

    final_node = curr

    # Calling the path finding function
    path=dfsPath(final_node,graph,start,path,reached)

    if reached == True:
        print(f"It takes {total_steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, total_steps


def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm
          and the number of steps it takes to find it.

       arguments:
       grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
              e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
       start - The start node in the map. e.g. [0, 0]
       goal -  The goal node in the map. e.g. [2, 2]

       return:
       path -  A nested list that represents coordinates of each step (including start and goal node),
               with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
       steps - Number of steps it takes to find the final solution,
               i.e. the number of nodes visited before finding a path (including start and goal node)

       >>> from main import load_map
       >>> grid, start, goal = load_map('test_map.csv')
       >>> dij_path, dij_steps = dijkstra(grid, start, goal)
       It takes 10 steps to find a path using Dijkstra
       >>> dij_path
       [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
       '''
    found = False
    ROW = len(grid)
    COL = len(grid[0])
    vis = [False for i in range(ROW * COL)]
    prev = [False for i in range(ROW * COL)]
    dist = [[numpy.inf for i in range(ROW)] for j in range(COL)]
    #Creating Deepcopy to make it non immutable
    gridDijk = copy.deepcopy(grid)
    dist,prev,steps = dijkstraSolve(gridDijk, start, goal, vis, prev,dist)
    # print(allPaths)
    path = reconstructPath(prev, gridDijk, goal)
    if len(path) == 0:
        found = False
    else:
        found = True

    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps

def dir_astar(dir, graph, curr, possible_nodes, visited, total_row, total_col):
    #This function is used to find all the neighours for the current node in AStar
    for i in dir:
        new_row = curr.row + i[0]
        new_col = curr.col + i[1]
        poss_rows = range(total_row)
        poss_cols = range(total_col)

        if new_row in poss_rows and new_col in poss_cols:
            if graph[new_row][new_col].is_obs == False:
                if(is_visited(graph[new_row][new_col], visited)==False):
                    node = graph[new_row][new_col]
                    node.parent = curr
                    #Calculating to come value + heuristic
                    node.g = curr.g +1
                    node.cost = node.g + node.h
                    possible_nodes.append(node)
    return possible_nodes
def AStarPath(goal_node,new_graph,start,path,reached):
    #Finds out the path traced by Astar in a recurisve method
    if reached == True:
        while goal_node != new_graph[start.row][start.col]:
            path.append([goal_node.row, goal_node.col])
            goal_node = goal_node.parent
            # print(goal_node)
        path.append([start.row, start.col])
        path = path[::-1]
        return path
def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
           and the number of steps it takes to find it.

        arguments:
        grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
               e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
        start - The start node in the map. e.g. [0, 0]
        goal -  The goal node in the map. e.g. [2, 2]

        return:
        path -  A nested list that represents coordinates of each step (including start and goal node),
                with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
        steps - Number of steps it takes to find the final solution,
                i.e. the number of nodes visited before finding a path (including start and goal node)

        >>> from main import load_map
        >>> grid, start, goal = load_map('test_map.csv')
        >>> astar_path, astar_steps = astar(grid, start, goal)
        It takes 7 steps to find a path using A*
        >>> astar_path
        [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
        '''
    ### YOUR CODE HERE ###

    path = []
    total_steps = 0
    reached = False
    visited = []

    start = Node(start[0], start[1], False, False)
    goal = Node(goal[0], goal[1], False, False)
    total_row = len(grid)
    total_col = len(grid[0])
    #Direction vectors
    dir = [[0, 1], [1, 0], [0, -1], [-1, 0]]

    # Creating a graph of all my nodes
    graph = make_graph(total_row, total_col, grid, start)
    new_graph = heuristics(graph, goal, total_row, total_col)
    possible_nodes = [new_graph[start.row][start.col]]

    while len(possible_nodes) > 0:
        possible_nodes = sorted(possible_nodes, key=lambda x: x.cost)
        curr = possible_nodes.pop(0)

        if is_visited(curr, visited) == True:
            continue

        total_steps = total_steps + 1

        if curr.row == goal.row and curr.col == goal.col:
            reached = True
            break

        visited.append(curr)

        possible_nodes = dir_astar(dir, new_graph, curr, possible_nodes, visited, total_row, total_col)

    goal_node = curr

    path=AStarPath(goal_node,new_graph,start,path,reached)

        # Printing the path and the steps taken to reach the goal
    if reached == True:
        print(f"It takes {total_steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, total_steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
