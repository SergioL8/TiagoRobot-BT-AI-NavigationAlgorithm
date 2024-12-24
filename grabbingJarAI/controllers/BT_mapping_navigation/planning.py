from helperFunctions import *
import py_trees
import numpy as np
from os.path import exists
from heapq import heappush, heappop, heapify
from collections import defaultdict




def getShortestPath(map, start, goal):
    '''
    A* algorithm that efficiently computes teh shortest path from a start to a goal in a map with obstacles.
    Input: map (2d matrix that represents a map with obstacles), start (x, y coordinates), goal (x, y coordinates)
    Output: path (list of xy coordinates where the robot has to navigate)
    '''

    # function that gets all the possible candidates to move
    def getNeightbors(u, goal):
        neighbors = [] # declare return variale
        # loop through all possible directions the robot can move
        for delta in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)): 
            cand = (u[0] + delta[0], u[1] + delta[1]) # get the new possible candidate to move
            
            # check that the candidate is inside the map
            if 0 <= cand[0] and cand[0] < len(map) and 0 <= cand[1] and cand[1] < len(map[0]) and map[cand[0]][cand[1]] < 0.3: 
                cost = np.sqrt((goal[0]-cand[0])**2+(goal[1]-cand[1])**2) # compute the new cost
                neighbors.append((cand, cost)) # append the candidate to the list of neighbors
        return neighbors
        
    # variable declaration
    queue = [(0, start)]
    heapify(queue)
    visited = {start}
    distances = defaultdict(lambda: float('inf'))
    distances[start] = 0
    parent = {}
    

    while queue: # loop until all the spots have been visited
        (currentdist, v) = heappop(queue) # pop the first element of the queue
        visited.add(v) # mark the element as visited
        
        for (u, costvu) in getNeightbors(v, goal): # loop through the neighbors of the current node
            if u not in visited: 
                newCost = distances[v] + costvu # compute the new cost
                if newCost < distances[u]: # compare the new cost to previous and update if smaller
                    distances[u] = newCost 
                    heappush(queue, (costvu, u))
                    parent[u] = v
        
        if v == goal: # if you have reaced the goal break the loop, not necessary to run through all the map
            break

    key = goal
    path = []

    # create the path with the parents list
    while key in parent.keys():
        key = parent[key]
        path.insert(0, key)

    path.append(goal) # append the goal a part of the path
    
    return path




class Planning(py_trees.behaviour.Behaviour):

    # define the initialization of the class
    def __init__(self, name, blackboard, goal):
        super(Planning, self).__init__(name)
        
        self.robot = blackboard.robot # load the robot from the blackboard
        self.blackboard = blackboard # load the blackboard
        self.goal = (goal[0], goal[1])
        
        # variable declaration for future use
        self.worldWP = []
        self.map = np.zeros((300, 300))
        
        
        
    def setup(self):
        self.logger.debug(" %s [Navigation::setup()]" %self.name)
        
        self.timestep = int(self.robot.getBasicTimeStep())# get the time step of the current world.
        
        # GPS
        self.gps =  self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

        
        # check that the file we are trying to read exists (this should always be true since we are checking in BT_mapping_navigation.py)
        file_exists = exists('cspace.npy')
        if not file_exists:
            print('File doesn not exist. Failure')
            return py_trees.common.Status.FAILURE
        
        # load the map
        self.map = np.load('cspace.npy')
    
        
    
    
    def update(self):
        print('PLANNING')
        # load the map and other variables (this is necessary in case the update has not run yet (race conditions in trees)
        self.map = np.load('cspace.npy')
        self.worldWP = []
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        
        # transform the start world coordinates into map coordinates
        start = world2map(xw, yw)

        # transform the goal into map coordinates
        self.goal = world2map(self.goal[0], self.goal[1])

        # cumpute the shortes path between the start and the goal
        waypoints = getShortestPath(self.map, start, self.goal)

        # transform the compute waypoints into world coordinates        
        for xm, ym in waypoints:
            self.worldWP.append(map2world(xm, ym))
        
        print('Mapping terminated successfully. Waypoints', self.worldWP)
        return py_trees.common.Status.SUCCESS
        
        
    def terminate(self, new_status):
        self.logger.debug(" %s [Mapping::Terminate()]" %self.name)
        self.blackboard.waypoints = self.worldWP # write in the blackboard the new waypoints
        
        
        
        
        
    
    