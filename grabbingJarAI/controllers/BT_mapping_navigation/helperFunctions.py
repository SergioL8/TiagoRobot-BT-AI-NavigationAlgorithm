import numpy as np


def world2map(xw, yw):
    # Hard-coded boundaries of the world coordinates
    x_min, x_max = 2.18, -2.28
    y_min, y_max = -3.9, 1.783
 

    # Normalize xw and yw into the range [0, 299]
    px = int(((xw - x_min) / (x_max - x_min)) * 299)
    py = int(((yw - y_min) / (y_max - y_min)) * 299)
    
    # Ensure the pixel values stay within the display's [0, 299] range
    px = min(max(px, 0), 299)
    py = min(max(py, 0), 299)
    
    return px, py
    
    
    
    
def map2world(px, py):
    # Hard-coded boundaries of the world coordinates
    x_min, x_max = 2.18, -2.28
    y_min, y_max = -3.9, 1.783

    # Ensure the pixel values are within the valid range
    px = min(max(px, 0), 299)
    py = min(max(py, 0), 299)

    # Convert px and py back to world coordinates
    xw = x_min + (px / 299) * (x_max - x_min)
    yw = y_min + (py / 299) * (y_max - y_min)

    return xw, yw
    
    
    
    
    
def getNeightbors(u, goal, map):
    '''
    Function that returns the neighbors of a given position in a map
    Input: u (starting point), goal (overal desitation), map (2d array representing the map)
    Output: neighbors of the given point u with the cost with respect to the goal
    '''

    neighbors = [] # declare the variable that will be returned
    for delta in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)): # traverse in the 8 directions
        cand = (u[0] + delta[0], u[1] + delta[1]) # create the new candidate by moving in the selected direction
        
        # check that the candidate is inside the map
        if 0 <= cand[0] and cand[0] < len(map) and 0 <= cand[1] and cand[1] < len(map[0]) and map[cand[0]][cand[1]] < 0.3:
            cost = np.sqrt((goal[0]-cand[0])**2+(goal[1]-cand[1])**2) # compute new square
            neighbors.append((cand, cost)) # add the neighbor to the list of neigbors
    return neighbors
    
    

    
    
    
    
    
    
    
    
    
