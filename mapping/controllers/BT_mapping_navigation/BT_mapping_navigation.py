import py_trees
from py_trees.composites import Sequence, Parallel, Selector
from py_trees.blackboard import Blackboard
from controller import Robot, Supervisor
from os.path import exists
from navigation import Navigation
from mapping import Mapping
from planning import Planning



blackboard = Blackboard() #initialize the blackboard of py_trees

robot = Supervisor()# create the Robot instance.
timestep = int(robot.getBasicTimeStep())# get the time step of the current world.
MAX_SPEED = 5.00 # set the max speed (really 10.2)


blackboard.robot = robot # add the robot to the blackboard
blackboard.waypoints = [(0.35, -2.9), (-1.55, -2.9), (-1.5, 0.11), (0.4, 0.6)] # add the waypoints of getting the map to the blackboard



# this class is used to check if the file that contains the map exists
class DoesMapExist(py_trees.behaviour.Behaviour):
    def update(self):
        # check if the file exists
        file_exists = exists('cspace.npy') 
        if file_exists: # determine if the map exists of not and return success or failuer accordingly 
            print('Map already exists')
            return py_trees.common.Status.SUCCESS
        else:
            print('Map does not exists')
            return py_trees.common.Status.FAILURE
            
            

# declare the behaviour tree
tree = Sequence("Main", children=[
    Selector("Does map exist", children=[ # this selector will be successful if any of its children is successful
        DoesMapExist('Test for map'), # check if the map already exists
        Parallel("Mapping", policy = py_trees.common.ParallelPolicy.SuccessOnOne(), children=[ # map and navigate to create the map
            Mapping('mapping test', blackboard),
            Navigation("MoveAroundTable", blackboard)
        ])
    ], memory=True),
    Planning('planning test', blackboard, (-1.65, -3.30)),
    Navigation('Go bottom left corner', blackboard),
    Planning('planning test', blackboard, (0.65, 0.00)),
    Navigation('Go bottom left corner', blackboard)
    ], memory=True)


tree.setup_with_descendants()


# execute the behaviour 
while robot.step(timestep) != -1:
    # behaviour_tree.tick()
    tree.tick_once()
    
    # Optionally, print the status of the tree
    # print(f"Tree status: {tree.status}")
    
    
 