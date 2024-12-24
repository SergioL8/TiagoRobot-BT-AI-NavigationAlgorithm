# TiagoRobot-BT-AI-NavigationAlgorithm

This repository is organized into two very similar projects. The first project (mapping) involves the Tiago robot mapping the environment if no map is detected, then planning a route and navigating it. The second project is the larger of the two. It is more complex and requires the Tiago robot to recognize three jars, pick them up and place them on the table. Multiple techniques are used to complete these tasks.

## Behavior trees
The robot’s behavior is managed with a behavior tree, a plan of execution used in many fields of computer science. It consists of switching between a finite set of tasks in a modular way. The strength of this technique comes from the ability to execute complex tasks, like the one presented in this project, by combining simple tasks.

In this project, the tree is organized into three blocks. Each consisting of a set of behaviors used to pick a jar and brings it back to the table. The behaviors used are: armPosition, gripper, cameraNavigation, planning, moveBack, navigation and finish. Each of these executes a simple action, but when properly combined we are able to form a more complex task.

One significant improvement for this project would be to optimize the number of behaviors and simplify the tree. Some of the behaviors in the tree are very similar, such as navigation and cameraNavigation or moveBack. We could, for example, modify cameraNavigation to simply output the destination of the robot and then run a parallel process to execute both behaviors at the same time. Furthermore, instead of having three separate blocks in the tree, we could just make one block and repeat it three times. However, this would required some extra polishing I didn’t had time to do.


## A* algorithm
The A* search algorithm is used in many fields of computer science due to its completeness and efficient pathfinding. Compared to Dijkstra’s algorithm, A* focuses only on finding the shortest path from a specified source to a specified goal rather than finding paths to all possible goals. This makes the A* faster because fewer squares of the map need to be visited.

The development of this algorithm is one of the major features of the robot as it allows for quick route planning multiple times during the behavior tree’s execution. 

In the media folder you can see a video of how this algorithm works.



## Use of AI
Another important feature of this project is the use of AI to identify and grab objects. The goal was not to develop an AI for object recognition. Instead, we used Webots apackage and a camera that uses AI to recognize objects and compute the distance with respect to the camera. This feature allowed the robot to identify the jar that had to be picked up. With help of a controller, we were able to compute the position of this jar dynamically, allowing the robot to perfectly adjust the gripper and reliably pick up the jar.



## Mapping
As explained above, this project consists of two parts, one controller that will map the environment and another controller that will move the jars to the table. The mapping uses common industry techniques considered good practice. For example, the use of probabilistic mapping is central this project since the lidar sensor can pick up noise. Only objects that appear consistently for a minimum amount of time are considered valid.

A key feature for navigation is the use of convolution on the map. We do this by using a convolution kernel that effectively grows every object above a certain probability by a the radius of the robot. This produces a ‘free space’ area where the robot can move without risk of collision

In the media folder of this repository, you can find the convolved map created by the mapping program of this project.


Thank you for reading, if you have any questions don’t hesitate to ask me. You can find my contact in my profile.






