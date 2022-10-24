# Path-Planning-Algorithms-on-Real-World-Maps
This project aims to compare different path planning algorithms including A*, RRT, RRT*, Deep Q reinforcement learning (DQN) based path planning, and test them in a realistic and complex indoor environment. The environment is represented by a 2D occupancy map which is generated through a SLAM algorithm in Gazebo simulator. Using path planning time, path length, path smoothness as metrics, different algorithms outperform others in different aspects. A* is capable to find a shortest path and develop a dynamically feasible trajectory because of its relative path smoothness. RRT or RRT* could be used for planning a path in large maps due to its short planning time and exploration ability. Though DQN based path planning sometimes didn't give a promising result comparing with other path planners, it still can be improved by incorporating complex network structures to generate more optimal path.

## Instructions to run different algorithms:
### 1. How to run astar path planning 
Run “astar.ipynb” which should be in the same directory as “astar.py”, change the map file name to the occupancy map you want to test. It will return a visualization of both map and planned path.

### 2. SLAM
A gazebo world model named “myWorld_levin4.WORLD” is included in the file.

### 3. To run RRT and RRT*
In the RRT folder
Open “rrt.py”
To run RRT, use rrt_solver.solve() in the main function
To run RRT*, use rrt_solver.solve_star() in the main function

### 4. DQN
The DQN algorithm is implemented in the notebook called ‘DQN.ipynb’. Please follow the steps below to run it.
1. First make sure the notebook file is in the same directory as the image ‘simplified_map.jpg’. 
2. Then, choose the test map to run the DQN in ‘Generate different map’ section, which includes random map and simplified grid world map of 4th floor in Levine Hall
3. Last, initialize the running environment with map name and specified start and goal location by entering the coordinate of both locations.
**NOTE:** The coordinate origin for start and goal is located at the bottom left corner of the map with x-axis pointing rightward and y-axis pointing upward. Then you can run the main function. \
The ‘DQN_mapPreprocess.ipynb’ file is used to pre-process the configuration map of 4th floor in Levine Hall to develop a simplified grid world consisting of 1 and 0 2D arrays for the ease of running DQN algorithm.

