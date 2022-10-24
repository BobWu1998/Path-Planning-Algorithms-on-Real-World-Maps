# Path-Planning-Algorithms-on-Real-World-Maps
This project aims to compare different path planning algorithms including A*, RRT, RRT*, Deep Q reinforcement learning (DQN) based path planning, and test them in a realistic and complex indoor environment. The environment is represented by a 2D occupancy map which is generated through a SLAM algorithm in Gazebo simulator. Using path planning time, path length, path smoothness as metrics, different algorithms outperform others in different aspects. A* is capable to find a shortest path and develop a dynamically feasible trajectory because of its relative path smoothness. RT or RRT* could be used for planning a path in large maps due to its short planning time and exploration ability. Though DQN based path planning sometimes didn't give a promising result comparing with other path planners, it still can be improved by incorporating complex network structures to generate more optimal path.