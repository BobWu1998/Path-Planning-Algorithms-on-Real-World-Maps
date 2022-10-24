import cv2
import numpy as np
import math
import random
import matplotlib.pyplot as plt
import time

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.coor = np.array([x,y])
        self.parent = []


class rrt:
    def __init__(self, map_path, step_size=20):
        # Initialize map
        self.map  = cv2.imread(map_path,0)
        self.draw_map = cv2.imread(map_path)
        self.height, self.width, _ = self.draw_map.shape

        # Initialize node list and step size
        self.node_list = []
        self.step_size = step_size


    def select_point(self):
        '''
        Helper function for user to select customized point on the map as start and goal point
        Reference:
        https://stackoverflow.com/questions/28327020/opencv-detect-mouse-position-clicking-over-a-picture
        '''
        # Select start and end point
        self.coordinates=[]
        print("Select start and end points by double clicking, press 'escape' to exit")
        # cv2.namedWindow('image')
        # cv2.setMouseCallback('image',self.draw_circle)
        # while(1):
        #     cv2.imshow('image',self.draw_map)
        #     if cv2.waitKey(20) == ord('c'):
        #         break
        # self.start = np.array([self.coordinates[0], self.coordinates[1]])
        # self.goal = np.array([self.coordinates[2], self.coordinates[3]])
        # coordinates = [250, 500, 722, 337]#[10, 200, 500, 350]#[250, 500, 722, 337]#[250, 500, 602, 286] #[500, 300, 500, 350]#[300, 500, 350, 500]#[20, 200, 400, 205]
        coordinates = [180, 560, 555, 220]#[443, 226, 243, 351]#4:[443, 226, 243, 351]#3:[190, 220, 560, 472]#2:[180, 560, 555, 220] #1:[50, 515, 560, 347]
        self.start=np.array([coordinates[0],coordinates[1]])
        self.goal=np.array([coordinates[2],coordinates[3]])
    
    def solve(self, numIterations=1000, print_path=False):
        '''
        Input:
        numIterations: Maximum allowed number of iterations to find the path
        print_path: Boolean to determine if print out the path
        Return:
        None. Generate the cv2 plot of final 
        '''
        # First, initialize the node list with the start node
        initial_node = Node(self.start[0], self.start[1])
        initial_node.parent.append(self.start)
        self.node_list.append(initial_node)
        
        # Then, iterate the rrt loop to find the path 
        h,w = self.map.shape[:2]
        path_found = False
        iter = 0

        while not path_found and iter < numIterations:
            # Generate new random point
            new_node = self.random_point(h,w) # (2,) (x,y)
            nearest_node_idx = self.find_nearest_node(new_node) # (1,)
            nearest_node = self.node_list[nearest_node_idx]
            # Based on the new random point, determine if it has a valid step point
            xy_step, end_con, node_con = self.check_collision(new_node, nearest_node.coor)

            # Case 1: The step point can connect both goal and its nearest point
            if end_con and node_con:
                time_end = time.time()
                print('time cost:', time_end-time_start, 's')
                path_found = True
                node_step = Node(xy_step[0], xy_step[1])
                node_step.parent = self.node_list[nearest_node_idx].parent.copy()
                node_step.parent.append(xy_step)
                self.node_list.append(node_step)
                # Plot the path
                cv2.circle(self.draw_map, xy_step, 2,(0,0,255),thickness=3, lineType=8)
                cv2.line(self.draw_map, xy_step, nearest_node.coor, (0,255,0), thickness=1, lineType=8)
                cv2.line(self.draw_map, xy_step, self.goal, (255,0,0), thickness=2, lineType=8)
                for j in range(len(node_step.parent)-1):
                        cv2.line(self.draw_map, node_step.parent[j], node_step.parent[j+1], (255,0,0), thickness=2, lineType=8)
                cv2.imshow('Final',self.draw_map)
                cv2.waitKey(0)
                if print_path:
                    print('Path found.')
                    print(node_step.parent)
                print('Path Length:', self.cost(node_step))
            # Case 2: Step node connects to current nearest node but not goal
            elif node_con:
                node_step = Node(xy_step[0], xy_step[1])
                node_step.parent = self.node_list[nearest_node_idx].parent.copy()
                node_step.parent.append(xy_step)
                self.node_list.append(node_step)
                self.graph_plot()
                cv2.waitKey(10)
            # Keep track of number of iterations
            iter += 1

        # No path find information
        if not path_found:
            print('No path find. Try to change numIterations or step_size to increase chances of finding a pth')

    def solve_star(self, numIterations=15000, RADIUS=80, print_path=False):
        # Initialize node list by appending the start node
        initial_node = Node(self.start[0], self.start[1])
        initial_node.parent.append(self.start)
        self.node_list.append(initial_node)

        h,w = self.map.shape[:2]
        path_found = False

        for iter in range(numIterations):
            # Generate new random point
            new_node = self.random_point(h,w) # (2,) (x,y)
            nearest_node_idx = self.find_nearest_node(new_node) # (1,)
            nearest_node = self.node_list[nearest_node_idx]
            # Based on the new random point, determine if it has a valid step node
            xy_step, end_con, node_con = self.check_collision(new_node, nearest_node.coor)
            node_step = Node(xy_step[0], xy_step[1])
            if not node_con:
                continue
            else:
                # find all neighbors
                neighbors_idx = self.find_neighbors(xy_step, RADIUS)
                self.node_list.append(node_step) # must check neighbors first
                
                # get best neighbor
                if neighbors_idx:
                    # choose the best parent for the node_step
                    neighbor_cost = [self.cost(self.node_list[i])+self.dist(self.node_list[i].coor, node_step.coor) for i in neighbors_idx]
                    cost_min_idx = neighbors_idx[int(np.argmin(neighbor_cost))]
                    # print(cost_min_idx)
                    node_step.parent = self.node_list[cost_min_idx].parent.copy()
                    node_step.parent.append(xy_step)

                    # rewire
                    for i in neighbors_idx:
                        neighbor_node = self.node_list[i]
                        if self.cost(neighbor_node) > self.cost(node_step) + self.dist(node_step.coor, neighbor_node.coor):
                            neighbor_node.parent = node_step.parent
                            neighbor_node.parent.append(neighbor_node.coor)
                self.graph_plot()

            cv2.waitKey(10)
            if end_con and node_con:
                time_end = time.time()
                print('time cost:', time_end-time_start, 's')
                path_found = True
                node_step = Node(xy_step[0], xy_step[1])
                node_step.parent = self.node_list[nearest_node_idx].parent.copy()
                node_step.parent.append(xy_step)
                self.node_list.append(node_step)
                # Plot the path
                cv2.line(self.draw_map, xy_step, self.goal, (255,0,0), thickness=2, lineType=8)
                for j in range(len(node_step.parent)-1):
                        cv2.line(self.draw_map, node_step.parent[j], node_step.parent[j+1], (255,0,0), thickness=2, lineType=8)
                cv2.imshow('Final',self.draw_map)
                cv2.waitKey(0)
                print('Path found.')
                if print_path:
                    print(node_step.parent)
                print('Path Length:', self.cost(node_step))
                # print('iter_num:', iter)
                break

        if not path_found:
            print('No path find. Try to change numIterations or step_size to increase chances of finding a pth')

    def random_point(self,h,w):
        '''
        Input: 
        h: Height of map image
        w: Width of map image
        Return:
        new_point: (2,) Pixel coordinate of new randomly generated point on map
        '''
        y = np.random.randint(0,h)
        x = np.random.randint(0,w)
        new_point = np.array([x,y])

        return new_point


    def find_nearest_node(self,coor):
        '''
        Input:
        coor (2,): Point of interest
        Return:
        nearest_idx: Nearest point index in self.node_list w.r.t point of interest
        '''
        dist = []
        for i in range(len(self.node_list)):
            coor_curr = self.node_list[i].coor
            distance = np.linalg.norm(coor - coor_curr)
            dist.append(distance)
        nearest_idx = np.argmin(dist)

        return nearest_idx


    def check_collision(self, xy_new, xy_n):
        '''
        Input: 
        xy_new: (2,) New randomly generated point
        xy_n: (2,) Nearest node in self.node_list to this new generated point
        Return:
        xy_step: (2,) New step point on the ray from nearest point to xy_n
        end_con: Boolean denotes whether xy_step directly connects to goal 
        node_con: Boolean denotes whether xy_step is connected with nearest node without obstacle
        '''
        # Find new step point
        angle = math.atan2(xy_new[1] - xy_n[1], xy_new[0] - xy_n[0])
        x_step = int(xy_n[0] + self.step_size * np.cos(angle))
        y_step = int(xy_n[1] + self.step_size * np.sin(angle))
        xy_step = np.array([x_step, y_step])
        # Determine if the step point is out-of-bound
        h,w = self.map.shape[:2] 
        if y_step < 0 or y_step >= h or x_step < 0 or x_step >= w:
            end_con = False
            node_con = False
        else: # If in-bound, determine for end_con and node_con
            # First, check if node connects 
            if self.check_line_collision(xy_step, xy_n):
                node_con = False
            else:
                node_con = True
            # Then, check if end connects
            if self.check_line_collision(xy_step, self.goal):
                end_con = False
            else:
                end_con = True
        
        return xy_step, end_con, node_con


    def check_line_collision(self, xy_1, xy_2):
        '''
        Input: 
        xy_1: (2,) Coordinate of first point (new step)
        xy_2: (2,) Coordinate of second point (nearest)
        Return:
        collision: Boolean denotes whether the line between this two points are obstructed
        '''
        # Get all pixel coordinate along this two point ray with 0.2 pixel step size
        x1, y1 = xy_1[0], xy_1[1]
        x2, y2 = xy_2[0], xy_2[1]
        N_step = int(np.linalg.norm(xy_1 - xy_2)/0.2)
        x = np.linspace(x1, x2, N_step).astype(np.int64)
        y = np.linspace(y1, y2, N_step).astype(np.int64)
        # Check if there is obstacle within this ray
        ray = self.map[y,x]
        if 0 in ray:
            return True
        return False


    def draw_circle(self,event,x,y,flag,argument):
        '''
        Helper function to draw two user defined point on the map
        '''
        if event == cv2.EVENT_LBUTTONDBLCLK:
            cv2.circle(self.draw_map,(x,y),5,(0,0,255),-1)
            self.coordinates.append(x)
            self.coordinates.append(y)
    
    def find_neighbors(self, coor, radius):
        '''
        Input:
        coor (2,): Point of interest
        Return:
        neighbor_idx: all neighbor in radius range of the point of interest
        '''
        dist = []
        for i in range(len(self.node_list)):
            coor_curr = self.node_list[i].coor
            if not self.check_line_collision(coor, coor_curr):
                distance = np.linalg.norm(coor - coor_curr)
            else:
                distance = -1
            dist.append(distance)

        neighbor_idx = [i for i in range(len(dist)) if dist[i]> 0 and dist[i]<radius]

        return neighbor_idx # np.array(neighbor_idx)

    def graph_plot(self):
        cv2.destroyAllWindows()
        for node in self.node_list:
            if len(node.parent) > 1:
                cv2.circle(self.draw_map, node.coor, 2, (0,0,255),thickness=3, lineType=8)
                # print(node, node.parent[-2])
                cv2.line(self.draw_map, node.coor, node.parent[-2], (0,255,0), thickness=1, lineType=8)
                cv2.circle(self.draw_map, self.start, 2,(134,46,155),thickness=9, lineType=8)
                cv2.circle(self.draw_map, self.goal, 2,(154,205,50),thickness=9, lineType=8)
                    
        cv2.imshow("sdc", self.draw_map)

    def cost(self, node):
        cost = 0
        for i in range(len(node.parent)-1):
            # print(np.array(node.parent[i]), np.array(node.parent[i+1]))
            # cost += np.linalg.norm(np.array(node.parent[i]), np.array(node.parent[i+1]))
            cost += math.hypot(node.parent[i][0] - node.parent[i+1][0], node.parent[i][1] - node.parent[i+1][1])
        return cost
    
    def dist(self, coor1, coor2):
        return np.linalg.norm(coor1 - coor2)


if __name__ == '__main__':
    time_start = time.time()
    rrt_solver = rrt(map_path='Final.jpeg',step_size=34)
    rrt_solver.select_point()
    rrt_solver.solve(15000)
    # rrt_solver.solve_star(15000)
