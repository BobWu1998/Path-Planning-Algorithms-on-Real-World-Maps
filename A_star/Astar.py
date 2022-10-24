from heapq import heapify, heappush, heappop, heapreplace
import numpy as np

class Node:
    '''
    A customized class Node
    attributes:
        total_cost: total_cost = heuristic cost + cost_to_come
        heuristic_cost: the L2 norm distance between the node and goal node
        cost_to_come: the best cost to visit the node
        parent_index: the index of parent node, i.e [x, y, z]
        is_visited: True if visited
        index: the index of current node, i.e [x, y, z]
    '''

    def __init__(self,heuristic_cost, cost_to_come, index, parent_index, astar):
        self.heuristic_cost = heuristic_cost
        self.cost_to_come = cost_to_come
        self.index = index
        self.parent_index = parent_index
        self.is_visited = False
        self.total_cost = self.heuristic_cost + self.cost_to_come
        self.astar = astar

    def __lt__(self,other):
        if self.astar:
            return self.total_cost < other.total_cost
        else:
            return self.cost_to_come < other.cost_to_come

def graph_search(occ_map, start, goal, astar):
    """
    Parameters:
        occ_map,    2d occupancy map
        start,      xy position in index, tuple
        goal,       xy position in index, tuple
        astar,      if True use A*, else use Dijkstra
        xy coordinate align with the row, column convention
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """
    # the following code is to initialize a priority queue and a dictionary with all nodes stored inside
    heap = []
    dic = dict()
    unvisited_dic = dict()
    for x in range(occ_map.shape[0]):
        for y in range(occ_map.shape[1]):
                index = (x, y)
                parent_index = None
                heuristic_cost = np.linalg.norm(np.array(index)-np.array(goal)) # calculate the distance(L2 norm) to goal in index
                if x == start[0] and y == start[1]: # set the cost of the start node to be zero
                    cost_to_come = 0
                    node = Node(heuristic_cost, cost_to_come, index, parent_index, astar)
                    heappush(heap,node)
                else:
                    cost_to_come = np.inf
                    node = Node(heuristic_cost, cost_to_come, index, parent_index, astar)
                dic[index] = node # add the node to a dictionary referenced by its index
                unvisited_dic[index] = node
                # help_node = Node(np.inf,np.inf,(-1,-1,-1),None,astar)
                # heappush(heap,help_node)
    
    initial_num_node = len(dic)
    
    
    # the following code is excuting breath-first search
    if astar:
        comp = heap[0].total_cost
    else:
        comp = heap[0].cost_to_come

    while (unvisited_dic.get(goal) != None) and (comp < np.inf) and len(heap) !=0:
        visit_node = heappop(heap)
        visit_node.is_visited = True
        # print('visit node',visit_node.index,' ', 'visit node cost',visit_node.cost_to_come)
        # print(visit_node.index)
        del unvisited_dic[visit_node.index]
        # print('is goal visited', not goal_index in unvisited_dic)
        # print('visit_node',visit_node.index,'visit_node parent',visit_node.parent_index)
        neighboring_node = find_neighbors(visit_node, dic, occ_map)
        for node in neighboring_node:
            edge_cost = np.linalg.norm(np.array(node.index)-np.array(visit_node.index))
            d = visit_node.cost_to_come + edge_cost
            if node.cost_to_come == np.inf and d < node.cost_to_come:
                heappush(heap, node)
            if d < node.cost_to_come:
                node.cost_to_come = d
                node.total_cost = node.cost_to_come + node.heuristic_cost
                node.parent_index = visit_node.index
                
                # print('node:',node.index,' ','parent:',visit_node.index,' ','node cost',node.cost_to_come)
        heapify(heap)

    # retrive the shortest path 
    num_node_expanded = initial_num_node - len(unvisited_dic)
    current_node = dic[goal]
    index_path = [current_node.index]

    while current_node.parent_index != None:
        parent_node = dic[current_node.parent_index]
        index_path.insert(0, parent_node.index)
        current_node = parent_node
    # print(index_path)
    if index_path[0] != start:
        path = None
    else:
        path = index_path
    # print(path)
    return path, num_node_expanded

def find_neighbors(node, dic, occ_map):
    '''
    find the neighboring nodes of a given nodes in a map. Qualifeid neighbors should not exceed the boundary and not occupied
    Input:
        node: the neighbors of which to be found
        dic: the dictionary which stores all the node indexed by their coordinate
        map: an occupancy_map object
    Return:
        a list of neighboring nodes
    '''
    index = node.index
    neighbor_list = []
    for x in range(index[0]-1, index[0]+2):
        for y in range(index[1]-1, index[1]+2):
            if not occ_map[x,y]:
                if not dic[(x,y)].is_visited:
                    # print('nb node',(x,y,z))
                    neighbor_list.append(dic[(x,y)])
                            
    return neighbor_list
