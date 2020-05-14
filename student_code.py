from helpers import Map, load_map, show_map
import math
import heapq

def shortest_path(M,start,goal):
    start_from = {}
    cost_g = {}
    start_from[start] = None
    cost_g[start] = 0
    frontier_node = [(0, start)]
    
    while len(frontier_node) > 0:
        current_node = heapq.heappop(frontier_node)[1]
        
        if current_node == goal:
            break
        
        for neighbor_node in M.roads[current_node]:
            path_cost_g = distance(M.intersections[current_node], M.intersections[neighbor_node])
            segment_distance = cost_g[current_node] + path_cost_g
            
            if neighbor_node not in cost_g or segment_distance < cost_g[neighbor_node]:
                start_from[neighbor_node] = current_node
                cost_g[neighbor_node] = segment_distance
                # implement A* function f = g + h
                f_cost = segment_distance + distance(M.intersections[goal], M.intersections[neighbor_node])               
                heapq.heappush(frontier_node, (f_cost, neighbor_node))
                
                
    return shortest_route(start_from, start, goal)

def distance(start_node, end_node):
    return math.hypot(end_node[0] - start_node[0], end_node[1] - start_node[1])

def shortest_route(start_from, start, goal):
    node = goal
    path = []
    
    if node not in start_from:
        print('Node: {} not found in map.'.format(node))
        return
    
    while node != start:
        path.append(node)
        node = start_from[node]
    
    path.append(start)
    path.reverse()
    print(path)
    return path