# -*- coding: utf-8 -*-
# @Copyright Dorabot Inc.
# @date : 2018-10
# @author: tian.xiao@dorabot.com
# @brief : gridmap data structure for A* using
from math import ceil, sqrt
from geometry import Point
from representation.gridmap import GridMap
from queue import deque
from sets import Set
class GridmapWithNeighbors(GridMap):
    '''new functions for gridmap
    '''
    weighted_edges={}
    
    def __init__(self, gridmap):
        GridMap.__init__(self, gridmap.width_in_meters, gridmap.height_in_meters, gridmap.resolution, gridmap.top_left_corner)
        self.array = gridmap.array

    def in_bounds(self, position):
        (x, y) = position
        return 0 < x < self.num_x_grids and 0 < y < self.num_y_grids
    
    def passable(self, position):
        (x, y) = position
        try:
            return self.array[int(x)][int(y)] in (0, 2, 3)
        except:
            return False
        
    def neighbors(self, position):
        if position == None:
            return None
        (x, y) = position
        # results = [(x + 1, y), (x, y - 1), (x - 1, y), (x, y + 1),(x + 1, y+1),(x + 1, y-1),(x - 1, y+1),(x - 1, y-1)]
        results =  [(x+1,y),(x,y-1),(x-1,y),(x,y+1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results
    '''Cost for specific use
       Short distance ~= Bigger weight
       For A* using, the cost should be same here because to_position is the neighbors of from position
    '''
    def available_list_from_location(self, start):
        visited = Set()
        visiting = deque()
        visiting.appendleft(start)
        while visiting:
            temp_visiting = visiting.pop()
            visited.add(temp_visiting)
            neighbors = self.neighbors(temp_visiting)
            for neighbor in neighbors:
                if neighbor not in visited and not visiting.count(neighbor):
                    visiting.appendleft(neighbor)
        return visited

    def available_index_list(self):
        results = []
        for i in range(self.num_x_grids):
            for j in range(self.num_y_grids):
                results.append((i,j))
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        results = filter(self.__get_coordinates, results)
        index_list = map((lambda position: position[0]/self.resolution + self.width_in_meters * position[1]/self.resolution), results)
        return index_list 


    def __get_coordinates(self, position):
        (x,y) = position
        return x%self.resolution == 0 and y%self.resolution == 0 
    '''Cost for specific use
       Short distance ~= Bigger weight
       For A* using, the cost should be same here because to_position is the neighbors of from position
    '''
    def cost(self, from_position, to_position):
        #for a in GridmapWithNeighbors.weighted_edges.keys():
        (x1, y1) = from_position
        (x2, y2) = to_position
        distance = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        try:
            result = GridmapWithNeighbors.weighted_edges[(x1, y1)][(x2, y2)] + distance
        except:
            result = distance
        return result
        
        
    def add_weighted_edge(self, from_position, to_position, weight, directed = False):
        try:
            GridmapWithNeighbors.weighted_edges[from_position][to_position] = weight
        except:
            GridmapWithNeighbors.weighted_edges[from_position] = {to_position : weight}
        if directed == False:
            try:
                GridmapWithNeighbors.weighted_edges[to_position][from_position] = weight
            except:
                GridmapWithNeighbors.weighted_edges[to_position] = {from_position : weight}
        return
        
    def static_obstacle_inflation(self, inflation = 4.0, directed = False):
        for i in range(self.num_x_grids):
            for j in range(self.num_y_grids):
                u = (i,j)
                if self.array[i][j] == 0:
                    continue
                elif self.array[i][j] == 3:
                    for v in self.neighbors(u):
                        self.add_weighted_edge(u, v, inflation)
                        for w in self.neighbors(v):
                            if w != u:
                                self.add_weighted_edge(v,w, inflation/2)
                else:# self.array[i][j]==4:
                    for v in self.neighbors(u):
                        for w in self.neighbors(v):
                            self.add_weighted_edge(v, w, inflation)
                        
        return