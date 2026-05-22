#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 10:06:59 2018

@author: cathychen
"""
import shapely.geometry as geo
from math import ceil
from representation.gridmap_a import *
class GridMapCalculator(object):
    def polygon_to_gridmap(gridmap, polygon_vertices, resolution, val,  inflation=0.0, centroid=False):
        """
        Set a polygon with value onto a gridmap.
        gridmap: representation.Gridmap object
        polygon_vertices: a list of polygon vertices, written in float tuples [(f,f),(f,f)]
        inflation: polygon inflation to avoid agent-obstacle collision. 
        resolution: int, 1,2,3 etc.
        val: value for points on the gridmap:
            0 for empty
            1 for obstacle, 
            2 for unloading port
            3 for loading port
            4 for queuing area
        """
        # Resolution Transformation:
        enlarged_vertices=[(v[0]*resolution, v[1]*resolution) for v in polygon_vertices]
        enlarged_polygon=geo.Polygon(enlarged_vertices)
        minx, miny, maxx, maxy=enlarged_polygon.bounds # a (minx, miny, maxx, maxy) tuple (float values) 
        minx, miny, maxx, maxy =int(minx-inflation*resolution)-1, int(miny-inflation*resolution)-1, ceil(maxx+inflation*resolution)+1, ceil(maxy+inflation*resolution)+1 # transform float bounds to enlarged int bounds
        for x in range(minx, maxx+1):
            for y in range(miny, maxy+1):
                grid_gridmap_index=(x,y)
                float_gridmap_index=grid_to_float(grid_gridmap_index, resolution, centroid=False)
                float_point=geo.Point(float_gridmap_index)
                if float_point.distance(enlarged_polygon)<=inflation*resolution:
                    gridmap[x,y]=val
        return

    def rectangle_to_gridmap(gridmap, location, dimension, val, resolution, inflation=0.0, centroid=False):
        """
        Set a rectangle with value onto a gridmap.
        location: geometry.Point object representing the upper-left corner of the rectangle
        dimension: 2D float 
        """
        upper_left=(location.x, location.y)
        upper_right=(location.x+dimension[0], location.y)
        lower_left=(location.x, location.y+dimension[1])
        lower_right=(location.x+dimension[0], location.y+dimension[1])
        vertices=[upper_left, upper_right, lower_left, lower_right]
        polygon_to_gridmap(gridmap, vertices, inflation, resolution, val,centroid=False)
        return

    def agent_to_gridmap(position, gridmap, resolution, destination_location=None, centroid=False):
        """
        Transform an agent position to an AVAILABLE point on a gridmap.
        (Optional: From the available points, choose one closest to destination_location.)
        """
        px, py=position.x*resolution, position.y*resolution
        int_x=int(px-0.5*int(centroid))
        int_y=int(py-0.5*int(centroid))
        sample_pool=[(int_x, int_y),(int_x+1, int_y),(int_x, int_y+1),(int_x+1, int_y+1)]
        results = filter(gridmap.in_bounds, sample_pool)
        results = filter(gridmap.passable, results)
        if destination_location != None:
            results.sort(key = destination_location.distance)
        return results[0]
        
        
    def float_to_grid(gridmap_index, resolution, centroid=False):
        """
        Transform a 2D float tuple to a 2D int tuple representing a point on a gridmap.
        centroid: whether points on a grid are in the center or upper left corner of the integer squares on a float map.
        """
        fx,fy = (gridmap_index[0]*resolution, gridmap_index[1]*resolution) # Scale to resolution so that grids are 1x1.
        grid_x = round(fx-0.5*int(centroid))
        grid_y = round(fy-0.5*int(centroid))
        return (grid_x, grid_y)

    def grid_to_float(gridmap_index, resolution, centroid=False):
        """
        Transform a 2D int tuple to a 2D float tuple representing a point on the geometric map.
        centroid: whether points on a grid are in the center or upper left corner of the integer squares on a float map.
        """    
        grid_x, grid_y = gridmap_index
        fx = (grid_x+0.5*int(centroid))/resolution
        fy = (grid_y+0.5*int(centroid))/resolution
        return (fx, fy)
    
    
    
    
    
    