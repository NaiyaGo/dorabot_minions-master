#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 10:06:59 2018

@author: cathychen
"""
from shapely import geometry as geo
from math import ceil
from representation.gridmap_a import *
class GridMapCalculator:
    def polygon_to_gridmap(gridmap, polygon_vertices, inflation=0.0, resolution=1, val=1, centroid=False):
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
                grid_coord=(x,y)
                float_coord=grid_to_float(grid_coord, resolution=1, centroid=False)
                float_point=geo.Point(float_coord)
                if float_point.distance(enlarged_polygon)<=inflation*resolution:
                    gridmap[x,y]=val
        return


def polygon_to_gridmap(gridmap, polygon_vertices, inflation=0.0, resolution=1, val=1, centroid=False):
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
            grid_coord=(x,y)
            float_coord=grid_to_float(grid_coord, resolution=1, centroid=False)
            float_point=geo.Point(float_coord)
            if float_point.distance(enlarged_polygon)<=inflation*resolution:
                gridmap[x,y]=val
    return

def rectangle_to_gridmap(gridmap, location, dimension, inflation=0.0, resolution=1, val=1,centroid=False):
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

def agent_to_gridmap(position, gridmap, resolution=1, destination_location=None, centroid=False):
    """
    Transform an agent position to an AVAILABLE point on a gridmap.

    If the rounded grid cell is passable, return it immediately.
    Otherwise do a spiral search (expanding Chebyshev rings up to radius 5)
    for the nearest passable in-bounds cell.  If destination_location is
    given, among the ties pick the cell closest to the destination.

    This fallback is what allows a physically stuck agent — whose body may
    have nudged slightly into a wall cell after a collision — to still have
    a valid start point for the global planner, so it can re-plan its way
    out instead of being skipped by A*.
    """
    px = position.x * resolution - 0.5 * int(centroid)
    py = position.y * resolution - 0.5 * int(centroid)
    rx, ry = int(round(px)), int(round(py))

    if gridmap.in_bounds((rx, ry)) and gridmap.passable((rx, ry)):
        return (rx, ry)

    # Spiral / Chebyshev-ring search for nearest passable cell.
    dest_cell = None
    if destination_location is not None:
        dx = destination_location.x * resolution - 0.5 * int(centroid)
        dy = destination_location.y * resolution - 0.5 * int(centroid)
        dest_cell = (int(round(dx)), int(round(dy)))

    MAX_RING = 5
    for r in range(1, MAX_RING + 1):
        candidates = []
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                # Only outermost ring at this radius
                if max(abs(dx), abs(dy)) != r:
                    continue
                c = (rx + dx, ry + dy)
                if gridmap.in_bounds(c) and gridmap.passable(c):
                    candidates.append(c)
        if not candidates:
            continue
        if dest_cell is None:
            return candidates[0]
        # Break ties by proximity to destination
        candidates.sort(key=lambda c: (c[0] - dest_cell[0]) ** 2 + (c[1] - dest_cell[1]) ** 2)
        return candidates[0]

    # Truly hopeless — return the unsafe rounded cell and let the caller handle it.
    return (rx, ry)

        
def float_to_grid(coord, resolution=1, centroid=False):
    """
    Transform a 2D float tuple to a 2D int tuple representing a point on a gridmap.
    centroid: whether points on a grid are in the center or upper left corner of the integer squares on a float map.
    """
    fx,fy=(coord[0]*resolution, coord[1]*resolution) # Scale to resolution so that grids are 1x1.
    grid_x=round(fx-0.5*int(centroid))
    grid_y=round(fy-0.5*int(centroid))
    return (grid_x, grid_y)

def grid_to_float(coord, resolution=1, centroid=False):
    """
    Transform a 2D int tuple to a 2D float tuple representing a point on the geometric map.
    centroid: whether points on a grid are in the center or upper left corner of the integer squares on a float map.
    """    
    grid_x, grid_y=coord
    fx=(grid_x+0.5*int(centroid))/resolution
    fy=(grid_y+0.5*int(centroid))/resolution
    return (fx, fy)

    
    
    
    
    