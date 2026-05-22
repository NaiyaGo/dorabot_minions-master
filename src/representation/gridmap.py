# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-10
@author: {xiaoyu.ge, tian.xiao}@dorabot.com
@brief : gridmap data structure
"""
from math import ceil
class GridMap:
    """ Data structuct of a 2D labelled map
    * width_in_meters
    * height_in_meters
    * resolution: pixel per meter
    * top_left_point (origin): coordinate of the top left corner of the gridmap
    * array: internal storage structure
    TODO Gary: @Tian add necessary functions according to the c++ version:
    http://gitlab.dorabot.com/dorabot/dr_maps/blob/master/include/dr_maps/gridmap_2d.hh
    """

    def __init__(self, width_in_meters, height_in_meters, resolution, top_left_corner=(0, 0)):
        self.resolution = resolution
        self.width_in_meters = width_in_meters
        self.height_in_meters = height_in_meters
        self.top_left_corner = top_left_corner
        self.num_x_grids = 0
        self.num_y_grids = 0
        self.zero()

    def zero(self):
        self.num_x_grids = int(ceil(self.width_in_meters * self.resolution))
        self.num_y_grids = int(ceil(self.height_in_meters * self.resolution))
        self.array = [
                [0] * self.num_y_grids
                        for row_idx in range(self.num_x_grids)]
    """ Gridmap accessors
        gridmap[row_idx, col_idx] to access a value
    """
    def __getitem__(self, idx_tup):
        return self.array[int(idx_tup[0])][int(idx_tup[1])]
    def __setitem__(self, idx_tup, val):
        self.array[int(idx_tup[0])][int(idx_tup[1])] = val
