# -*- coding: utf-8 -*-
# @Copyright Dorabot Inc.
# @date : 2018-07
# @author : {xiaoyu.ge, tian.xiao}@dorabot.com
# @brief : possible shapes of an agent
class Rectangle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
    def get_half_dimension(self):
        return (self.width/2.0, self.height/2.0)
    def get_dimension(self):
        return (self.width, self.height)
    def get_radius(self):
        return pow(self.width**2+self.height**2,0.5)/2
    def get_box2d_location(self):
        return(self.x + self.width/2.0, self.y + self.height/2.0)

class RectangleWallCreater(Rectangle):
    def __init__(self, location, dimension):
        Rectangle.__init__(self, location.x, location.y, dimension[0], dimension[1])

