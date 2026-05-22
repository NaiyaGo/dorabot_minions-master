# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-07
@author : {xiaoyu.ge, chong.chen2, tian.xiao}@dorabot.com
@brief : Due to lack of linear-algebra support in python, we implement
geometric primitves and basic linear algebra operations here.
"""
import math

class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.tuple = (x, y)
    def dot(self, vec):
        return self.x * vec.x + self.y * vec.y
    def scale(self, ratio):
        return Vector(self.x*ratio, self.y*ratio)
    # For convenience in interfacing with box2d
    def to_tuple(self):
        return self.tuple
    def norm(self):
        return math.sqrt(self.dot(self) + 0.000001)
    def get_angle(self, v2):
        angle = math.acos(self.dot(v2)/(self.norm() * v2.norm()))
        if v2[1] < 0:
            return -angle
        else:
            return angle
    def normalize(self):
        return self.scale(1/self.norm())
    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)
    def __getitem__(self, idx):
        return self.tuple[idx]
    def __str__(self):
        return "Vector({},{})".format(self.x, self.y)
class Point:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
    def squared_distance(self, point):
        if point == None: # no destination
            return 0.0 # arrived
        delta_x = point.x - self.x
        delta_y = point.y - self.y
        return delta_x * delta_x + delta_y * delta_y
    """ NOTE We should minise the use of sqrt throughout the project
    """
    def distance(self, point):
        return math.sqrt(self.squared_distance(point))
    #Check equal
    def equal(self, point):
        return self.x == point.x and self.y == point.y
    # For convenience in interfacing with box2d
    def to_tuple(self):
        return (self.x, self.y)
    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)
    def __str__(self):
        return str(self.x) + "  " + str(self.y)
    def copy(self):
        """return a new copy with same x,y"""
        return Point(self.x, self.y)
    def arrive(self, pos):
        if self.distance(pos) < 2e-1:
            return True
        return False
# compute center from a top left point and dimension
def comptue_center_point(top_left_point, dimension):
    x = top_left_point.x + dimension[0]/2
    y = top_left_point.y + dimension[1]/2
    return Point(x, y)

# direction from point_1 to point_2
def compute_direction(src, dest):
    distance = src.distance(dest)
    try:
        vector = Vector((dest.x - src.x)/distance, (dest.y - src.y)/distance)
    except:
        vector = Vector(0,0)
    return vector

"""distance functions"""
def point_point_square_distance(point1, point2):
    return point1.squared_distance(point2)

def point_edge_shortest_square_distance(point, line_end1, line_end2):
    """inputs are all Points;
    adapted from https://stackoverflow.com/questions/27161533/find-the-shortest-distance-between-a-point-and-line-segments-not-line"""
    if line_end1.equal(line_end2):
        return point_point_square_distance(point, line_end1)
    
    px, py = point.x, point.y
    lx1, ly1 = line_end1.x, line_end1.y
    lx2, ly2 = line_end2.x, line_end2.y

    dx = lx2 - lx1
    dy = ly2 - ly1
    dr2 = float(dx ** 2 + dy ** 2)

    lerp = ((px - lx1) * dx + (py - ly1) * dy) / dr2
    if lerp < 0:
        lerp = 0
    elif lerp > 1:
        lerp = 1

    x = lerp * dx + lx1
    y = lerp * dy + ly1

    _dx = x - px
    _dy = y - py
    square_dist = _dx ** 2 + _dy ** 2
    return square_dist

def edge_edge_shortest_square_distance(line1_end1, line1_end2, line2_end1, line2_end2):
    """inputs are all Points;
    adapted from http://jeffreythompson.org/collision-detection/line-rect.php;
    return (shortest_square_distance, intersection_point if applicable otherwise None)"""
    
    if line1_end1.equal(line1_end2) and line2_end1.equal(line2_end2): # point-point
        square_distance = point_point_square_distance(line1_end1, line2_end1)
        if square_distance == 0.0: # intersected (same position)
            return (square_distance, line1_end1.copy())
        return (square_distance, None)
    elif line1_end1.equal(line1_end2): # point-line segment
        square_distance = point_edge_shortest_square_distance(line1_end1, line2_end1, line2_end2)
        if square_distance == 0.0: # intersected
            return (square_distance, line1_end1.copy())
        return (square_distance, None)
    elif line2_end1.equal(line2_end2): # point-line segment
        square_distance = point_edge_shortest_square_distance(line2_end1, line1_end1, line1_end2)
        if square_distance == 0.0: # intersected
            return (square_distance, line2_end1.copy())
        return (square_distance, None)

    x1, y1 = line1_end1.x, line1_end1.y
    x2, y2 = line1_end2.x, line1_end2.y
    x3, y3 = line2_end1.x, line2_end1.y
    x4, y4 = line2_end2.x, line2_end2.y

    if (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)==0: # parallel
        square_distance1 = point_edge_shortest_square_distance(line1_end1, line2_end1, line2_end2)
        square_distance2 = point_edge_shortest_square_distance(line1_end2, line2_end1, line2_end2)
        if min([square_distance1, square_distance2]) == 0.0: # line1 and line2 on the same line, in fact, infinity intersection points exist
            if square_distance1 <= square_distance2:
                return (square_distance1, line1_end1.copy())
            else:
                return (square_distance2, line1_end2.copy())
        else:
            return (min([square_distance1, square_distance2]), None)

    # (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1) != 0
    # calculate the direction of the lines
    uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
    uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))

    # if uA and uB are between 0-1, lines are colliding
    if 0 <= uA <= 1 and 0 <= uB <= 1:
        # compute the intersection
        intersection_x = x1 + (uA * (x2-x1))
        intersection_y = y1 + (uA * (y2-y1))
        return (0.0, Point(intersection_x, intersection_y))
    # not intersected, compute the shortest distance
    square_distance1 = point_edge_shortest_square_distance(line1_end1, line2_end1, line2_end2)
    square_distance2 = point_edge_shortest_square_distance(line1_end2, line2_end1, line2_end2)
    square_distance3 = point_edge_shortest_square_distance(line2_end1, line1_end1, line1_end2)
    square_distance4 = point_edge_shortest_square_distance(line2_end2, line1_end1, line1_end2)
    return (min([square_distance1, square_distance2, square_distance3, square_distance4]), None)

"""collision functions"""
def point_point_collision(point1, point2, square_radius = 0):
    '''return True if collide (the distance between points *** <= *** !!!square_radius!!!)'''
    return point_point_square_distance(point1, point2) <= square_radius
    
def point_edge_collision(point, line_end1, line_end2, square_radius = 0):
    '''return True if collide (the distance between point and line segment *** <= *** !!!square_radius!!!)'''
    return point_edge_shortest_square_distance(point, line_end1, line_end2) <= square_radius

def edge_edge_collision(line1_end1, line1_end2, line2_end1, line2_end2, square_radius = 0):
    '''return True if collide (the distance between line segments *** <= *** !!!square_radius!!!)'''
    (square_distance, _) = edge_edge_shortest_square_distance(line1_end1, line1_end2, line2_end1, line2_end2)
    return square_distance <= square_radius
