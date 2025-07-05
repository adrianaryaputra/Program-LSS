import numpy as np
from geometry_msgs.msg import Point

def distance_line_to_point(init : Point, dest : Point, curr : Point):
    y2Miny1 = dest.y-init.y
    x2Minx1 = dest.x-init.x
        
    return ((y2Miny1*curr.x - x2Minx1*curr.y + dest.x*init.y - dest.y*init.x)
                / np.sqrt(y2Miny1*y2Miny1 + x2Minx1*x2Minx1))


def get_angle(pos1 : Point, pos2 : Point):
    '''
    param:
        pos1 : Point
        pos2 : Point
    return:
        angle : float
    description:
        get the angle between two points
    '''
    return np.arctan2(-pos1.y + pos2.y, -pos1.x + pos2.x)


def rad_to_deg(anglee : float):
    '''
    param:
        angle : float
    return:
        angle : float
    description:
        convert angle from radian to degree
    '''
    return anglee * 180 / np.pi

