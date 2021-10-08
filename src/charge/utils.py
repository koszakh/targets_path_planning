#!/usr/bin/env python
# -*- coding: utf-8 -*-

from targets_path_planning.msg import AllPaths, Vector2d, Path, WorkPath, ChargePoints, ChargePoint
from geometry_msgs.msg import Point

def convert_to_point(msg):
    x = state.x
    y = state.y
    z = state.z
    p = Point(x, y, z)
    return p

def convert_to_path(msg):
    path = []
    for state in msg:
        x = state.x
        y = state.y
        z = state.z
        p = Point(x, y, z)
        path.append(p)
    return path


def prepare_paths_msg(name, paths):
    msg = WorkPath()
    msg.paths = []
    #msg.workpoints = prepare_path_msg(workpoints)
    msg.robot_name = name

    for path in paths:
        path_msg = prepare_path_msg(path)
        msg.paths.append(path_msg)

    return msg


def prepare_path_msg(path):
    msg = Path()
    msg.path = []

    for state in path:
        point_msg = prepare_point_msg(state)
        msg.path.append(point_msg)

    return msg


def prepare_point_msg(point):
    # print('point: ' + str(point))
    msg = Point()
    msg.x = point.x
    msg.y = point.y
    msg.z = point.z

    return msg
