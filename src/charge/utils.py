#!/usr/bin/env python
# -*- coding: utf-8 -*-

from targets_path_planning.msg import AllPaths, Vector2d, Path
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


def prepare_paths_msg(names, paths):
    msg = AllPaths()
    msg.path_list = []
    for name in names:
        path = paths[name]
        path_msg = prepare_path_msg(name, path)
        msg.path_list.append(path_msg)
    return msg


def prepare_path_msg(name, path):
    msg = Path()
    msg.path = []
    msg.robot_name = name
    for state in path:
        point = Point()
        point.x = state.x
        point.y = state.y
        point.z = state.z
        msg.path.append(point)
    return msg
