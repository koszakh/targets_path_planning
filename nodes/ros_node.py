#!/usr/bin/env python

import rospy
from targets_path_planning.msg import Point3d, AllPaths, Path
from path_planning.Point import Point, Vector2d
from geometry_msgs.msg import Pose, Twist
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
import path_planning.PathPlanner as pp
import path_planning.Constants as const
from path_planning.ORCA import ORCAsolver
from path_planning.Heightmap import Heightmap
import copy
from math import sqrt, asin, fabs, pi
from time import sleep
from time import time
import random

     
def convert_to_path(msg):
    path = []
    for point_msg in msg.path:
        p = convert_to_point(point_msg)
        path.append(p)
    return path

def convert_to_point(msg):
    x = msg.x
    y = msg.y
    z = msg.z
    point = Point(x, y, z)
    return point

def make_pose_msg(state, orient):
    msg = Pose()
    msg.position.x = state.x
    msg.position.y = state.y
    msg.position.z = state.z
    if orient:
        msg.orientation.x = orient[0]
        msg.orientation.y = orient[1]
        msg.orientation.z = orient[2]
    return msg        

rospy.init_node('ros_node')
sleep(1)
hm = Heightmap()
hmap, height, width, x_step, y_step, grid_step = hm.prepare_heightmap()
map_handler = pp.PathPlanner(hmap, height, width, grid_step, x_step, y_step)
name = 'p3at1'
model_directory = gc_const.ROBOT_MODEL_PATH + name[len(name) - 1] + ".urdf"
robot_pos = map_handler.get_random_start_pos()
gc.spawn_urdf_model(name, model_directory, robot_pos)
p = Point(0, 0, 0)
robot = gc.Robot(name)
robot.waypoint_publisher(p)
robot.start()
print('Finish!')
rospy.spin()

