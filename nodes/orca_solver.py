#!/usr/bin/env python

# Optimal Reciprocal Collision Avoidance calculation node
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

def callback(msg_data):
    orca = ORCAsolver()
    for msg in msg_data.path_list:
        robot_name = msg.robot_name
        path = convert_to_path(msg)
        orca.add_agent(robot_name, path)
    orca.run_orca()
     
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

rospy.init_node('orca_solver')
path_subscriber = rospy.Subscriber('path_data', AllPaths, callback)
rospy.spin()

