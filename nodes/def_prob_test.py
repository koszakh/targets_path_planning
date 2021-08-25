#!/usr/bin/env python

import rospy
from targets_path_planning.msg import AllPaths, Path, TargetDamage
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

rospy.init_node('def_prob_test')
sleep(1)
def_prob_pub = rospy.Publisher('/sim_p3at1/damage', TargetDamage, queue_size=10)
name = 'p3at1'
name1 = 'p3at2'
g = Point(1, 1, 0)
gc.spawn_sdf_model(g, gc_const.GREEN_VERTICE_PATH, 'g')
p = Point(0, 0, 0.2)
p1 = Point(0, 2, 0.2)
gc.spawn_target(name, p, (0, 0, 0, 0))
gc.spawn_target(name1, p1, (0, 0, 0, 0))
robot = gc.Robot(name)
robot1 = gc.Robot(name1)
sleep(1)
robot1.movement(robot1.ms, 0)

for i in range(9):

	sleep(0.5)
	msg = TargetDamage()
	msg.damage = 0.12
	def_prob_pub.publish(msg)

print('Finish!')
rospy.spin()

