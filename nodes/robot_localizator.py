#!/usr/bin/env python

# Target localization node

import rospy
import rvo2
from targets_path_planning.msg import Point3d, AllPaths, AllRobotsPos, RobotPos, Vector2d
from path_planning.Point import Point
from geometry_msgs.msg import Pose
import gazebo_communicator.GazeboConstants as const
import gazebo_communicator.GazeboCommunicator as gc
from time import sleep

def prepare_pose_msg(state, orient):
    msg = Pose()
    msg.position.x = state.x
    msg.position.y = state.y
    msg.position.z = state.z
    if orient:
        msg.orientation.x = orient[0]
        msg.orientation.y = orient[1]
        msg.orientation.z = orient[2]
    return msg

def prepare_robot_pos_msg(state, vector, robot_name):
    msg = RobotPos()
    msg.robot_name = robot_name
    msg.pos.x = state.x
    msg.pos.y = state.y
    msg.pos.z = state.z
    msg.vector.x = vector.x
    msg.vector.y = vector.y
    return msg

def prepare_all_robots_pos_msg(robot_names):
    poses_msg = AllRobotsPos()
    poses_msg.pos_list = []
    for name in robot_names:
        robot_pos = gc.get_model_position(name)
        robot_orient = gc.get_robot_orientation_vector(name)
        msg = prepare_robot_pos_msg(robot_pos, robot_orient, name)
        poses_msg.pos_list.append(msg)
    return poses_msg

rospy.init_node('robot_localizator')
robot_pos_pub = rospy.Publisher('robots_pos_data', AllRobotsPos, queue_size=10)
sleep(2)
print('Robot localization started!')
poses_msg = prepare_all_robots_pos_msg(const.ROBOT_NAMES)
robot_pos_pub.publish(poses_msg)
rospy.spin()
