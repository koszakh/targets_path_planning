#!/usr/bin/env python

# Path planning node for ground targets
import path_planning.TargetAssignment as ta
import gazebo_communicator.GazeboConstants as const
from gazebo_communicator.Robot import Robot
from targets_path_planning.msg import AllPaths, WorkPath, Path, Poses, RobotState, Vector2d, NamesList
from geometry_msgs.msg import Point
import rospy
import time

def prepare_all_paths_msg(names, paths, workpoints):

	msg = AllPaths()
	msg.all_paths = []

	for name in names:

		path = paths[name]
		robot_workpoints = workpoints[name]
		path_msg = prepare_paths_msg(name, path, robot_workpoints)
		msg.all_paths.append(path_msg)

	return msg

def prepare_paths_msg(name, paths, workpoints):

	msg = WorkPath()
	msg.paths = []
	msg.workpoints = prepare_path_msg(workpoints)
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

	#print('point: ' + str(point))
	msg = Point()
	msg.x = point.x
	msg.y = point.y
	msg.z = point.z
	
	return msg
	
def prepare_orient_msg(orient):

	vect = Vector2d()
	vect.x = orient.x
	vect.y = orient.y
	return vect

def prepare_poses_msg(poses, orients):

	msg = Poses()
	msg.robot_states = []
	
	for name in poses.keys():

		robot_msg = RobotState()
		robot_msg.robot_name = name
		robot_msg.pose = prepare_point_msg(poses[name])
		robot_msg.orient = prepare_orient_msg(orients[name])
		msg.robot_states.append(robot_msg)
		
	return msg

def prepare_names_msg(names):

	msg = NamesList()
	msg.names = []
	
	for name in names:
	
		msg.names.append(name)
		
	return msg
		
rospy.init_node('target_assignment')
paths_pub = rospy.Publisher('worker_paths', AllPaths, queue_size=10)
poses_pub = rospy.Publisher('robot_poses', Poses, queue_size=10)
c_names_pub = rospy.Publisher('chargers_names', NamesList, queue_size=10)
t_as = ta.TargetAssignment(const.WORKERS_COUNT, const.CHARGERS_COUNT, const.TARGETS_COUNT)
s_exec_time = time.time()
paths, workpoints, names, w_names, c_names = t_as.target_assignment()
poses, orients = t_as.get_robots_pos_orient(names)

poses_msg = prepare_poses_msg(poses, orients)
poses_pub.publish(poses_msg)

names_msg = prepare_names_msg(c_names)
c_names_pub.publish(names_msg)

paths_msg = prepare_all_paths_msg(w_names, paths, workpoints)
paths_pub.publish(paths_msg)

#msg = prepare_all_paths_msg(paths.keys(), paths, workpoints)
#paths_pub.publish(msg)

f_exec_time = time.time()
exec_time = f_exec_time - s_exec_time
print('Target assignment execution time: ' + str(exec_time))
