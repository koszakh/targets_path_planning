#!/usr/bin/env python

# Path planning node for ground targets
import path_planning.TargetAssignment as ta
import gazebo_communicator.GazeboConstants as const
from gazebo_communicator.Robot import Robot
from targets_path_planning.msg import AllPaths, WorkPaths, Path
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

	msg = WorkPaths()
	msg.paths = []
	msg.workpoints = []
	msg.robot_name = name

	for path in paths:

		path_msg = prepare_path_msg(path)
		msg.paths.append(path_msg)
		
	for wp in workpoints:
	
		point_msg = prepare_point_msg(wp)
		msg.workpoints.append(point_msg)
		
	return msg

def prepare_path_msg(path):

	msg = Path()
	msg.path = []
	
	for state in path:
	
		point_msg = prepare_point_msg(state)
		msg.path.append(point_msg)
	
	return msg
	
def prepare_point_msg(point):

	point = Point()
	point.x = point.x
	point.y = point.y
	point.z = point.z
	return point
		
rospy.init_node('target_assignment')
#paths_pub = rospy.Publisher('all_paths_data', AllPaths, queue_size=10)
t_as = ta.TargetAssignment(const.WORKERS_COUNT, const.CHARGERS_COUNT, const.TARGETS_COUNT)
s_exec_time = time.time()
paths, workpoints, w_names, c_names, b_trackers = t_as.target_assignment()

workers = {}
chargers = {}

for name in w_names:

	robot = Robot(name, "worker", b_trackers)
	w_points = workpoints[name]
	path = paths[name]
	robot.workpoints_publisher(w_points)
	robot.waypoints_publisher(path)
	workers[name] = robot
	
cont_flag = False
	
while not cont_flag:

	cont_flag = True

	for key in workers.keys():

		if not workers[key].paths:
		
			cont_flag = False

print('Robot movement has begun!')
for key in workers.keys():

	workers[key].start()

#msg = prepare_all_paths_msg(paths.keys(), paths, workpoints)
#paths_pub.publish(msg)

f_exec_time = time.time()
exec_time = f_exec_time - s_exec_time
print('Target assignment execution time: ' + str(exec_time))
