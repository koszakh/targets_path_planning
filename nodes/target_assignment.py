#!/usr/bin/env python

# Path planning node for ground targets
import path_planning.TargetAssignment as ta
import gazebo_communicator.GazeboConstants as const
from gazebo_communicator.Robot import Robot
import rospy
import time

		
rospy.init_node('target_assignment')
paths, workpoints, w_names, c_names, b_trackers = ta.start_target_assignment(const.WORKERS_COUNT, const.CHARGERS_COUNT, const.TARGETS_COUNT)
workers = {}
chargers = {}

for name in w_names:

	robot = Robot(name, "worker", b_trackers)
	w_points = workpoints[name]
	path = paths[name]
	robot.workpoints_publisher(w_points)
	robot.waypoints_publisher(path)
	workers[name] = robot

f_exec_time = time.time()
exec_time = f_exec_time - s_exec_time
print('Target assignment execution time: ' + str(exec_time))

				
	
	
