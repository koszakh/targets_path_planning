#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rvo2
from targets_path_planning.msg import AllPaths, Vector2d, Path, NamesList
from geometry_msgs.msg import Point
from path_planning.Heightmap import Heightmap
from charge.charge import eval_charge_points
import path_planning.Constants as const
import gazebo_communicator.GazeboConstants as gc_const
import gazebo_communicator.GazeboCommunicator as gc
from time import sleep
import copy
from charge.utils import convert_to_path, prepare_paths_msg, prepare_path_msg, convert_to_point
from charge.MsgListener import MsgListener


def c_names_callback(msg):
	c_names = msg_data.names
	return c_names


def parse_worker_paths_msg(workers_msg):
	all_paths = workers_msg.all_paths
	w_names = []
	workers_data = dict()
	workers_workpoints = dict()
	for paths in all_paths:
		w_name = paths.robot_name
		w_names.append(w_name)
		workers_data[w_name] = []
		workers_workpoints[w_name] = []
	for paths in all_paths:
		w_name = paths.robot_name
		for item in paths.paths:
			path = item.path
			path = convert_to_path(path)
			workers_data[w_name].append(path)
		workpoints = paths.workpoints.path
		workpoints = convert_to_path(workpoints)
		workers_workpoints[w_name].append(workpoints)

	return w_names, workers_data, workers_workpoints


def charge_alloc(charging_pts, charging_robot_names):
	# Queue of names
	print(charging_robot_names)
	ch_robot_names = charging_robot_names
	# Data preparation
	allocation = dict()
	for name in ch_robot_names:
		allocation[name] = []

	nums_of_ch_points = [len(points) for points in charging_pts.values()]
	max_nums_of_points = max(nums_of_ch_points) if len(nums_of_ch_points) > 0 else 0
	i = 0
	# While i less than number of every list of charging points
	while i < max_nums_of_points:
		for work_robot_name in charging_pts.keys():
			# If i less than number of list of charging points
			if i < len(charging_pts[work_robot_name]):
				ch_robot_name = ch_robot_names[0]
				allocation[ch_robot_name].append(charging_pts[work_robot_name][i])
				tmp_name = ch_robot_names.pop(0)
				ch_robot_names.append(tmp_name)
		i += 1

	return allocation


def define_charging_points(workers_data, workers_workpoints):
	ch_p = dict()
	for w_name in workers_data.keys():
		ch_p[w_name] = []

	for w_name in workers_data.keys():
		energy_resource = 100
		paths = workers_data[w_name]
		for path in paths:
			ch_pts, energy_resource = eval_charge_points(path, workers_workpoints, energy_resource)
			for ch_pt in ch_pts:
				ch_p[w_name].append(ch_pt)

	return ch_p


if __name__ == "__main__":
	rospy.init_node('energy_check')
	c_names_sub = MsgListener()

	#hm = Heightmap(const.HEIGHTMAP_SDF_PATH)
	#hmap, l_scale, w_scale, x_step, y_step, step_count = hm.prepare_heightmap()

	workers_data = rospy.wait_for_message('/worker_paths', AllPaths)
	w_names, workers_data, workers_workpoints = parse_worker_paths_msg(workers_data)
	print("Converting charging names")
	c_names = c_names_sub.c_names
	print("Defining charging points")
	charging_points = define_charging_points(workers_data, workers_workpoints)
	print("Allocating charging robots")
	robot_allocation = charge_alloc(charging_points, c_names)

	print("Publishing allocation")
	allocation_pub = rospy.Publisher('/charging_robots/allocation', AllPaths, queue_size=10)
	allocation_msg = prepare_paths_msg(c_names, robot_allocation)

	print("Publishing points")
	charging_points_pub = rospy.Publisher('/working_robots/charging_points', AllPaths, queue_size=10)
	charging_points_msg = prepare_paths_msg(w_names, charging_points)

	print("Done!")
	print(allocation_msg)
	print(charging_points_msg)
	while True:
		allocation_pub.publish(allocation_msg)
		charging_points_pub.publish(charging_points_msg)

	#rospy.spin()
