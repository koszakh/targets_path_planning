#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rvo2
from targets_path_planning.msg import AllPaths, Vector2d, Path
from geometry_msgs.msg import Point
from path_planning.Heightmap import Heightmap
from charge.charge import calc_charge_points, eval_charge_points
import path_planning.Constants as const
import gazebo_communicator.GazeboConstants as gc_const
import gazebo_communicator.GazeboCommunicator as gc
from time import sleep
import copy
from charge.utils import convert_to_path, prepare_paths_msg, prepare_path_msg


def charge_alloc(charging_pts, charging_robot_names):
	allocation_pub = rospy.Publisher('allocation', AllPaths, queue_size=10)
	# Queue of names
	ch_robot_names = charging_robot_names
	# Data preparation
	allocation = {}
	for name in ch_robot_names:
		allocation[name] = []

	# All lengths of all paths
	lengths_of_points = [len(points) for points in charging_pts.values()]

	i = 0
	# While i less than length of every path
	while i < max(lengths_of_points):
		for work_robot_name in charging_pts.keys():
			# If i less than length of this path
			if i < len(charging_pts[work_robot_name]):
				ch_robot_name = ch_robot_names[0]
				allocation[ch_robot_name].append(charging_pts[work_robot_name][i])
				#allocation[ch_robot_name].append(BASE_POINT)  # TODO: добавить в Constants точку базы/стартовую точку
				# TODO: в allocation должны быть траектории. Сюда надо засунуть построение траекторий между всеми соседними точками
				tmp_name = ch_robot_names.pop(0)
				ch_robot_names.append(tmp_name)
		i += 1

	# allocation_msg = prepare_paths_msg(charging_robot_names, allocation)
	# allocation_pub.publish(allocation_msg)
	return allocation

def paths_callback(msg_data):
	charging_points_pub = rospy.Publisher('charging_points', AllPaths, queue_size=10)
	print('Paths received.')
	paths_list = msg_data.path_list
	# robots = []
	robot_names = []
	charging_points = {}
	for path_msg in paths_list:
		final_path = convert_to_path(path_msg.path)
		ch_pts = eval_charge_points(final_path, hmap) #  TODO: надо учитывать изначальный запас топлива рабочей платформы
		# TODO: сделать учет расстояния, которое может преодолеть подзаряжающий робот

		name = path_msg.robot_name

		charging_points[name] = ch_pts
		robot_names.append(name)

		# robot = gc.Robot(name)
		# robot.waypoints_publisher(final_path)
		# robots.append(robot)

	#print(charging_points['sim_p3at1'][0].x)
	# charging_points = prepare_paths_msg(robot_names, charging_points)
	# charging_points_pub.publish(charging_points)
	return charging_points


if __name__ == "__main__":
	hm = Heightmap(const.HEIGHTMAP_SDF_PATH)
	hmap, l_scale, w_scale, x_step, y_step, step_count = hm.prepare_heightmap()

	rospy.init_node('energy_check')
	#work_points = rospy.wait_for_message('work_pts', AllPaths)  # TODO: сделать учет точек для работы и трату энергии на них

	#paths_sub = rospy.Subscriber('all_paths_data', AllPaths, paths_callback)
	paths_sub = rospy.wait_for_message('all_paths_data', AllPaths)
	charging_points = paths_callback(paths_sub)

	charging_robots_names = ["charge_p3at1", "charge_p3at2", "charge_p3at3"]  # TODO: надо брать откуда-то из Constants
	robot_allocation = charge_alloc(charging_points, charging_robots_names)

	rospy.spin()
