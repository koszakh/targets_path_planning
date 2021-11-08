#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import datetime
import copy
from numpy import sqrt
from gazebo_communicator import GazeboConstants as gc_const

TIME_ON_CHARGING = (gc_const.HIGH_LIMIT_BATTERY - gc_const.LOWER_LIMIT_BATTERY)/gc_const.CHARGING_SPEED


def eval_mission_time(paths_w_robots, paths_c_robots_to_ch_p, paths_c_robots_to_base, robot_allocation, charging_points, workpoints):
	times_for_w = eval_arrival_times_for_w(paths_w_robots, charging_points, prepare_times_dict(paths_w_robots))
	times_for_c = eval_arrival_times_for_c(paths_c_robots_to_ch_p, paths_c_robots_to_base, robot_allocation, prepare_times_dict(paths_w_robots))

	waiting_times = eval_waiting_times(times_for_w, times_for_c)
	mission_times = eval_mission_times(paths_w_robots, waiting_times, workpoints)

	times = mission_times.values()
	print(mission_times)
	max_mission_time = max(times)
	total_time = datetime.timedelta(seconds=max_mission_time)
	rospy.loginfo("Approximate mission time: " + str(total_time))
	write_mission_log(total_time)
	return max_mission_time


def eval_arrival_times_for_w(paths_w_robots, charging_points, times):
	""" times: key - w_name, value - (ch_p, arrival_time) """
	arrival_times = times
	for w_name in paths_w_robots.keys():
		#print(w_name)
		for ch_p in charging_points[w_name]:
			point, distance = ch_p
			time_spent = eval_time_on_path(distance)
			data = point, time_spent
			arrival_times[w_name].append(data)
	return arrival_times


def eval_arrival_times_for_c(paths_c_robots_to_ch_p, paths_c_robots_to_base, robot_allocation, times):
	""" times: key - w_name, value - (ch_p, arrival_time) """
	arrival_times = times
	for c_name in paths_c_robots_to_ch_p.keys():
		total_time = 0
		for i in range(len(robot_allocation[c_name])):
			pre_ch_p, ch_p, w_name, _ = robot_allocation[c_name][i]
			path_to_ch_p = paths_c_robots_to_ch_p[c_name][i]
			path_to_base = paths_c_robots_to_base[c_name][i]

			path_length_to_ch_p = eval_path_length(path_to_ch_p)
			path_length_to_base = eval_path_length(path_to_base)

			path_time_to_ch_p = eval_time_on_path(path_length_to_ch_p)
			path_time_to_base = eval_time_on_path(path_length_to_base)

			docking_time = eval_docking_time(pre_ch_p, ch_p)

			total_time += path_time_to_ch_p

			data = ch_p, total_time
			arrival_times[w_name].append(data)

			total_time += docking_time + TIME_ON_CHARGING + path_time_to_base

	return arrival_times


def eval_waiting_times(times_for_w, times_for_c):
	""" Dict with times that each working robot waiting for the arrival of the charging robot. """
	waiting_times = dict()
	for w_name in times_for_w.keys():
		waiting_times[w_name] = 0
		for i in range(len(times_for_w[w_name])):
			_, arriving_time_w = times_for_w[w_name][i]
			_, arriving_time_c = times_for_c[w_name][i]
			waiting_time = arriving_time_c - arriving_time_w if arriving_time_c - arriving_time_w > 0 else 0
			waiting_times[w_name] += waiting_time
	return waiting_times


def eval_mission_times(paths_w_robots, waiting_times, workpoints):
	mission_times = dict()
	for w_name in paths_w_robots.keys():
		mission_times[w_name] = 0
		for path in paths_w_robots[w_name]:
			path_length = eval_path_length(path)
			time_on_path = eval_time_on_path(path_length)
			mission_times[w_name] += time_on_path
		num_of_workpoints = len(workpoints[w_name])
		mission_times[w_name] += gc_const.TASK_EXEC_DURATION * num_of_workpoints + waiting_times[w_name]
	return mission_times


def prepare_times_dict(paths_w_robots):
	times = dict()
	for w_name in paths_w_robots.keys():
		times[w_name] = list()
	return times


def eval_path_length(path):
	path_length = 0
	for i in range(1, len(path)):
		path_length += eval_distance_between_two_points(path[i-1], path[i])
	return path_length


def eval_distance_between_two_points(p1, p2):
	distance = sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
	return distance


def eval_time_on_path(distance):
	time_on_path = distance/gc_const.MOVEMENT_SPEED
	return time_on_path


def eval_docking_time(pre_ch_p, ch_p):
	distance = eval_distance_between_two_points(pre_ch_p, ch_p)
	distance /= 2  # error accounting
	docking_time = distance/gc_const.DOCKING_SPEED
	return docking_time
	
def get_float_time(seconds):

	total_mins = seconds // 60
	total_secs = (seconds % 60) / 60
	total_time = total_mins + total_secs
	return total_time
	
def write_mission_log(mission_time):
	
	f = open(gc_const.MISSION_LOG_PATH, 'a+')
	t_c = rospy.get_param('real_t_count')
	w_c = rospy.get_param('real_w_count')
	c_c = rospy.get_param('real_c_count')
	#max_cost = rospy.get_param('max_path_cost')
	total_time = get_float_time(mission_time.total_seconds())
	print(total_time)
	f.write('T_num: ' + str(t_c) + ' | W_num: ' + str(w_c) + ' | C_num: ' + str(c_c) + ' | Mission duration: ' + str(total_time) + '\n')
	f.close()
