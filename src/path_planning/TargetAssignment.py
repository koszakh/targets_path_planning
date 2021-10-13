# Target assignment module
import PathPlanner as pp
import rospy
import copy
import numpy as np
from targets_path_planning.msg import Path
from geometry_msgs.msg import Point
from Heightmap import Heightmap
import Constants as const
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
import gazebo_communicator.BatteryTracker as bt
from MovementManager import MovementManager
import random
import itertools
import functools
import time

class RobotTracker():

	def __init__(self, name):
	
		self.name = name
		self.last_p_id = None
		self.last_vect = None
		self.start_id = None
		
	def get_robot_position(self):
	
		robot_pose = gc.get_model_position(self.name)
		return robot_pose

	def get_robot_orientation_vector(self):

		vect = gc.get_robot_orientation_vector(self.name)
		return vect

class TargetAssignment():

	def __init__(self, w_names, c_names, t_count):
	
		self.w_names = w_names
		self.c_names = c_names
		self.robots_count = len(self.w_names) + len(self.c_names)
		self.t_count = t_count
		self.mh = prepare_hmap()
		self.start_r, self.goal_r = self.calc_start_and_goal_centers()
		self.names = ['sim_p3at' + str(i) for i in range(1, self.robots_count + 1)]
		self.target_ids = self.prepare_targets()
		self.trackers = self.prepare_trackers()

		
	def get_robots_pos_orient(self, names):
	
		poses = {}
		orients = {}
		for name in names:
		
			rt = self.trackers[name]
			poses[name] = rt.get_robot_position()
			orients[name] = rt.get_robot_orientation_vector()
			
		return poses, orients
	
	def calc_start_and_goal_centers(self):
		
		avg_x = np.mean([self.mh.min_x, self.mh.max_x])
		avg_y = np.mean([self.mh.min_y, self.mh.max_y])
		
		s_x = const.S_X_OFFSET
		s_y = const.S_Y_OFFSET
		g_x = const.G_X_OFFSET
		g_y = const.G_Y_OFFSET
		
		start = (avg_x + s_x, avg_y + s_y)
		goal = (avg_x + g_x, avg_y + g_y)
		
		return start, goal
	
	def prepare_targets(self):
		
		target_ids = self.mh.get_random_ids_in_area(self.goal_r[0], self.goal_r[1], const.GOAL_DIST_OFFSET, self.t_count)
		
		targets = [self.mh.heightmap[v_id] for v_id in target_ids]
			
		return target_ids

	def prepare_trackers(self):

		trackers = get_robot_trackers(self.names)
		
		for name in trackers.keys():

			r_tracker = trackers[name]
			robot_pos, orient = self.mh.get_start_pos(self.start_r[0], self.start_r[1], const.START_DIST_OFFSET)
			gc.spawn_target(name, robot_pos, orient)
			r_tracker.last_vect = r_tracker.get_robot_orientation_vector()
			start_id, start_pos = self.mh.get_start_vertice_id(robot_pos, r_tracker.last_vect)
			r_tracker.last_p_id = start_id
			r_tracker.start_id = start_id
			
		return trackers
		
	def target_assignment(self):

		print('Target assignment started.')
		robot_per = {}
		best_per = {}
		total_pers = []
		combs = get_permutations(self.w_names, len(self.w_names), len(self.w_names), 0)
		best_paths_cost = float('inf')
		for comb in combs:

			rem_workers_count = len(self.w_names)
			rem_targets = copy.copy(self.target_ids)
			closed_targets = []
			cur_best_per = {}
			cur_paths_cost = 0
			cur_workpoints = {}
			for name in comb:

				pers = get_task_permutations(rem_targets, self.t_count, len(self.w_names), rem_workers_count)
				robot_per[name] = []
				init_rt = self.trackers[name]
				init_r_pos = self.mh.heightmap[init_rt.start_id]

				for per in pers:
				
					print(name, per)
					rt = copy.deepcopy(self.trackers[name])
					robot_pos = init_r_pos
					cur_cost = 0

					for target_id in per:

						target = self.mh.heightmap[target_id]
						cost = robot_pos.get_distance_to(target)
						robot_pos = target
						cur_cost += cost

					cost = robot_pos.get_distance_to(init_r_pos)
					cur_cost += cost

					robot_per[name].append((cur_cost, per))

				new_mas = sort_tuple_mas(robot_per[name])
				cur_best_per[name] = get_best_free_config(closed_targets, new_mas)
				rem_targets = difference_of_sets(rem_targets, cur_best_per[name][1])
				cur_wp_ids = cur_best_per[name][1]
				closed_targets.extend(cur_wp_ids)
				cur_workpoints[name] = [self.mh.heightmap[cur_id] for cur_id in cur_wp_ids]
				cur_paths_cost += cur_best_per[name][0]
				rem_workers_count -= 1
				total_pers.append((cur_paths_cost, cur_workpoints))

			if cur_paths_cost < best_paths_cost:

				best_per = copy.copy(cur_best_per)
				best_paths_cost = cur_paths_cost
				workpoints = copy.copy(cur_workpoints)

		self.all_pers = sort_tuple_mas(total_pers)

		break_flag = True
		
		while break_flag:

			best_per = self.get_best_per()
			paths, break_flag = self.calc_task_paths(best_per[1])

		#print('\n>>> Best combination <<<\n')
		w_points = {}
		for key in self.w_names:

			#print(key, best_per[key])
			workpoints = best_per[1][key]
			new_per = self.sort_tasks(workpoints, key)
			w_points[key] = new_per

		return paths, w_points
	
	def get_best_per(self):
	
		per = self.all_pers[0]
		self.all_pers.pop(0)
		return per
	
	def sort_tasks(self, per, robot_name):
	
		robot_pos = self.trackers[robot_name].get_robot_position()
		new_per = []
		
		while len(per) > 0:
		
			min_dist = float('inf')
		
			for target in per:
			
				dist = robot_pos.get_distance_to(target)
				if dist < min_dist:
				
					cur_target = target
					min_dist = dist
					
			new_per.append(cur_target)
			per.remove(cur_target)
			
		return new_per
	
	def calc_task_paths(self, goals):

		paths = {}
		break_flag = False
			
		for key in goals.keys():
		
			
			robot_goals = goals[key]
			paths[key] = self.calc_task_path(key, robot_goals)
			
			if not paths[key]:

				break_flag = True
				break

		return paths, break_flag
		
	def calc_task_path(self, name, robot_goals):
		
		r_tracker = self.trackers[name]
		last_vect = r_tracker.last_vect
		last_id = r_tracker.start_id
		whole_path = []
		break_flag = False
		
		for goal in robot_goals:
		
			goal_id = self.mh.get_nearest_vertice_id(goal.x, goal.y)
			path, path_cost = self.mh.find_path(last_id, goal_id, last_vect)
			
			if path:

				last_id = goal_id
				last_vect = get_end_vect(path)
				whole_path.append(path)
				
			else:
				
				break_flag = True
				break
				
		if not break_flag:
			
			path, path_cost = self.mh.find_path(last_id, r_tracker.start_id, last_vect)
			whole_path.append(path)

		return whole_path
		
def get_best_free_config(closed_targets, pers):

	per = None
	while not per and len(pers) > 0:

		copy_pers = copy.copy(pers)
		cur_per = copy_pers[0]
		#print(cur_per)
		pers.pop(0)
		cur_targets = cur_per[1]

		if not mas_intersection(cur_targets, closed_targets):

			per = cur_per

	return per
		
def mas_intersection(m1, m2):

	m = list(set(m1) & set(m2))
	if m:

		return True

	else:

		return False		

def difference_of_sets(m1, m2):

	m = list(set(m1) - set(m2))
	if m:

		return m

	else:

		return None
	
def get_end_vect(path):

	goal = path[len(path) - 1]
	last_goal = path[len(path) - 2]
	last_vect = last_goal.get_dir_vector_between_points(goal)
	return last_vect
	
def prepare_hmap():

	hm = Heightmap(const.HEIGHTMAP_SDF_PATH)
	hmap, l_scale, w_scale, x_step, y_step, step_count = hm.prepare_heightmap()
	mh = pp.PathPlanner(hmap, l_scale, w_scale, x_step, y_step, step_count)
	mh.gridmap_preparing()
	return mh

def calc_current_seq_len(seq_len, w_count, iter_num):

	cur_len = seq_len - (w_count * iter_num)
	if cur_len > w_count:

		cur_len = w_count

	return cur_len

def calc_task_seq_len(seq_len, w_count, rem_workers_num):

	tasks_count = int(seq_len / w_count)
	if rem_workers_num > 1:

		cur_len = tasks_count

	else:

		cur_len = tasks_count + 1

	return cur_len

def get_permutations(mas, seq_len, w_count, i):

	cur_len = calc_current_seq_len(seq_len, w_count, i)
	init_per_list = itertools.permutations(mas, cur_len)
	per_list = []

	for item in init_per_list:

		per_list.append(list(item))

	return per_list
	
def get_task_permutations(mas, seq_len, w_count, rem_workers):

	cur_len = calc_task_seq_len(seq_len, w_count, rem_workers)
	init_per_list = itertools.permutations(mas, cur_len)
	per_list = []

	for item in init_per_list:

		per_list.append(list(item))

	return per_list

		
def subtraction_of_set(mas1, mas2):

	new_mas = []
	
	for n in mas1:
	
		if not mas2.__contains__(n):
		
			new_mas.append(n)

	return new_mas
	
def get_robot_trackers(names):

	r_trackers = {}
	
	for name in names:
	
		rt = RobotTracker(name)
		r_trackers[name] = rt
		
	return r_trackers
	
def sort_tuple_mas(mas):

	new_mas = []

	while len(mas) > 0:

		copy_mas = copy.copy(mas)
		min_cost = float('inf')

		for item in copy_mas:

			cost = item[0]
			if cost < min_cost:

				min_cost = cost
				cur_item = item

		mas.remove(cur_item)
		new_mas.append(cur_item)

	return new_mas
