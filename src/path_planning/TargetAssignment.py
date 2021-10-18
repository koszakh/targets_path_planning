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
from scipy.optimize import linear_sum_assignment
from networkx.algorithms.flow import preflow_push

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
		self.names = w_names + c_names
		self.w_count = len(w_names)
		self.t_count = t_count
		self.mh = prepare_hmap()
		self.start_r, self.goal_r = self.calc_start_and_goal_centers()
		self.target_ids = self.prepare_targets()
		self.trackers = get_robot_trackers(self.names)
		self.spawn_workers()
		self.spawn_chargers()
		self.task_matrix = self.get_task_matrix()
	
	def get_target_pos(self, cur_i):
	
		if isinstance(cur_i, int):
		
			task_id = self.target_ids[cur_i]
		
		else:
		
			task_id = cur_i
			
		task_pos = self.mh.heightmap[task_id]
		return task_pos
		
	def get_robots_pos_orient(self):
	
		poses = {}
		orients = {}
		for key in self.trackers.keys():
		
			rt = self.trackers[key]
			poses[key] = rt.get_robot_position()
			orients[key] = rt.get_robot_orientation_vector()
			
		return poses, orients
		
	def get_robot_position(self, name):
	
		robot_pos = self.trackers[name].get_robot_position()
		return robot_pos
		
	def get_task_matrix(self):
	
		#rd = np.empty((2, 3), int)

		t_l = self.t_count
		w_l = self.w_count

		if (t_l > w_l and not t_l % w_l == 0) or (t_l < w_l and not w_l % t_l == 0):

			mat_shape = max(t_l, w_l) + 1

		else:

			mat_shape = max(t_l, w_l)

		task_matrix = np.empty((mat_shape, mat_shape), int)
		cur_i = 0
		cur_j = 0
		
		for i in range(mat_shape):

			for j in range(mat_shape):

				cur_i = i % t_l
				cur_j = j % w_l

				worker_name = self.w_names[cur_j]
				w_pos = self.get_robot_position(worker_name)
				task_pos = self.get_target_pos(cur_i)
				dist = w_pos.get_distance_to(task_pos)
				if i > t_l - 1:

					task_matrix[i][j] = 0

				else:

					task_matrix[i][j] = dist
					
		print('task_matrix:\n' + str(task_matrix))
					
		return task_matrix
	
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
		
	def spawn_workers(self):
	
		for name in self.w_names:

			r_tracker = self.trackers[name]
			robot_pos, orient = self.mh.get_start_pos(self.start_r[0], self.start_r[1], const.START_DIST_OFFSET)
			gc.spawn_worker(name, robot_pos, orient)
			r_tracker.last_vect = r_tracker.get_robot_orientation_vector()
			start_id, start_pos = self.mh.get_start_vertice_id(robot_pos, r_tracker.last_vect)
			r_tracker.last_p_id = start_id
			r_tracker.start_id = start_id
		
	def spawn_chargers(self):
	
		for name in self.c_names:
		
			robot_pos, orient = self.mh.get_start_pos(self.start_r[0], self.start_r[1], const.START_DIST_OFFSET)
			gc.spawn_charger(name, robot_pos, orient)
		
	def hungary(self):
	
		b = self.task_matrix.copy()

		for i in range(len(b)):
		
			row_min = np.min(b[i])
		
			for j in range(len(b[i])):
		
				b[i][j] -= row_min
		
		for i in range(len(b[0])):
		
			col_min = np.min(b[:, i])
		
			for j in range(len(b)):
		
				b[j][i] -= col_min
		
		line_count = 0
		
		while (line_count < len(b)):
			
			line_count = 0
			row_zero_count = []
			col_zero_count = []
			
			for i in range(len(b)):
			
				row_zero_count.append(np.sum(b[i] == 0))
			
			for i in range(len(b[0])):

				col_zero_count.append((np.sum(b[:, i] == 0)))
				
			line_order = []
			row_or_col = []

			for i in range(len(b[0]), 0, -1):

				while (i in row_zero_count):

					line_order.append(row_zero_count.index(i))
					row_or_col.append(0)
					row_zero_count[row_zero_count.index(i)] = 0

				while (i in col_zero_count):

					line_order.append(col_zero_count.index(i))
					row_or_col.append(1)
					col_zero_count[col_zero_count.index(i)] = 0

			delete_count_of_row = []
			delete_count_of_rol = []
			row_and_col = [i for i in range(len(b))]
			
			for i in range(len(line_order)):
			
				if row_or_col[i] == 0:
			
					delete_count_of_row.append(line_order[i])
			
				else:
			
					delete_count_of_rol.append(line_order[i])
			
				c = np.delete(b, delete_count_of_row, axis=0)
				c = np.delete(c, delete_count_of_rol, axis=1)
				line_count = len(delete_count_of_row) + len(delete_count_of_rol)
				
				if line_count == len(b):
					
					break
					
				if 0 not in c:
			
					row_sub = list(set(row_and_col) - set(delete_count_of_row))
					min_value = np.min(c)
			
					for i in row_sub:
			
						b[i] = b[i] - min_value
			
					for i in delete_count_of_rol:
			
						b[:, i] = b[:, i] + min_value
			
					break
		
		row_ind, col_ind = linear_sum_assignment(b)
		return row_ind, col_ind
		
	def target_assignment(self):

		row_ind, col_ind = self.hungary()
		
		assigned_pairs = [(row_ind[i], col_ind[i]) for i in range(self.t_count)][:self.t_count]
		
		workpoints = self.get_workpoints_dict(assigned_pairs)
		paths, break_flag = self.calc_task_paths(workpoints)
		
		if not break_flag:

			return paths, workpoints
			
		else:
		
			print('Tasks cant be reached!')
			return None, None
		
	def get_workpoints_dict(self, assigned_pairs):
	
		workpoints = {}

		for pair in assigned_pairs:

			w_ind = pair[0] % self.w_count
			t_ind = pair[1] % self.t_count
			w_name = self.w_names[w_ind]
			target = self.get_target_pos(t_ind)
			
			if workpoints.get(w_name):
			
				workpoints[w_name].append(target)
				
			else:
			
				workpoints[w_name] = [target]
		
		print('Number of tasks per robot:\n')		
		
		for key in workpoints.keys():
		
			w_mas = workpoints[key]
			print(key, len(w_mas))
			workpoints[key] = self.sort_tasks(w_mas, key)
			
		return workpoints
	
	def get_best_per(self):
	
		per = self.all_pers[0]
		self.all_pers.pop(0)
		return per
	
	def sort_tasks(self, per, robot_name):
	
		robot_pos = self.get_robot_position(robot_name)
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
		
			print(key)	
			robot_goals = goals[key]
			paths[key] = self.calc_task_path(key, robot_goals)
			print(len(paths[key]))
			if not paths[key]:

				return {}, True

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
				
				return []
				
		path, path_cost = self.mh.find_path(last_id, r_tracker.start_id, last_vect)
		whole_path.append(path)

		return whole_path
		
def clean_dict(d):

	new_d = {}

	for key in d.keys():
	
		if d[key]:
		
			new_d[key] = d[key]
			
	return new_d

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

def calc_task_seq_len(seq_len, w_count, rem_workers_num):

	td = int(seq_len / w_count)
	td_mod = seq_len % w_count

	if rem_workers_num > td_mod:

		cur_len = td

	else:

		cur_len = td + 1

	return cur_len

def get_permutations(mas, seq_len, w_count, i):

	cur_len = calc_current_seq_len(seq_len, w_count, i)
	init_per_list = itertools.permutations(mas, cur_len)
	per_list = []

	for item in init_per_list:

		per_list.append(list(item))

	return per_list
	
def get_task_combinations(mas, seq_len, w_count, rem_workers):

	cur_len = calc_task_seq_len(seq_len, w_count, rem_workers)
	init_per_list = itertools.combinations(mas, cur_len)
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
