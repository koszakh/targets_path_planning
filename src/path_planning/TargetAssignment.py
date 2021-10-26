# Target assignment module
import PathPlanner as pp
import rospy
import copy
import numpy as np
from targets_path_planning.msg import Path
from Point import Point
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
from math import sqrt
from scipy.optimize import linear_sum_assignment
import io

class RobotTracker():

	def __init__(self, name):
	
		self.name = name
		self.last_p_id = None
		self.last_vect = None
		self.start_id = None
		self.init_vect = None
		self.path_cost = 0
		
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
		self.mh = prepare_hmap()
		self.start_r, self.goal_r = self.calc_start_and_goal_centers()
		self.print_areas_dist()
		self.trackers = get_robot_trackers(self.names)
		self.prepare_mission_from_file(t_count)
	
	def prepare_mission_from_file(self, t_count):
	
		self.prepare_targets_from_file(t_count)
		self.spawn_workers_from_file()
		self.spawn_chargers_from_file()
		
	def prepare_mission(self, t_count):
	
		self.prepare_targets(t_count)
		self.spawn_workers()
		self.spawn_chargers()
	
	def get_point_pos(self, ind):
	
		if isinstance(ind, int):
		
			p_id = self.target_ids[ind]
		
		else:
		
			p_id = ind
			
		p = self.mh.heightmap[p_id]
		return p
		
	def get_point_id(self, target):
	
		t_id = self.mh.get_nearest_vertice_id(target.x, target.y)
		return t_id

	def print_areas_dist(self):
	
		sr_x = self.start_r[0]
		sr_y = self.start_r[1]
		s = Point(sr_x, sr_y, 0)
		gr_x = self.goal_r[0]
		gr_y = self.goal_r[1]
		g = Point(gr_x, gr_y, 0)
		dist = g.get_2d_distance(s)
		print('\nAreas dist: ' + str(dist) + ' m')

		
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
		
	def get_task_matrix(self, target_ids):

		t_l = len(target_ids)
		w_l = self.w_count

		if t_l % w_l > 0:
		
		    mod_c = 1  
		
		else:
		
			mod_c = 0
		
		mat_shape = max(t_l, w_l)#((t_l // w_l) + mod_c) * w_l
		print('\nMatrix dimensions: [' + str(mat_shape) + 'x' + str(mat_shape) + ']')
		
		task_matrix = np.empty((mat_shape, mat_shape), int)
		cur_i = 0
		cur_j = 0
		
		for i in range(mat_shape):

			line_mas = []

			for j in range(mat_shape):

				if i > t_l - 1:

					task_matrix[i][j] = 0

				else:

					cost = self.get_cell_cost(i, j, target_ids)
					task_matrix[i][j] = cost
					
		print('task_matrix:\n' + str(task_matrix))
					
		return task_matrix
	
	def get_cell_cost(self, i, j, target_ids):
	
		worker_name = self.w_names[j]
		rt = self.trackers[worker_name]
		last_vect = rt.last_vect
		start_id = rt.last_p_id
		start_p = self.mh.heightmap[start_id]
		goal_id = target_ids[i]
		path, path_cost = self.mh.find_path(start_id, goal_id, last_vect)
		total_cost = path_cost + rt.path_cost 
		
		return total_cost
	
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
	
	def prepare_targets(self, t_count):
		
		self.target_ids = self.mh.get_random_ids_in_area(self.goal_r[0], self.goal_r[1], const.GOAL_DIST_OFFSET, t_count)
		
		targets = [self.get_point_pos(p_id) for p_id in self.target_ids]
		#gc.visualise_path(targets, gc_const.BIG_GREEN_VERTICE_PATH, 'task')
		
		save_targets(self.target_ids)
		
	def prepare_targets_from_file(self, t_count):
		
		self.target_ids = get_target_ids_from_file(t_count)
		
	def spawn_workers(self):
	
		poses = []
	
		for name in self.w_names:

			r_tracker = self.trackers[name]
			robot_pos, orient = self.mh.get_start_pos(self.start_r[0], self.start_r[1], const.START_DIST_OFFSET)
			poses.append((robot_pos, orient))
			gc.spawn_worker(name, robot_pos, orient)
			r_tracker.init_vect = r_tracker.last_vect = r_tracker.get_robot_orientation_vector()
			start_id, start_pos = self.mh.get_start_vertice_id(robot_pos, r_tracker.last_vect)
			r_tracker.last_p_id = r_tracker.start_id = start_id
			
		save_workers(poses)
		
	def spawn_workers_from_file(self):
	
		robot_poses = get_worker_poses_from_file(self.w_count)
	
		for name in self.w_names:

			r_tracker = self.trackers[name]
			robot_state = robot_poses[0]
			robot_pos = robot_state[0]
			orient = robot_state[1]
			robot_poses.pop(0)
			gc.spawn_worker(name, robot_pos, orient)
			r_tracker.init_vect = r_tracker.last_vect = r_tracker.get_robot_orientation_vector()
			start_id, start_pos = self.mh.get_start_vertice_id(robot_pos, r_tracker.last_vect)
			r_tracker.last_p_id = r_tracker.start_id = start_id
		
	def spawn_chargers(self):
	
		poses = []
		
		for name in self.c_names:
		
			robot_pos, orient = self.mh.get_start_pos(self.start_r[0], self.start_r[1], const.START_DIST_OFFSET)
			poses.append((robot_pos, orient))
			gc.spawn_charger(name, robot_pos, orient)
			
		save_chargers(poses)
		
	def spawn_chargers_from_file(self):
		
		robot_poses = get_charger_poses_from_file(len(self.c_names))
		
		for name in self.c_names:
		
			robot_state = robot_poses[0]
			robot_pos = robot_state[0]
			orient = robot_state[1]
			robot_poses.pop(0)
			gc.spawn_charger(name, robot_pos, orient)
		
	def set_new_targets_list(self, workpoints):
	
		wp_ids = self.get_point_ids_from_dict(workpoints)
		self.target_ids = wp_ids
		
	def hungary(self, target_ids):
	
		s_time = time.time()
		#b = self.task_matrix.copy()
		b = self.get_task_matrix(target_ids)
		
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

		rem_targets = copy.copy(self.target_ids)
		workpoints = {}
		
		while len(rem_targets) > 0:

			if len(rem_targets) > self.w_count:
			
				t_count = self.w_count
				
			else:
			
				t_count = len(rem_targets)

			cur_targets = self.get_closest_tasks(rem_targets, t_count)
			#print('\nCurrent targets: ' + str(cur_targets))
			rem_targets = difference_of_lists(rem_targets, cur_targets)
			row_ind, col_ind = self.hungary(cur_targets)
			
			assigned_pairs = [(row_ind[i], col_ind[i]) for i in range(len(row_ind))][:len(cur_targets)]
			
			#print('assigned_pairs: ' + str(assigned_pairs))
			new_workpoints = self.get_workpoints_dict(assigned_pairs, cur_targets)
			workpoints = supplement_dict(workpoints, new_workpoints)
			comp_flag = self.move_robot_trackers(new_workpoints)
			
			if not comp_flag:
			
				self.clean_path_costs()
				return None

		self.clean_path_costs()
		
		return workpoints

		
	def move_robot_trackers(self, w_points):
	
		comp_flag = True
	
		for key in w_points.keys():	
			
			rt = self.trackers[key]
			wp_ids = [p.id for p in w_points[key]]
			#print(key, wp_ids)
			wp = w_points[key][-1]
			#print(key, rt.last_p_id, wp.id)
			last_id = rt.last_p_id
			last_vect = rt.last_vect
			start_p = self.get_point_pos(last_id)
			dist = start_p.get_distance_to(wp)
			#print('dist: ' + str(dist))
			path, path_cost = self.mh.find_path(last_id, wp.id, last_vect)
			rt.last_p_id = wp.id
			
			if path_cost:
			
				rt.path_cost += path_cost
				rt.last_vect = get_end_vect(path)
				
			else:
			
				comp_flag = False
				break
		
		return comp_flag
				
		
	def get_closest_tasks(self, rem_tasks, t_count):
	
		closest_tasks = []
		start_p = Point(self.start_r[0], self.start_r[1], 0)
		while len(closest_tasks) < t_count:
		
			min_dist = float('inf')
			for task_id in rem_tasks:
			
				task_p = self.mh.heightmap[task_id]
				dist = task_p.get_2d_distance(start_p)
				if dist < min_dist and not task_id in closest_tasks:
				
					min_dist = dist
					closest_id = task_id
					
			closest_tasks.append(closest_id)
			
		return closest_tasks
				
		
	def get_workpoints_dict(self, assigned_pairs, target_ids):
	
		workpoints = {}
		t_count = len(target_ids)

		for pair in assigned_pairs:

			t_ind, w_ind = pair
			
			t_id = target_ids[t_ind]
			w_name = self.w_names[w_ind]
			target = self.get_point_pos(t_id)
			
			if workpoints.get(w_name):
			
				workpoints[w_name].append(target)
				
			else:
			
				workpoints[w_name] = [target]
		
		for key in workpoints.keys():
		
			w_mas = workpoints[key]
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
		all_paths_cost = 0
		real_w_c = len(goals.keys())
		max_path_cost = 0
			
		for key in goals.keys():
	
			robot_goals = goals[key]
			paths[key], path_cost = self.calc_task_path(key, robot_goals)
			if not paths[key]:

				print('Tasks cant be reached!')
				return {}, True
				
			all_paths_cost += path_cost
			if path_cost > max_path_cost:
			
				max_path_cost = path_cost
			
		print('\nWorkers paths cost: ' + str(all_paths_cost))
		#rospy.set_param('workers_path_cost', all_paths_cost)
		rospy.set_param('max_path_cost', max_path_cost)

		return paths, break_flag
		
	def calc_task_path(self, name, robot_goals):
		
		r_tracker = self.trackers[name]
		last_vect = r_tracker.last_vect
		last_id = r_tracker.start_id
		whole_path = []
		break_flag = False
		total_path_cost = 0
		
		for goal in robot_goals:
		
			goal_id = self.get_point_id(goal)
			path, path_cost = self.mh.find_path(last_id, goal_id, last_vect)
			
			if path:

				last_id = goal_id
				last_vect = get_end_vect(path)
				whole_path.append(path)
				total_path_cost += path_cost
				
			else:
				
				return [], None
				
		path, path_cost = self.mh.find_path(last_id, r_tracker.start_id, last_vect)
		whole_path.append(path)

		return whole_path, total_path_cost
		
	def get_point_ids_from_dict(self, points_dict):
	
		wp_ids = []
		for key in points_dict.keys():
		
			w_points = points_dict[key]
			for wp in w_points:
			
				wp_id = self.get_point_id(wp)
				wp_ids.append(wp_id)
				
		return wp_ids
		
	def clean_path_costs(self):
	
		for key in self.trackers.keys():
		
			rt = self.trackers[key]
			rt.path_cost = 0
			rt.last_vect = rt.init_vect
			rt.last_p_id = rt.start_id
				
		
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

def difference_of_lists(m1, m2):

	m = list(set(m1) - set(m2))
	if m:

		return m

	else:

		return []
	
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
	
def save_targets(target_ids):

	f = open(const.TARGETS_LOG_PATH, 'w+')

	for t_id in target_ids:
	
		f.write(str(t_id) + '\n')

	f.close()
	
def save_workers(w_poses):

	f = open(const.WORKERS_LOG_PATH, 'w+')

	for w_pos in w_poses:
	
		orient_s = make_num_str(w_pos[1])
		f.write(str(w_pos[0]) + ', ' + orient_s + '\n')

	f.close()
	
def save_chargers(c_poses):

	f = open(const.CHARGERS_LOG_PATH, 'w+')

	for c_pos in c_poses:
	
		orient_s = make_num_str(c_pos[1])
		f.write(str(c_pos[0]) + ', ' + orient_s + '\n')

	f.close()
	
def get_target_ids_from_file(t_count):

	t_ids = []

	with io.open(const.TARGETS_LOG_PATH, encoding='utf-8') as file:
			
		for line in file:

			s = line[line.find('('):line.rfind(')')]
			indices = [item for item in s.split("'") if try_float(item) == float]
			col = indices[0]
			row = indices[1]
			t_id = (str(col), str(row))
			t_ids.append(t_id)
			
			if len(t_ids) == t_count:
			
				break
				
	return t_ids
	
def get_worker_poses_from_file(w_count):

	w_poses = []

	with io.open(const.WORKERS_LOG_PATH, encoding='utf-8') as file:
			
		for line in file:

			state = line.split(', ')
			pos = state[0].split(' ')
			x = float(pos[0])
			y = float(pos[1])
			z = float(pos[2])
			p = Point(x, y, z)
			orient_coords = state[1].split(' ')
			rot = [float(c) for c in orient_coords if try_float(c) == float]
			w_poses.append((p, rot))
			
			if len(w_poses) == w_count:
			
				break
				
	return w_poses
	
def get_charger_poses_from_file(c_count):

	c_poses = []

	with io.open(const.CHARGERS_LOG_PATH, encoding='utf-8') as file:
			
		for line in file:

			state = line.split(', ')
			pos = state[0].split(' ')
			x = float(pos[0])
			y = float(pos[1])
			z = float(pos[2])
			p = Point(x, y, z)
			orient_coords = state[1].split(' ')
			rot = [float(c) for c in orient_coords if try_float(c) == float]
			c_poses.append((p, rot))
			
			if len(c_poses) == c_count:
			
				break
				
	return c_poses
	
def supplement_dict(init_dict, sub_dict):

	f_dict = copy.copy(init_dict)
	
	for key in sub_dict.keys():
	
		if f_dict.get(key):
		
			f_dict[key].extend(sub_dict[key])
			
		else:
		
			f_dict[key] = sub_dict[key]
	
	return f_dict
	
def try_float(x):

	try:

		fl_x = float(x)
		return type(fl_x)

	except ValueError:

		return type(x)
		
def get_points_count(w_dict):

	count = 0

	for key in w_dict.keys():
	
		count += len(w_dict[key])
		
	return count
	
def make_num_str(points):

	s = ''
	
	for p in points:
	
		s += str(p) + ' '
		
	return s
