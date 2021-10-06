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
from ORCA import ORCAsolver
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

	def __init__(self, w_count, c_count, t_count):
	
		self.w_count = w_count
		self.c_count = c_count
		self.robots_count = w_count + c_count
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
	
		offset = const.DIST_OFFSET
		
		avg_x = np.mean([self.mh.min_x, self.mh.max_x])
		avg_y = np.mean([self.mh.min_y, self.mh.max_y])
		
		start = (avg_x - 30, avg_y - 30)
		goal = (avg_x + 30, avg_y + 30)
		
		return start, goal
	
	def prepare_targets(self):
		
		target_ids = self.mh.get_random_ids_in_area(self.goal_r[0], self.goal_r[1], const.DIST_OFFSET, self.t_count)
		
		targets = [self.mh.heightmap[v_id] for v_id in target_ids]
		gc.visualise_path(targets, gc_const.BIG_GREEN_VERTICE_PATH, 'target')
			
		return target_ids

	def prepare_trackers(self):

		trackers = get_robot_trackers(self.names)
		
		for name in trackers.keys():
		
			r_tracker = trackers[name]
			robot_pos, orient = self.mh.get_start_pos(self.start_r[0], self.start_r[1], const.DIST_OFFSET)
			gc.spawn_target(name, robot_pos, orient)
			r_tracker.last_vect = r_tracker.get_robot_orientation_vector()
			start_id, start_pos = self.mh.get_start_vertice_id(robot_pos, r_tracker.last_vect)
			r_tracker.last_p_id = start_id
			r_tracker.start_id = start_id
			
		return trackers
		
	def target_assignment(self):

		robot_per = {}
		best_per = {}
		combs = get_permutations(self.names, len(self.names), self.w_count, 0)
		best_paths_cost = float('inf')
		for comb in combs:

			rem_workers_count = self.w_count
			rem_targets = copy.copy(self.target_ids)
			closed_targets = []
			cur_best_per = {}
			cur_paths_cost = 0
			cur_workpoints = {}
			for name in comb:

				pers = get_task_permutations(rem_targets, self.t_count, self.w_count, rem_workers_count)
				robot_per[name] = []
				init_rt = self.trackers[name]
				init_r_pos = self.mh.heightmap[init_rt.start_id]

				for per in pers:
				
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

					robot_per[name].append((per, cur_cost))

				new_mas = sort_tuple_mas(robot_per[name])
				cur_best_per[name] = get_best_free_config(closed_targets, new_mas)
				rem_targets = difference_of_sets(rem_targets, cur_best_per[name][0])
				closed_targets.extend(cur_best_per[name][0])
				cur_workpoints[name] = cur_best_per[name][0]
				cur_paths_cost += cur_best_per[name][1]
				rem_workers_count -= 1


			if cur_paths_cost < best_paths_cost:

				best_per = copy.copy(cur_best_per)
				best_paths_cost = cur_paths_cost
				workpoints = copy.copy(cur_workpoints)

			#print(comb, cur_paths_cost, best_paths_cost)

		paths = self.calc_task_paths(best_per)
		w_names = best_per.keys()
		c_names = subtraction_of_set(self.names, w_names)
		print('Worker names: ' + str(w_names))
		print('Charger names: ' + str(c_names))

		#print('\n>>> Best combination <<<\n')
		# for key in best_per.keys():
		#
		#	 print(key, best_per[key])
		return paths, workpoints, w_names, c_names
		
	def calc_task_paths(self, goals):

		paths = {}
			
		for key in goals.keys():
		
			robot_goals = goals[key][0]
			r_tracker = self.trackers[key]
			last_id = r_tracker.start_id
			whole_path = []
			
			for goal in robot_goals:
			
				path, path_cost = self.mh.find_path(last_id, goal, r_tracker.last_vect)
				last_id = goal
				r_tracker.last_vect = get_end_vect(path)
				whole_path.append(path)
				
			paths[key] = copy.copy(whole_path)
			
		return paths
		
def get_best_free_config(closed_targets, pers):

	per = None
	while not per and len(pers) > 0:

		copy_pers = copy.copy(pers)
		cur_per = copy_pers[0]
		#print(cur_per)
		pers.pop(0)
		cur_targets = cur_per[0]

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
	
def get_sequence_of_des_len(mas, w_count, seq_len):

	total_per_list = []
	

	count = int(seq_len / w_count)
	
	if seq_len % w_count > 0:

		count += 1

	sub_per_lists = [get_permutations(mas, seq_len, w_count, i) for i in range(0, count)]
	indices_count = []
	for sub_per_list in sub_per_lists:

		indices_count.append(len(sub_per_list))

	seq_list = get_sequence_list(indices_count)

	for seq in seq_list:

		ids_seq = []

		for i in range(len(sub_per_lists)):

			cur_per_ind = seq[i]
			cur_seq_list = sub_per_lists[i]
			cur_per = cur_seq_list[cur_per_ind]
			ids_seq.extend(cur_per)

		un_el_count = len(set(ids_seq))
		
		if un_el_count <= w_count:
		
			total_per_list.append(ids_seq)

	return total_per_list

def get_sequence_list(num_mas):

	new_mas = []
	m_mas = []

	for i in range(len(num_mas)):

		m = num_mas[i]
		m_mas.append([])

		for j in range(m):

			m_mas[i].append(j)

	prod = []
	init_prod = itertools.product(*m_mas)
	print([len(x) for x in m_mas])
	for item in init_prod:

		prod.append(list(item))
		#print(item)

	return prod

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

def remove_mas_repeats(mas):

	copy_mas = copy.copy(mas)

	for i in range(len(mas)):

		per1 = mas[i]

		for j in range(i, len(mas)):

			per2 = mas[j]
			if compare_lists(per1, per2):

				if per2 in copy_mas and copy_mas.count(per2) > 1:

					copy_mas.remove(per2)

	return copy_mas

def remove_repeats(mas):

	new_mas = new_x = [el for el, _ in itertools.groupby(mas)]
	return new_mas

def compare_lists(mas1, mas2):

	if functools.reduce(lambda x, y: x and y, map(lambda p, q: p == q, mas1, mas2), True):

		return True

	else:

		return False
		
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

			cost = item[1]
			if cost < min_cost:

				min_cost = cost
				cur_item = item

		mas.remove(cur_item)
		new_mas.append(cur_item)

	return new_mas
