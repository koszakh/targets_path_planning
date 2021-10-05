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

		trackers = bt.get_battery_trackers(self.names)
		
		for name in trackers.keys():
		
			b_tracker = trackers[name]
			robot_pos, orient = self.mh.get_start_pos(self.start_r[0], self.start_r[1], const.DIST_OFFSET)
			gc.spawn_target(name, robot_pos, orient)
			b_tracker.last_vect = b_tracker.get_robot_orientation_vector()
			start_id, start_pos = self.mh.get_start_vertice_id(robot_pos, b_tracker.last_vect)
			b_tracker.last_p_id = start_id
			b_tracker.start_id = start_id
			
		return trackers
		
	def target_assignment(self):

		iter_count = 0
		path_cost_sum = float('inf')
		s_paths = {}

		print('Workers count: ' + str(self.w_count))
		print('Targets count: ' + str(self.t_count))

		combs = get_sequence_of_des_len(self.names, self.w_count, self.t_count)
		print('Combs count: ' + str(len(combs)))

		for item in combs:

			cur_path_cost_sum = 0
			iter_count += 1
			cur_paths = {}
			cur_workpoints = {}
			cur_goals = {}
			names_cp = copy.copy(list(item))
			cur_worker_names = []
			trackers_cp = copy.deepcopy(self.trackers)
			
			for target_id in self.target_ids:

				name = names_cp[0]
				orig_bt = self.trackers[name]
				b_tracker = trackers_cp[name]
				break_flag = False
					
				if not cur_workpoints.get(name):
				
					cur_workpoints[name] = []
					
				if not cur_goals.get(name):
				
					cur_goals[name] = []


				if not cur_worker_names.__contains__(name):
				
					cur_worker_names.append(name)

				robot_pos = self.mh.heightmap[b_tracker.last_p_id]
				goal = self.mh.heightmap[target_id]
				path_cost = robot_pos.get_distance_to(goal)
				b_tracker.last_p_id = target_id
				cur_path_cost_sum += path_cost
				cur_workpoints[name].append(goal)
				cur_goals[name].append(goal)
				names_cp.pop(0)
			
			for name in cur_worker_names:

				b_tracker = trackers_cp[name]
				
				robot_pos = self.mh.heightmap[b_tracker.last_p_id]
				goal = self.mh.heightmap[b_tracker.start_id]
				path_cost = robot_pos.get_distance_to(goal)
				cur_goals[name].append(goal)

				cur_path_cost_sum += path_cost

			if cur_path_cost_sum < path_cost_sum:

				path_cost_sum = cur_path_cost_sum
				worker_names = copy.copy(cur_worker_names)
				workpoints = copy.copy(cur_workpoints)
				goals = copy.copy(cur_goals)

			print('Iter count: ' + str(iter_count) + ' | Current path cost sum: ' + str(cur_path_cost_sum) + ' | Path cost sum: ' + str(path_cost_sum))
			
		paths = self.calc_task_paths(goals)
				
		charger_names = subtraction_of_set(self.names, worker_names)
		print('\nFinal path cost sum for ' + str(len(worker_names)) + ' workers: ' + str(path_cost_sum))
		print('Worker names: ' + str(worker_names))
		print('Charger names: ' + str(charger_names))
		return paths, workpoints, worker_names, charger_names, self.trackers
		
	def calc_task_paths(self, goals):

		paths = {}
			
		for key in goals.keys():
		
			robot_goals = goals[key]
			b_tracker = self.trackers[key]
			last_id = b_tracker.start_id
			whole_path = []
			
			for goal in robot_goals:
			
				path, path_cost = self.mh.find_path(last_id, goal.id, b_tracker.last_vect)
				last_id = goal.id
				b_tracker.last_vect = get_end_vect(path)
				whole_path.append(path)
				
			paths[key] = copy.copy(whole_path)
			
		return paths
	
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

def calc_current_seq_len(seq_len, mas_len, w_count, iter_num):

	cur_len = seq_len - (w_count * iter_num)
	if cur_len > w_count:

		cur_len = w_count

	return cur_len

def get_permutations(mas, seq_len, w_count, i):

	cur_len = calc_current_seq_len(seq_len, len(mas), w_count, i)
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
