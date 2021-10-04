# Target assignment module
import PathPlanner as pp
import rospy
import copy
import numpy as np
from targets_path_planning.msg import AllPaths, Path
from geometry_msgs.msg import Point
from Heightmap import Heightmap
import Constants as const
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
from gazebo_communicator.BatteryTracker import BatteryTracker
from ORCA import ORCAsolver
import random
import itertools
import functools
import time

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

def prepare_hmap():

	hm = Heightmap(const.HEIGHTMAP_SDF_PATH)
	hmap, l_scale, w_scale, x_step, y_step, step_count = hm.prepare_heightmap()
	mh = pp.PathPlanner(hmap, l_scale, w_scale, x_step, y_step, step_count)
	mh.gridmap_preparing()
	return mh

def prep_targets(t_count):

	mh = prepare_hmap()
	
	offset = const.DIST_OFFSET
	
	avg_x = np.mean([mh.min_x, mh.max_x])
	avg_y = np.mean([mh.min_y, mh.max_y])
	
	start = (avg_x - 30, avg_y - 30)
	goal = (avg_x + 30, avg_y + 30)
	
	target_ids = mh.get_random_ids_in_area(goal[0], goal[1], offset, t_count)
	
	targets = [mh.heightmap[v_id] for v_id in target_ids]
	gc.visualise_path(targets, gc_const.BIG_GREEN_VERTICE_PATH, 'target')
	
	paths = {}
	trackers = {}
	
	for i in range(1, const.ROBOTS_COUNT + 1):
	
		name = 'sim_p3at' + str(i)
		robot_pos, orient = mh.get_start_pos(start[0], start[1], offset)
		gc.spawn_target(name, robot_pos, orient)
		bt = BatteryTracker(name)
		start_id, start_pos = mh.get_start_vertice_id(robot_pos, bt.last_vect)
		bt.last_p_id = start_id
		bt.start_id = start_id
		trackers[name] = bt
		
	return mh, trackers, target_ids
	
def target_assignment(mh, workers_count, chargets_count, trackers, target_ids):

	names = copy.copy(trackers.keys())
	iter_count = 0
	path_cost_sum = float('inf')
	s_paths = {}

	print('Workers count: ' + str(workers_count))
	print('Targets count: ' + str(len(target_ids)))

	combs = get_sequence_of_des_len(names, workers_count, len(target_ids))
	print('Combs count: ' + str(len(combs)))

	for item in combs:

		cur_path_cost_sum = 0
		iter_count += 1
		cur_paths = {}
		cur_workpoints = {}
		names_cp = copy.copy(list(item))
		cur_worker_names = []
		trackers_cp = copy.deepcopy(trackers)
		
		for target_id in target_ids:

			name = names_cp[0]
			orig_bt = trackers[name]
			bt = trackers_cp[name]
			break_flag = False
			
			if not cur_paths.get(name):
			
				cur_paths[name] = []
				
			if not cur_workpoints.get(name):
			
				cur_workpoints[name] = []
				
			path_id = (str(name), str(bt.last_p_id), str(target_id))
				
			if not s_paths.get(path_id):

				path, path_ids = mh.find_path(bt.last_p_id, target_id, bt.last_vect)	
				
				if path:

					if not cur_worker_names.__contains__(name):
					
						cur_worker_names.append(name)

					goal = path[len(path) - 1]
					bt.last_vect = goal.path_cost
					bt.last_p_id = target_id
					cur_path_cost_sum += get_path_cost(path)
					cur_paths[name].append(path)
					cur_workpoints[name].append(goal)
					s_paths[path_id] = path_cost
					names_cp.pop(0)
				
				else:
				
					#print('Path was not found!')
					break_flag = True
					break
					
			else:
			
					if not cur_worker_names.__contains__(name):
					
						cur_worker_names.append(name)

					path = s_paths[path_id]
					bt.last_vect = get_end_vect(path)
					bt.last_p_id = target_id
					goal = path[len(path) - 1]
					cur_path_cost_sum += goal.path_cost
					cur_paths[name].append(path)
					cur_workpoints[name].append(goal)
					names_cp.pop(0)
				

		if not break_flag:
		
			for name in cur_worker_names:

				bt = trackers_cp[name]
			
				path, path_ids = mh.find_path(bt.last_p_id, bt.start_id, bt.last_vect)
				
				if path:

					path_cost = get_path_cost(path)
					cur_path_cost_sum += path_cost
					cur_paths[name].extend(path)
					
				else:
				
					break_flag = True
					break

			if cur_path_cost_sum < path_cost_sum and not break_flag:
		
				paths = copy.copy(cur_paths)
				path_cost_sum = cur_path_cost_sum
				worker_names = copy.copy(cur_worker_names)
				workpoints = copy.copy(cur_workpoints)
				

		print('Iter count: ' + str(iter_count) + ' | Current path cost sum: ' + str(cur_path_cost_sum) + ' | Path cost sum: ' + str(path_cost_sum))
			
	charger_names = subtraction_of_set(names, worker_names)
	print('\nFinal path cost sum for ' + str(len(worker_names)) + ' workers: ' + str(path_cost_sum))
	print('Worker names: ' + str(worker_names))
	print('Charger names: ' + str(charger_names))
	return paths, workpoints, worker_names, charger_names
	
def start_target_assignment(w_count, c_count, t_count):

	mh, trackers, target_ids = prep_targets(t_count)
	s_exec_time = time.time()
	paths, workpoints, w_names, c_names = target_assignment(mh, w_count, c_count, trackers, target_ids)
	return paths, workpoints, w_names, c_names
	
def get_end_vect(path):

	goal = path[len(path) - 1]
	last_goal = path[len(path) - 2]
	last_vect = last_goal.get_dir_vector_between_points(goal)
	return last_vect
