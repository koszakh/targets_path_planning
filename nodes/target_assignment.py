#!/usr/bin/env python

# Path planning node for ground targets
import path_planning.PathPlanner as pp
import rospy
import copy
import numpy as np
from targets_path_planning.msg import AllPaths, Path
from geometry_msgs.msg import Point
from path_planning.Point import Point as PointGeom, Vector2d
from path_planning.Heightmap import Heightmap
import path_planning.Constants as const
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
from path_planning.ORCA import ORCAsolver
import random
import itertools
import functools

def get_sequence_of_des_len(mas, des_len):

	new_mas = copy.copy(mas)
	multiply_coeff = int(des_len / len(mas))

	for i in range(multiply_coeff):

		new_mas.extend(mas)

	init_per_list = itertools.permutations(new_mas, des_len)
	per_list = []
	
	for item in init_per_list:

		print(item)
		per_list.append(list(item))

	copy_per_list = remove_repeats(per_list)

	return copy_per_list

def remove_repeats(mas):

	copy_mas = copy.copy(mas)

	for i in range(len(mas)):

		per1 = mas[i]

		for j in range(i, len(mas)):

			per2 = mas[j]
			if compare_lists(per1, per2):

				if per2 in copy_mas and copy_mas.count(per2) > 1:

					copy_mas.remove(per2)

	return copy_mas

def compare_lists(mas1, mas2):

	if functools.reduce(lambda x, y: x and y, map(lambda p, q: p == q, mas1, mas2), True):

		return True

	else:

		return False

def prepare_hmap():

	hm = Heightmap(const.HEIGHTMAP_SDF_PATH)
	hmap, l_scale, w_scale, x_step, y_step, step_count = hm.prepare_heightmap()
	mh = pp.PathPlanner(hmap, l_scale, w_scale, x_step, y_step, step_count)
	mh.gridmap_preparing()
	return mh

def prep_targets():

	mh = prepare_hmap()
	
	offset = const.DIST_OFFSET
	
	avg_x = np.mean([mh.min_x, mh.max_x])
	avg_y = np.mean([mh.min_y, mh.max_y])
	
	start = (avg_x - 30, avg_y - 30)
	goal = (avg_x + 30, avg_y + 30)
	
	target_ids = mh.get_random_ids_in_area(goal[0], goal[1], offset, const.ROBOTS_COUNT)
	
	targets = [mh.heightmap[v_id] for v_id in target_ids]
	gc.visualise_path(targets, gc_const.BIG_GREEN_VERTICE_PATH, 'target')
	
	paths = {}
	trackers = {}
	
	for i in range(1, const.ROBOTS_COUNT + 1):
	
		name = 'sim_p3at' + str(i)
		robot_pos, orient = mh.get_start_pos(start[0], start[1], offset)
		gc.spawn_target(name, robot_pos, orient)
		bt = gc.BatteryTracker(name)
		start_id, start_pos = mh.get_start_vertice_id(robot_pos, bt.last_vect)
		bt.last_p_id = start_id
		trackers[name] = bt
		
	return mh, trackers, target_ids
	
def target_assignment(mh, trackers, target_ids):

	names = copy.copy(trackers.keys())
	iter_count = 0
	chargers_quota = [x for x in range(20, 70, 10)]
	print(chargers_quota)
	path_cost_sum = float('inf')
	
	for quota in chargers_quota:
	
		chargers_num = int((len(names) * quota) / 100 )
		worker_names = names[:len(names) - chargers_num]
		charger_names = names[len(names) - chargers_num:]
#		print('Worker names: ' + str([item[item.find('t') + 1:] for item in worker_names]))
#		print('Chargers names: ' + str([item[item.find('t') + 1:] for item in charger_names]))
		
		print('Workers count: ' + str(len(worker_names)) + ' | Chargers count: ' + str(len(charger_names)) + ' | Number of tasks: ' + str(len(target_ids)))
		
		combs = get_sequence_of_des_len(worker_names, len(target_ids))
		print(combs)
		#combs = list(itertools.permutations(worker_names))

		for item in combs:
		
			#print('\n>>> NEW ITER')
			print('\n' + str(item))
			cur_path_cost_sum = 0
			iter_count += 1
			cur_paths = {}
			names_cp = copy.copy(list(item))
			trackers_cp = copy.deepcopy(trackers)
			
			for target_id in target_ids:

				print(names_cp[0], target_id)
				name = names_cp[0]
				bt = trackers_cp[name]
				break_flag = False
				
				if not cur_paths.get(name):
				
					cur_paths[name] = []

				path, path_ids, path_cost = mh.find_path(bt.last_p_id, target_id, bt.last_vect)	
				
				if path:

					print('Path was found!')
					current_energy_cost = path_cost * gc_const.PATH_COST_CHARGE_COEF + gc_const.TASK_ENERGY_COST		
					cur_path_cost_sum += path_cost
					cur_paths[name].extend(path)
					bt.add_task_cost(current_energy_cost)
					goal = path[len(path) - 1]
					last_goal = path[len(path) - 2]
					bt.last_vect = last_goal.get_dir_vector_between_points(goal)
					bt.last_p_id = target_id
					bt.path_costs[target_id] = path_cost
					names_cp.pop(0)
				
				else:
				
					print('Path was not found!')
					break_flag = True
					break
					
			if cur_path_cost_sum < path_cost_sum and not break_flag:
			
				paths = copy.copy(cur_paths)
				path_cost_sum = cur_path_cost_sum

			print('Iter count: ' + str(iter_count) + ' | Current path cost sum: ' + str(cur_path_cost_sum) + ' | Path cost sum: ' + str(path_cost_sum))
			
	print('\nFinal path cost sum for ' + str(len(worker_names)) + ' workers : ' + str(path_cost_sum))
	return paths
		
rospy.init_node('target_assignment')
mh, trackers, target_ids = prep_targets()
paths = target_assignment(mh, trackers, target_ids)
				
	
	
