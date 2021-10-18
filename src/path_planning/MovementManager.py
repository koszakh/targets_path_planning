# The module for calculating the path, taking into account the avoidance of potential collisions between targets

import rospy
import path_planning.Constants as const
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
from gazebo_communicator.Worker import Worker
from gazebo_communicator.Charger import Charger
import gazebo_communicator.BatteryTracker as bt
from path_planning.Point import Point
import copy
import rvo2
from time import sleep
from math import fabs
import random
from threading import Thread
import dubins

# A class that implements the calculation of non-collisionless trajectories of a group of target objects

# ms: initial robot speed
# sim: group of targets movement simulator instance
# heightmap: a dictionary containing all vertices of the heightmap
# cells: a dictionary containing all cells of the heightmap (includes four vertices)
# x_step: distance between adjacent vertices of the height map in x
# y_step: distance between adjacent vertices of the height map in y
# robots: Target Management Object Dictionary in Gazebo
# agents: dictionary of agents used in sim
# final_paths: dictionary of final routes of robots
# init_paths: dictionary of initial routes of robots
class MovementManager(Thread):

	def __init__(self, mh, w_names, c_names):
	
		Thread.__init__(self)
		self.ms = gc_const.MOVEMENT_SPEED
		self.mh = mh
		self.robots = get_robots_dict(w_names, c_names)
		self.w_names = w_names
		self.c_names = c_names
		self.time_step = rospy.Duration(0, gc_const.PID_NSEC_DELAY)

# Adding an agent to the group movement simulation
# Input
# robot_name: target object name used in Gazebo
# path: list of points of the original path of the target

# Calculating the smallest distance from a given agent to any of the others

# Input
# robot_name: the name of this robot

# Output
# min_dist: distance to nearest robot
	def calc_min_neighbor_dist(self, robot_name):
	
		robots_copy = copy.copy(self.robots)
		current_robot = self.robots[robot_name]
		current_robot_pos = current_robot.get_robot_position()
		robots_copy.pop(robot_name)
		min_dist = float('inf')
		closest_r_name = None
		
		for name in robots_copy.keys():
		
			robot_pos = self.robots[name].get_robot_position()
			dist = current_robot_pos.get_distance_to(robot_pos)
		
			if dist < min_dist:

				min_dist = dist
				closest_r_name = name
		
		return min_dist, closest_r_name

	def start_robots(self):
	
		for key in self.robots:
		
			self.robots[key].start()
			
	def prepare_robots(self, w_paths, w_points, w_ch_points, ch_points, to_ch_p_paths, to_base_paths):
		
		for name in self.w_names:
		
			if w_points.get(name):

				self.robots[name].set_worker_data(w_paths[name], w_points[name], w_ch_points[name])
				
			else:

				self.robots[name].set_worker_data([], [], [])
			
		for name in self.c_names:
		
			self.robots[name].set_charger_data(ch_points[name], to_ch_p_paths[name], to_base_paths[name])
			
	
	def run(self):

		print('Mission started.')
		self.start_robots()
		
		cont_flag = True
		
		while cont_flag:
		
			cont_flag = False
			rospy.sleep(self.time_step)
			
			for key in self.robots:
			
				robot = self.robots[key]
				
				if not robot.mode == "finished":

					cont_flag = True
					
					if robot.mode == "movement":

						self.robot_avoiding(key)
							
					elif robot.mode == "waiting_for_worker":
					
						cur_worker = self.robots[robot.cur_worker]
						ch_p = robot.ch_points[0]
						w_pos = cur_worker.get_robot_position()
						dist = w_pos.get_distance_to(ch_p)
						
						if dist < gc_const.DISTANCE_ERROR:
						
							robot.dock_path, robot.dock_point = self.get_docking_path(robot.name, cur_worker.name)
							robot.change_mode("prepare_to_dock")
							
					elif robot.mode == "finished_charging":
					
						cur_worker = self.robots[robot.cur_worker]
						w_pos = cur_worker.get_robot_position()
						c_pos = robot.get_robot_position()
						dist = w_pos.get_distance_to(c_pos)
						if dist < const.MIN_NEIGHBOR_DIST:
						
							robot.wait()
							
						else:
						
							robot.stop_waiting()
						
							
							
		print('ALL ROBOTS FINISHED!')
		
	def is_robot_standing(self, robot):
	
		if robot.mode == "task_performing" or robot.mode == "waiting_for_charger" or robot.mode == "finished" or robot.mode == "waiting_for_worker" or robot.waiting:
		
			return True
			
		else:
		
			return False
		
	def robot_avoiding(self, key):
	
		robot = self.robots[key]
	
		min_dist, neighbor_name = self.calc_min_neighbor_dist(key)

		neighbor = self.robots[neighbor_name]
		robot_pos = robot.get_robot_position()
		neighbor_pos = neighbor.get_robot_position()
		
		robot_vect = robot.get_robot_orientation_vector()
		neighbor_vect = neighbor.get_robot_orientation_vector()
		
		robot_to_n_vect = robot_pos.get_dir_vector_between_points(neighbor_pos)
		n_to_robot_vect = neighbor_pos.get_dir_vector_between_points(robot_pos)
		
		robot_angle = robot_vect.get_angle_between_vectors(robot_to_n_vect)
		neighbor_angle = neighbor_vect.get_angle_between_vectors(n_to_robot_vect)
		
		robots_dir_angle = robot_vect.get_angle_between_vectors(neighbor_vect)

		if min_dist < const.MIN_NEIGHBOR_DIST and robot_angle < const.HW_ORIENT_BOUND and robot_angle > const.LW_ORIENT_BOUND and not self.is_robot_standing(neighbor):

			#print(key + ' is waiting!')
			robot.wait()
			
		elif min_dist < const.MIN_NEIGHBOR_DIST and robot_angle > 0 and robot_angle < const.DODGE_ORIENT_BOUND and self.is_robot_standing(neighbor):

			#print(key + ' is dodging to the right!')
			robot.dodging = True
			robot.movement(robot.ms, -gc_const.ROTATION_SPEED)
		
		elif min_dist < const.MIN_NEIGHBOR_DIST and robot_angle < 0 and robot_angle > -const.DODGE_ORIENT_BOUND and self.is_robot_standing(neighbor):
		
			#print(key + ' is dodging to the left!')
			robot.dodging = True
			robot.movement(robot.ms, gc_const.ROTATION_SPEED)

		elif robot.waiting:
		
			robot.stop_waiting()
			
		elif robot.dodging:
		
			robot.stop_dodging()

	def get_docking_path(self, c_name, w_name):
	
		charger = self.robots[c_name]
		worker = self.robots[w_name]
		
		rech_pos = worker.get_robot_position()
		rech_vect = worker.get_robot_orientation_vector()
		ch_pos = charger.get_robot_position()
		ch_vect = charger.get_robot_orientation_vector()
		pre_dock_point = calc_dock_point(rech_pos, rech_vect, const.PRE_DOCK_DISTANCE)
		dock_point = calc_dock_point(rech_pos, rech_vect, gc_const.DOCKING_THRESHOLD)
		dock_path = self.plan_path_to_dock_point(ch_pos, ch_vect, pre_dock_point, rech_vect)
		return dock_path, dock_point
						
	def plan_path_to_dock_point(self, s_pos, s_vect, e_pos, e_vect):
		
		q0 = (s_pos.x, s_pos.y, s_vect.vector_to_radians())
		q1 = (e_pos.x, e_pos.y, e_vect.vector_to_radians())
		solution = dubins.shortest_path(q0, q1, const.ROBOT_RADIUS)
		configurations, _ = solution.sample_many(gc_const.DOCK_DISTANCE_ERROR * 2)
		path = self.convert_tup_to_mas(configurations)
		last_p = e_pos.get_point_in_direction(e_vect, gc_const.DISTANCE_ERROR)
		path.append(last_p)
		gc.visualise_path(path, gc_const.GREEN_VERTICE_PATH, 'v')
		return path
			
	def convert_tup_to_mas(self, tup_mas):
	
		path = []
		
		for tup in tup_mas:
		
			point = self.convert_to_point3d(tup)
			path.append(point)
			
		return path
		
	def convert_to_point3d(self, tup):
	
		x = tup[0]
		y = tup[1]
		z = 0#self.mh.find_z(x, y)
		p = Point(x, y, z)
		
		return p
	
def get_robots_dict(w_names, c_names):

	robots = {}
	trackers = bt.get_battery_trackers(w_names, c_names)

	for name in w_names:
	
		robot = Worker(name, trackers)
		robots[name] = robot

	for name in c_names:
	
		robot = Charger(name, trackers)
		robots[name] = robot
		
	return robots
	
def calc_dock_point(pos, orient, offset):

	rev_orient = orient.get_rotated_vector(180)
	p = pos.get_point_in_direction(rev_orient, offset)
	
	return p
