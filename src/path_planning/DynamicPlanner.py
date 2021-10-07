# The module for calculating the path, taking into account the avoidance of potential collisions between targets

import rospy
import path_planning.Constants as const
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
from path_planning.Point import Point 
import copy
import rvo2
from time import sleep
from math import fabs
import random
from threading import Thread

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
class DynamicPlanner(Thread):

	def __init__(self, mh, robots):
	
		Thread.__init__(self)
		self.ms = gc_const.MOVEMENT_SPEED
		self.mh = mh
		self.robots = robots
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
	
	def run(self):

		self.start_robots()
		
		cont_flag = True
		
		while cont_flag:
		
			cont_flag = False
			rospy.sleep(self.time_step)
			
			for key in self.robots:
			
				robot = self.robots[key]
				
				if not robot.finished:

					cont_flag = True
					
					if robot.mode == "movement":

						min_dist, neighbor_name = self.calc_min_neighbor_dist(key)
						
						#print(key, neighbor_name)

						if min_dist < const.MIN_NEIGHBOR_DIST:
						
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
							
							if robot_angle < 30 and robot_angle > -60:
							
								robot.wait()
								
							if robot.waiting:
							
								print('\n')
								print(robot.name)
								print(robot.name + ' angle to neighbor: ' + str(robot_angle) + ' | waiting: ' + str(robot.waiting))
								print(neighbor_name + ' angle to neighbor: ' + str(neighbor_angle) + ' | waiting: ' + str(neighbor.waiting))
								
						elif robot.waiting:
						
							robot.stop_waiting()
							
		print('ALL ROBOTS FINISHED!')
						
					
			
	def finish_target_planning(self, robot_name):
	
		am = self.amanager[robot_name]
		agent = self.agents[robot_name]
	
		self.sim.setAgentPrefVelocity(agent, (0, 0))
		self.sim.setAgentVelocity(agent, (0, 0))
		print(' >>> ' + robot_name + ' has finished! Min neighbor dist: ' + str(am.min_n_dist) + ' <<<')
		am.finished_planning = True
		
def delete_doubled_vertices(path):
	new_path = copy.copy(path)
	i = 1
	goal = path[len(path) - 1]
	
	while True:
		
		p1 = path[i - 1]
		p2 = path[i]
		dist = p1.get_2d_distance(p2)
		
		if round(dist, 2) == 0:
		
			new_path.remove(p2)
			i += 2
		
		else:
		
			i += 1
		
		if p2.get_2d_distance(goal):
		
			break
		
		if i >= len(path):
		
			i = len(path) - 1
	
	return new_path

def path_loops_deleting(path):
	new_path = copy.copy(path)
	i = len(path) - 1
	goal = path[0]
	
	if len(path) > 2:

		while True:
		
			p1 = path[i - 2]
			p2 = path[i - 1]
			p3 = path[i]
			dist1 = p1.get_2d_distance(p3)
			dist2 = p2.get_2d_distance(p3)
		
			if dist1 < dist2:
		
				new_path.remove(p2)
				i -= 2
		
			else:
		
				i -= 1
		
			if round(p1.get_2d_distance(goal), 2) == 0:
		
				break
		
			if i < 2:
		
				i = 2
		
		#path_curv = get_path_curvature(new_path)
		#print('New path curvature: ' + str(path_curv))
		
		return new_path
		
	else:
	
		return path
	
def get_path_curvature(path):

	max_curvature = 0

	for i in range(0, len(path) - 2):

		p1 = path[i]
		p2 = path[i + 1]
		p3 = path[i + 2]
		v1 = p1.get_dir_vector_between_points(p2)
		v2 = p2.get_dir_vector_between_points(p3)
		angle_difference = fabs(v1.get_angle_between_vectors(v2))

		if angle_difference > max_curvature:

			max_curvature = angle_difference

	return max_curvature
	
def delete_intermediate_points(path, cut_step):

	path_copy = copy.copy(path)

	for i in range(len(path) - 1):

		if (i % cut_step > 0):

			if path[i] in path_copy:

				path_copy.remove(path[i])

	return path_copy
