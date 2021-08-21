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

class AgentManager:

	def __init__(self, agent_name):
	
		self.name = agent_name
		self.last_point = None
		self.last_goal = None
		self.current_goal = None
		self.goal_point = None
		self.finished_planning = False
		self.min_n_dist = float('inf')

	def get_agent_position(self):
	
		agent_pos = gc.get_model_position(self.name)
		return agent_pos

	def get_agent_orientation(self):
	
		agent_orient = gc.get_robot_orientation_vector(self.name)
		return agent_orient

# Setting the initial path for the robot
# Input
# path: list of path points
	def set_init_path(self, path):
	
		self.init_path = path
		self.goal_point = path[len(path) - 1]

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
class ORCAsolver:

	def __init__(self, heightmap, cells, x_step, y_step, l_scale, w_scale):
	
		self.ms = gc_const.MOVEMENT_SPEED
		self.sim = rvo2.PyRVOSimulator(const.ORCA_TIME_STEP, const.ORCA_NEIGHBOR_DIST, \
const.ORCA_MAX_NEIGHBORS, const.ORCA_TIME_HORIZON, const.ORCA_TIME_HORIZON_OBST, \
const.ORCA_RADIUS, self.ms)
		self.heightmap = heightmap
		self.cells = cells
		self.x_step = x_step
		self.y_step = y_step
		self.l_scale = l_scale
		self.w_scale = w_scale
		self.amanager = {}
		self.agents = {}
		self.final_paths = {}
		self.init_paths = {}
		self.path_subs = {}
		
# Finding the height value of an adjacent point on a height map
# Input
# x: point x-coordinate value 
# y: point y-coordinate value

# Output
# z: calculated point z-coordinate value

	def find_z(self, x, y):
	
		p = Point(x, y, 0)
		cell_id = self.get_current_cell_id(p)
		cell = self.cells[cell_id]
		z = cell.find_z_on_cell(x, y)
		return z
		
	def get_current_cell_id(self, point):
	
		x = point.x
		y = point.y
		j = int((x + (self.l_scale / 2)) // self.x_step)
		i = int(((self.w_scale / 2) - y) // self.y_step)
		cell_id = (str(float(i)), str(float(j)))
		return cell_id

# Adding an agent to the group movement simulation
# Input
# robot_name: target object name used in Gazebo
# path: list of points of the original path of the target

	def add_agent(self, robot_name, path):
	
		am = AgentManager(robot_name)
		robot_pos = am.get_agent_position()
		am.last_point = robot_pos
		
		if len(path) == 0:
		
			am.finished_planning = True
		
		else:
		
			am.set_init_path(path)
			goal_p = am.goal_point
			am.current_goal = path[0]
			gc.spawn_sdf_model(goal_p, gc_const.RED_VERTICE_PATH, robot_name + '_goal')
			self.init_paths[robot_name] = copy.copy(path)
			print('Robot ' + robot_name + ' initial path length: ' + str(len(path)))
		
		am.last_goal = robot_pos
		robot_pos_2d = robot_pos.get_xy()
		self.agents[robot_name] = self.sim.addAgent(robot_pos_2d)
		robot_vect = am.get_agent_orientation()
		am.last_vect = robot_vect
		vel_vect = (robot_vect.x * self.ms, robot_vect.y * self.ms)
		self.sim.setAgentPrefVelocity(self.agents[robot_name], vel_vect)
		self.final_paths[robot_name] = []
		radius = self.sim.getAgentRadius(self.agents[robot_name])
		self.amanager[robot_name] = am
		print(robot_name + ' radius: ' + str(radius))

# Calculating the smallest distance from a given agent to any of the others

# Input
# robot_name: the name of this robot

# Output
# min_dist: distance to nearest robot
	def calc_min_neighbor_dist(self, robot_name):
	
		agents_copy = copy.copy(self.agents)
		current_agent = self.agents[robot_name]
		current_agent_pos_2d = self.sim.getAgentPosition(current_agent)
		current_agent_pos = Point(current_agent_pos_2d[0], current_agent_pos_2d[1], 0)
		agents_copy.pop(robot_name)
		min_dist = float('inf')
		closest_agent = None
		
		for agent_name in agents_copy.keys():
		
			agent_pos_2d = self.sim.getAgentPosition(self.agents[agent_name])
			agent_pos = Point(agent_pos_2d[0], agent_pos_2d[1], 0)
			dist = current_agent_pos.get_distance_to(agent_pos)
		
			if dist < min_dist:
		
				min_dist = dist
				closest_agent = agent_name
		
		return min_dist, closest_agent

# Calculate a 2D velocity vector that takes into account the slope of the surface
# Input
# robot_name: target object name used in Gazebo

# Output
# vel_vect: 2d velocity vector (tuple)
	def calc_vel_vect_3d(self, robot_name, robot_pos):
		
		am = self.amanager[robot_name]
		current_goal = am.current_goal
		last_p = am.last_point
		last_vect = last_p.get_dir_vector_between_points(robot_pos)
		am.last_vect = last_vect
		min_neighbor_dist, closest_neighbor_id = self.calc_min_neighbor_dist(robot_name)
		
		if min_neighbor_dist < am.min_n_dist:
		
			am.min_n_dist = min_neighbor_dist
		
		if min_neighbor_dist > (const.ORCA_NEIGHBOR_DIST + const.ORCA_RADIUS + gc_const.DISTANCE_ERROR):
			
			vect = self.calc_new_vel_direction(last_p, last_vect, current_goal)
		
		else:
		   
		   	closest_am = self.amanager[closest_neighbor_id]
		   	vect_to_goal = last_p.get_dir_vector_between_points(current_goal)
		   	det_angle = fabs(last_vect.get_angle_between_vectors(vect_to_goal))
			n_last_vect = closest_am.last_vect
			n_last_point = closest_am.last_point
			vect_to_neighbor = robot_pos.get_dir_vector_between_points(n_last_point)
			angle_to_neighbor = last_vect.get_angle_between_vectors(vect_to_neighbor)
			
			next_n_p = n_last_point.get_point_in_direction(n_last_vect, self.ms * const.ORCA_TIME_STEP)
			
			dist = last_p.get_distance_to(n_last_point)
			next_dist = robot_pos.get_distance_to(n_last_point)
			
			n_vel = self.sim.getAgentPrefVelocity(self.agents[closest_neighbor_id])
			
			if det_angle > const.ORCA_MAX_ANGLE or next_dist > dist:
			
				vect = self.calc_new_vel_direction(last_p, last_vect, current_goal)
				
			elif angle_to_neighbor < 0 and n_vel == (0, 0):
			
				vect = last_vect.get_rotated_vector(gc_const.ANGLE_ERROR / 2)
				
			else:
			
				vect = last_vect.get_rotated_vector(-gc_const.ANGLE_ERROR / 2)
			
		goal_dist = current_goal.get_distance_to(robot_pos)
		goal_dist_2d = current_goal.get_2d_distance(robot_pos)
		current_ms = float(goal_dist_2d / goal_dist) * self.ms
		vel_vect = (vect.x * current_ms, vect.y * current_ms)
		return vel_vect
		
	def calc_new_vel_direction(self, last_p, last_vect, current_goal):
	
		new_vect = last_p.get_dir_vector_between_points(current_goal)
		angle_difference = last_vect.get_angle_between_vectors(new_vect)
		
		if angle_difference > gc_const.ANGLE_ERROR:
		
			vect = last_vect.get_rotated_vector(gc_const.ANGLE_ERROR / 2)
		
		elif angle_difference < -gc_const.ANGLE_ERROR:
		
			vect = last_vect.get_rotated_vector(-gc_const.ANGLE_ERROR / 2)
		
		else:
		
			vect = last_p.get_dir_vector_between_points(current_goal)
			
		return vect

# Checking the reachability of the target point by the robot
# Input
# robot_name: target object name used in Gazebo
# robot_pos: current calculated position of the robot
	def goal_achievement_check(self, robot_name, robot_pos):
	
		am = self.amanager[robot_name]
		current_goal = am.current_goal
		dist_2d = robot_pos.get_2d_distance(current_goal)
		pref_vect = robot_pos.get_dir_vector_between_points(current_goal)
		angle = fabs(am.last_vect.get_angle_between_vectors(pref_vect))
		
		if dist_2d < gc_const.DISTANCE_ERROR * 3 or (dist_2d < const.ROBOT_RADIUS and angle > const.ORCA_MAX_ANGLE):
		
			if len(self.init_paths[robot_name]) > 1:
			
				am.last_goal = current_goal
				self.init_paths[robot_name].pop(0)
				new_goal = self.init_paths[robot_name][0]
				am.current_goal = new_goal
				vel_vect = self.calc_vel_vect_3d(robot_name, robot_pos)
				self.sim.setAgentPrefVelocity(self.agents[robot_name], vel_vect)
				
			else:
			
				path = copy.copy(self.final_paths[robot_name])
				path = path_loops_deleting(path)
				self.final_paths[robot_name] = copy.copy(path)
				self.final_paths[robot_name].append(am.goal_point)
				self.sim.setAgentPrefVelocity(self.agents[robot_name], (0, 0))
				self.sim.setAgentVelocity(self.agents[robot_name], (0, 0))
				am.finished_planning = True
				print('>>> ' + robot_name + ' min neighbor dist: ' + str(am.min_n_dist))
				
		else:
		
			vel_vect = self.calc_vel_vect_3d(robot_name, robot_pos)
			self.sim.setAgentPrefVelocity(self.agents[robot_name], vel_vect)

# Running a simulation of the movement of a group of targets
	def run_orca(self):
	
		print('\nORCA3D for ' + str(len(self.agents)) + ' agents is running.')
		cont_flag = True
		
		while cont_flag:
		
			cont_flag = False
			
			self.sim.doStep()
			
			for key in self.amanager.keys():
				
				if not self.amanager[key].finished_planning:
					
					cont_flag = True
					pos = self.sim.getAgentPosition(self.agents[key])
					z = self.find_z(pos[0], pos[1])
					robot_pos = Point(pos[0], pos[1], z)						
					self.final_paths[key].append(robot_pos)
					self.goal_achievement_check(key, robot_pos)
					self.amanager[key].last_point = robot_pos
					
		for key in self.final_paths.keys():
	
			path = self.final_paths[key]
			short_path = delete_intermediate_points(path, 100)
			#gc.visualise_path(short_path, random.choice(list(gc_const.PATH_COLORS)), str(key) + '_p_')
			
		print('ORCA3D for ' + str(len(self.agents)) + ' agents is completed!')
		return self.final_paths
		
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
	
	if len(path) > 1:

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
		
		path_curv = get_path_curvature(new_path)
		print('New path curvature: ' + str(path_curv))
		
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
