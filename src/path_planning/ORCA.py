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

class AgentManager:
	def __init__(self, agent_name):
		self.name = agent_name
		self.last_point = None
		self.last_goal = None
		self.current_goal = None
		self.goal_point = None
		self.finished_planning = False

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
		self.sim = rvo2.PyRVOSimulator(gc_const.ORCA_TIME_STEP, gc_const.ORCA_NEIGHBOR_DIST, \
gc_const.ORCA_MAX_NEIGHBORS, gc_const.ORCA_TIME_HORIZON, gc_const.ORCA_TIME_HORIZON_OBST, \
gc_const.ORCA_RADIUS, self.ms)
		print('ORCA time step: ' + str(gc_const.ORCA_TIME_STEP))
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
		self.amanager[robot_name] = AgentManager(robot_name)
		robot_pos = self.amanager[robot_name].get_agent_position()
		self.amanager[robot_name].last_point = robot_pos
		init_dist = robot_pos.get_distance_to(path[0])
		self.amanager[robot_name].set_init_path(path)
		print('Robot ' + robot_name + ' initial path length: ' + str(len(path)))
		self.amanager[robot_name].current_goal = path[0]
		self.init_paths[robot_name] = copy.copy(path)
		self.amanager[robot_name].last_goal = robot_pos
		robot_pos_2d = robot_pos.get_xy()
		self.agents[robot_name] = self.sim.addAgent(robot_pos_2d)
		robot_vect = self.amanager[robot_name].get_agent_orientation()
		vel_vect = (robot_vect.x * self.ms, robot_vect.y * self.ms)
		self.sim.setAgentPrefVelocity(self.agents[robot_name], vel_vect)
		self.final_paths[robot_name] = []
		radius = self.sim.getAgentRadius(self.agents[robot_name])
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
		
		for agent_name in agents_copy:
		
			agent_pos_2d = self.sim.getAgentPosition(self.agents[agent_name])
			agent_pos = Point(agent_pos_2d[0], agent_pos_2d[1], 0)
			dist = current_agent_pos.get_distance_to(agent_pos)
		
			if dist < min_dist:
		
				min_dist = dist
		
		return min_dist

# Calculate a 2D velocity vector that takes into account the slope of the surface
# Input
# robot_name: target object name used in Gazebo

# Output
# vel_vect: 2d velocity vector (tuple)
	def calc_vel_vect_3d(self, robot_name, robot_pos):
		
		am = self.amanager[robot_name]
		current_goal = am.current_goal
		last_p = am.last_point
		
		if self.calc_min_neighbor_dist(robot_name) > gc_const.ORCA_NEIGHBOR_DIST + (gc_const.DISTANCE_ERROR * 2):
			
			last_vect = last_p.get_dir_vector_between_points(robot_pos)
			new_vect = last_p.get_dir_vector_between_points(current_goal)
			angle_difference = last_vect.get_angle_between_vectors(new_vect)
			
			if angle_difference > gc_const.ANGLE_ERROR:
			
				vect = last_vect.get_rotated_vector(gc_const.ANGLE_ERROR)
			
			elif angle_difference < -gc_const.ANGLE_ERROR:
			
				vect = last_vect.get_rotated_vector(-gc_const.ANGLE_ERROR)
			
			else:
			
				vect = last_p.get_dir_vector_between_points(current_goal)
		
		else:
		   
			last_vect = last_p.get_dir_vector_between_points(current_goal)
			vect = last_vect.get_rotated_vector(gc_const.ANGLE_ERROR)
		goal_dist = current_goal.get_distance_to(robot_pos)
		goal_dist_2d = current_goal.get_2d_distance(robot_pos)
		current_ms = float(goal_dist_2d / goal_dist) * self.ms
		vel_vect = (vect.x * current_ms, vect.y * current_ms)
		return vel_vect

# Checking the reachability of the target point by the robot
# Input
# robot_name: target object name used in Gazebo
# robot_pos: current calculated position of the robot
	def goal_achievement_check(self, robot_name, robot_pos):
		am = self.amanager[robot_name]
		current_goal = am.current_goal
		dist_2d = robot_pos.get_2d_distance(current_goal)
		
		if dist_2d < gc_const.DISTANCE_ERROR * 3:
		
			if len(self.init_paths[robot_name]) > 1:
			
				am.last_goal = current_goal
				self.init_paths[robot_name].pop(0)
				new_goal = self.init_paths[robot_name][0]
				am.current_goal = new_goal
				vel_vect = self.calc_vel_vect_3d(robot_name, robot_pos)
				self.sim.setAgentPrefVelocity(self.agents[robot_name], vel_vect)
				
			else:
			
				self.sim.setAgentPrefVelocity(self.agents[robot_name], (0, 0))
				self.sim.setAgentVelocity(self.agents[robot_name], (0, 0))
				am.finished_planning = True
				print('Robot ' + robot_name + ' reached the target point. Path length: ' + str(len(self.final_paths[robot_name])))
				
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
					#gc.spawn_sdf_model(key + '_v_' + str(len(self.final_paths[key])), gc_const.VERTICE_PATH, robot_pos)
					self.final_paths[key].append(robot_pos)
					self.goal_achievement_check(key, robot_pos)
					self.amanager[key].last_point = robot_pos
					
		print('ORCA3D for ' + str(len(self.agents)) + ' agents is completed!')
		return self.final_paths
