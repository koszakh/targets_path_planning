# Module for interacting with the Gazebo simulation environment

#!/usr/bin/env python

import rospy
import path_planning.Point as PointGeom
from targets_path_planning.msg import Path, TargetDamage
import GazeboConstants as const
import time
from math import sqrt, fabs, sin, asin, pi, cos, acos
from gazebo_msgs.msg import ModelState, ODEPhysics
from geometry_msgs.msg import Pose, Twist, Quaternion, Point, Vector3
from std_msgs.msg import Float64, Time, Duration
from gazebo_msgs.srv import SetModelState, GetModelState, GetModelProperties, SpawnModel, GetWorldProperties, GetLinkState, DeleteModel
import vector3d
import threading as thr
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import NavSatFix
import copy

# The class of the control object of the ground target in the Gazebo simulation environment

# threading is used to control multiple targets in parallel
# topic_name: ROS topic name used to send control commands
# vel_publisher: ROS Publisher object
# name: the name of the object in the environment
# dir_point: the name of the object used to calculate the direction of movement of the robot
# init_path: the path of the robot before processing with ORCA
# final_path: final path of the robot after processing with ORCA
# finished_planning: a flag whose true value means that the robot has reached the end point in the ORCA simulation
# last_goal: previous point of the robot route
class Robot(thr.Thread):
	
	def __init__(self, robot_name):
	
		thr.Thread.__init__(self)
		self.name = robot_name
		self.id = self.name[self.name.find('t') + 1:]
		self.init_topics()
		self.pid_delay = rospy.Duration(0, const.PID_NSEC_DELAY)
		msg = Twist()
		self.vel_publisher.publish(msg)
		rospy.sleep(self.pid_delay)
		self.set_movespeed(const.MOVEMENT_SPEED)
		self.dir_point = robot_name + const.DIR_POINT_SUFFIX
		self.path = []
		self.latitude = None
		self.longitude = None
		self.total_damage = 0
		self.path_p_count = 0
		self.local_path_dir = const.PATHS_DIR + const.LOCAL_PATH_DIRS[3] + self.name + '.txt'
		f = open(self.local_path_dir, 'w+')
		f.close()
		#self.get_wheel_distance()
	
	def get_wheel_distance(self):
	
		fl_name = self.name + "::p3at_front_left_wheel"
		fr_name = self.name + "::p3at_front_right_wheel"
		bl_name = self.name + "::p3at_back_left_wheel"
		br_name = self.name + "::p3at_back_right_wheel"
		fl_pos = get_link_position(fl_name)
		fr_pos = get_link_position(fr_name)
		bl_pos = get_link_position(bl_name)
		br_pos = get_link_position(br_name)
		f_dist = fl_pos.get_distance_to(fr_pos)
		b_dist = bl_pos.get_distance_to(br_pos)
		print('Front dist: ' + str(f_dist))
		print('Back dist: ' + str(b_dist))
		
	def init_topics(self):
		
		topic_subname = '/' + self.name
		self.vel_publisher = rospy.Publisher(topic_subname + '/cmd_vel', Twist, queue_size=10)
		self.waypoint_pub = rospy.Publisher(topic_subname + '/waypoint', Point, queue_size=10)
		self.waypoint_sub = rospy.Subscriber(topic_subname + '/waypoint', Point, self.waypoint_callback)
		self.waypoints_pub = rospy.Publisher(topic_subname + '/waypoints_array', Path, queue_size=10)
		self.waypoints_sub = rospy.Subscriber(topic_subname + '/waypoints_array', Path, self.set_path)
		self.gps_listener = rospy.Subscriber(topic_subname + '/fix', NavSatFix, self.gps_callback)
		self.def_prob_sub = rospy.Subscriber(topic_subname + '/damage', TargetDamage, self.damage_callback)
		
	def unregister_subs(self):
	
		self.waypoint_sub.unregister()
		self.waypoints_sub.unregister()
		self.gps_listener.unregister()
		self.def_prob_sub.unregister()
		
	def damage_callback(self, msg_data):
	
		damage = msg_data.damage
		self.total_damage += damage
		
		if self.total_damage >= const.UPPER_DAMAGE_LIMIT:
		
			print(self.name + ' was exploded!')
			self.stop()
			pos = self.get_robot_position()
			set_model_state(self.name, Point(pos.x + 100000, pos.y + 100000, pos.z + 10000), (0, 180, 0, 0))
			self.unregister_subs()
			del self
			
		else:
		
			print(self.name + ' current def prob value: ' + str(self.total_damage))
		
	def set_movespeed(self, ms):
	
		self.ms = ms
		
		self.kp = 0.125 * self.ms
		self.ki = 0.045 * self.ms
		self.kd = 0.0125 * self.ms

		self.i_min = -(5 + self.ms * 15)
		self.i_max = 5 + self.ms * 15


# Getting the position of the robot in 3D space in the Gazebo environment

# Output
# robot_pose: coordinates of the robot in 3D space
	def get_robot_position(self):
	
		robot_pose = get_model_position(self.name)
		return robot_pose

# Getting the orientation three-dimensional vector of the robot in the Gazebo environment

# Output
# dir_vector: robot direction vector
	def get_robot_orientation_vector(self):
	
		robot_pos = self.get_robot_position()
		dir_point_pos = get_link_position(self.dir_point)
		dir_vector = robot_pos.get_dir_vector_between_points(dir_point_pos)
		return dir_vector

# Getting the angle difference between the direction vector of the robot and the vector directed towards the point
# Input
# goal_pos: end point of the second vector

# Output
# angle_difference: angle between vectors in degrees
	def get_angle_difference(self, goal_pos):
	
		rv = self.get_robot_orientation_vector()
		robot_pos = self.get_robot_position()
		goal_vect = get_orientation_vector(robot_pos, goal_pos)
		angle_difference = rv.get_angle_between_vectors(goal_vect)
		return angle_difference

# Moving the robot to a point with a PID controller
# Input
# goal: target point
	def move_with_PID(self, goal):
	
		old_error = 0
		error_sum = 0
		real_error_sum = 0
		robot_pos = self.get_robot_position()
		
		while robot_pos.get_distance_to(goal) > const.DISTANCE_ERROR:
		
			robot_pos = self.get_robot_position()
			error = self.get_angle_difference(goal)
			error_sum += error
			real_error_sum += fabs(error)
			
			if error_sum < self.i_min:
			
				error_sum = self.i_min
				
			elif error_sum > self.i_max:
			
				error_sum = self.i_max
			
			up = self.kp * error
			ui = self.ki * error_sum
			ud = self.kd * (error - old_error)
			old_error = error
			u = up + ui + ud
			self.movement(self.ms, u)
			#self.add_path_gps('a+')
			rospy.sleep(self.pid_delay)


# Rotate the robot towards a point
# Input
# goal: target point
	def turn_to_point(self, goal):
	
		angle_difference = self.get_angle_difference(goal)
		
		while fabs(angle_difference) > const.ANGLE_ERROR:
		
			angle_difference = self.get_angle_difference(goal)
		
			if angle_difference > 0:
		
				self.movement(0, const.ROTATION_SPEED)
		
			else:
		
				self.movement(0, -const.ROTATION_SPEED)
		
		self.stop()

# Stopping the robot
	def stop(self):
	
		msg = Twist()
		msg.linear.x = 0
		msg.angular.z = 0
		self.vel_publisher.publish(msg)

# The movement of the robot with a given linear and angular velocity
# Input
# move_speed: linear velocity (m/s)
# rotation_speed: 
	def movement(self, move_speed, rotation_speed):
	
		msg = Twist()
		msg.linear.x = move_speed
		msg.angular.z = rotation_speed
		self.vel_publisher.publish(msg)

	def waypoint_callback(self, msg):
	
		point = convert_to_point(msg.point)
		self.path.append(point)

	def waypoint_publisher(self, point):
	
		msg = prepare_point_msg(point)
		self.waypoint_pub.publish(msg)

	def waypoints_publisher(self, path):
	
		msg = prepare_path_msg(self.name, path)
		self.waypoints_pub.publish(msg)

# Setting the final path for the robot
# Input
# path: list of path points
	def set_path(self, msg_data):
	
		path = convert_to_path(msg_data.path)
		self.path = path

# The movement of the robot along a given final route
# Start of thread
	def run(self):
	
		start_coords = self.get_gps_coords()
		#self.write_start_coords(start_coords)
		#self.add_path_gps('w+')
		if len(self.path) > 0:
		
			print(self.name + ' path curvature: ' + str(get_path_curvature(self.path)))
		
			for state in self.path:
		
				self.move_with_PID(state)
		
			self.stop()
			
			end_coords = self.get_gps_coords()
			self.write_coords(start_coords, end_coords)
			
			for state in self.path:
			
				pass
				self.add_path_local_coords(state)
			
			print(self.name + ' end GPS coordinates: ' + self.get_gps_coords())
			print('The robot ' + str(self.name) + ' has finished!')
			
		else:
		
			print('Path is empty!')
		
		del self
		
	def gps_callback(self, msg_data):
	
		self.latitude = msg_data.latitude
		self.longitude = msg_data.longitude

	def get_gps_coords(self):
	
		return '(' + str(self.longitude) + ', ' + str(self.latitude) + ')'
		
	def write_start_coords(self, coords):
		
		f = open(const.MAP_STATIC_COORDS_PATH, 'a+')
		f.write(self.name[8:] + ': ' + str(coords) + '\n\n')
		f.close()
	
	def write_coords(self, start_coords, end_coords):
		
		f = open(const.MAP_DYNAMIC_COORDS_PATH, 'a+')
		f.write(self.name[8:] + ': (' + str(start_coords) + ', ' + str(end_coords) + ')\n\n')
		f.close()
		
	def add_path_gps(self, open_mode):
	
		self.path_p_count += 1
		gps_coords = self.get_gps_coords()
		f = open(const.PATHS_DIR_PATH + self.name + '.txt', open_mode)
		f.write(str(self.path_p_count) + ': ' + gps_coords + '\n')
		f.close()
		
	def add_path_local_coords(self, state):

		x = state.x
		y = state.y
		z = state.z
		pos = '(' + str(x) + ', ' + str(y) + ', ' + str(z) + ')'
		f = open(self.local_path_dir, 'a+')
		f.write(pos + '\n')
		f.close()
		
# Creating Pose type ROS message
# Input
# state: position of an object in 3D space
# orient: orientation of an object in 3D space

# Output
# msg: Pose type ROS message 
def make_pose_msg(state, orient):

	msg = Pose()
	msg.position.x = state.x
	msg.position.y = state.y
	msg.position.z = state.z

	if orient:

		msg.orientation.x = orient[0]
		msg.orientation.y = orient[1]
		msg.orientation.z = orient[2]

	return msg

# Visualization of the route in the Gazebo environment
# Input
# path: list of points
# model_directory: the location on the device of the folder with the object model used when rendering the path
# obj_name: the name used when rendering the waypoint
def visualise_path(path, model_directory, obj_name):
	
	if path:
	
		i = 0
	
		for state in path:
	
			i += 1
	
			if state.id:
	
				vertice_name = obj_name + str(i) + '_' + str(state.id)
	
			else:
	
				vertice_name = obj_name + str(i)
	
			spawn_sdf_model(state, model_directory, vertice_name)

# Getting data about a property of the world in the Gazebo environment

# Output
# world_properties:
#  float64 sim_time
#  string[] model_names
#  bool rendering_enabled
#  bool success
#  string status_message
def get_world_properties():
	
	rospy.wait_for_service('/gazebo/get_world_properties/')
	
	try:
	
		get_world_prop = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
		world_properties = get_world_prop()
		return world_properties
	
	except:
	
		print "Service call failed: %s" % e
		return None


# Getting data about a property of the world in the Gazebo environment
# Input
# model_name: model name in Gazebo environment

# Output
# model_properties:
#  string parent_model_name
#  string canonical_body_name
#  string[] body_names
#  string[] geom_names
#  string[] joint_names
#  string[] child_model_names
#  bool is_static
#  bool success
#  string status_message
def get_model_properties(model_name):
	
	rospy.wait_for_service('/gazebo/get_model_properties/')
	
	try:
	
		get_model_prop = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
		model_properties = get_model_prop(model_name)
		return model_properties
	
	except rospy.ServiceException, e:
	
		print "Service call failed: %s" % e
		return None

# Getting position of an object in the Gazebo environment
# Input
# model_name: model name in Gazebo environment

# Output
# pose: position of an object in 3D space
def get_model_position(model_name):
	object_state = get_model_state(model_name)
	
	if object_state:
	
		x = object_state.pose.position.x
		y = object_state.pose.position.y
		z = object_state.pose.position.z
		pose = PointGeom.Point(x, y, z)
		return pose
	
	else:
		return None
		
# Getting orientation of an object in the Gazebo environment
# Input
# model_name: model name in Gazebo environment

# Output
# pose: position of an object in 3D space
def get_model_orientation(model_name):
	object_state = get_model_state(model_name)
	
	if object_state:
	
		roll = object_state.pose.orientation.x
		pitch = object_state.pose.orientation.y
		yaw = object_state.pose.orientation.z
		return roll, pitch, yaw
	
	else:
		return None, None, None

# Getting the position of a link in the Gazebo environment
# Input
# link_name: link name in Gazebo environment

# Output
# state:
#  gazebo_msgs/LinkState link_state
#  bool success
#  string status_message 
def get_link_position(link_name):
	rospy.wait_for_service('/gazebo/get_link_state')
	
	try:
	
		get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		link_state = get_link_state(link_name, 'world')
		pos = link_state.link_state.pose.position
		x = pos.x
		y = pos.y
		z = pos.z
		state = PointGeom.Point(x, y, z)
		return state
	
	except rospy.ServiceException, e:
	
		print "Service call failed: %s" % e
		return None

def get_robot_orientation_vector(robot_name):
	
		robot_pos = get_model_position(robot_name)
		dir_point_pos = get_link_position(robot_name + const.DIR_POINT_SUFFIX)
		dir_vector = robot_pos.get_dir_vector_between_points(dir_point_pos)
		return dir_vector

# Getting the state of a model in the Gazebo environment (including model orientation)
# Input
# model_name: model name in Gazebo environment

# Output
# model_coordinates:
#  geometry_msgs/Pose pose
#  geometry_msgs/Twist twist
#  bool success
#  string status_message
def get_model_state(model_name):
	rospy.wait_for_service('/gazebo/get_model_state')
	
	try:
	
		get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		model_coordinates = get_state(model_name, 'world')
		return model_coordinates
	
	except rospy.ServiceException, e:
	
		print "Service call failed: %s" % e
		return None
		
def delete_model(model_name):

	rospy.wait_for_service('/gazebo/delete_model')
	
	try:
	
		del_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
		ret = del_model(str(model_name))
		print(model_name + ' model was deleted.')
		print(ret)
		
	except rospy.ServiceException, e:
		
		print "Service call failed: %s" % e
		
# Generation of the object described in the .sdf file in Gazebo the environment
# Input
# model_name: the name of the object in the Gazebo environment
# model_directory: path to the folder where the model is stored
# state: the position at which the object will be spawned
def spawn_sdf_model(state, model_directory, model_name):
	rospy.wait_for_service('/gazebo/spawn_sdf_model')
	
	try:
	
		f = open(model_directory)
		point_xml = f.read()
		f.close()
		pose = make_pose_msg(state, None)
		spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
		resp = spawn_model(model_name, point_xml, '', pose, 'world')
	
	except rospy.ServiceException, e:
	
		print "Service call failed: %s" % e

def spawn_target(model_name, state, orient):

	state.set_z(float(state.z + const.SPAWN_HEIGHT_OFFSET))
	spawn_urdf_model(model_name, const.ROBOT_MODEL_PATH, state, orient)
	
def spawn_urdf_model(model_name, model_directory, state, orient):

	x = orient[0]
	y = orient[1]
	z = orient[2]
	w = orient[3]
	rospy.wait_for_service('/gazebo/spawn_urdf_model')
	
	try:
	
		spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
		spawn_model_client(model_name, open(model_directory, 'r').read(),
					"/" + model_name, Pose(position=Point(state.x, state.y, state.z), orientation=Quaternion(x, y, z, w)), "world")
	
	except rospy.ServiceException, e:
	
		print "Service call failed: %s" % e

# Setting the object of the required position and orientation in space
# model_name: the name of the object in the Gazebo environment
# pose: position of an object in 3D space
# orient: orientation of an object in 3D space
def set_model_state(model_name, pose, orient):
	
	state_msg = prepare_model_state_msg(model_name, pose, orient)
	
	rospy.wait_for_service('/gazebo/set_model_state')
	
	try:
	
		set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
		resp = set_state(state_msg)
	
	except:
	
		print "Service call failed: %s" % e	

def calc_twist(model_name, pose, orient):

	prev_pos = get_model_position(model_name)
	roll, pitch, yaw = get_model_orientation(model_name)
	
	lin_x = pose.x - prev_pos.x
	lin_y = pose.y - prev_pos.y
	lin_z = pose.z - prev_pos.z
	
	ang_x = orient[0] - roll
	ang_y = orient[1] - pitch
	ang_z = orient[2] - yaw
	
	return (lin_x, lin_y, lin_z), (ang_x, ang_y, ang_z)
	

def prepare_model_state_msg(model_name, pose, orient):

	state_msg = ModelState()
	
	state_msg.model_name = model_name
	
	#state_msg.pose.position.x = pose.x
	#state_msg.pose.position.y = pose.y
	#state_msg.pose.position.z = pose.z
	
	state_msg.pose.orientation.x = orient[0]
	state_msg.pose.orientation.y = orient[1]
	state_msg.pose.orientation.z = orient[2]
	state_msg.pose.orientation.w = orient[3]
	
	lin, ang = calc_twist(model_name, pose, orient)
	
	state_msg.twist.linear.x = lin[0]
	state_msg.twist.linear.y = lin[1]
	state_msg.twist.linear.z = lin[2]
	
	state_msg.twist.angular.x = ang[0]
	state_msg.twist.angular.y = ang[1]
	state_msg.twist.angular.z = ang[2]
	
	return state_msg

# Getting a three-dimensional vector directed from point p1 to point p2
# Input
# p1: vector origin point
# p2: vector end point

# Output
# dir_vector: vector between given points
def get_orientation_vector(p1, p2):
	
	det_x = p2.x - p1.x
	det_y = p2.y - p1.y
	dir_vector = PointGeom.Vector2d(det_x, det_y)
	return dir_vector

def get_3d_orientation_vector(p1, p2):
	
	x = p2.x - p1.x
	y = p2.y - p1.y
	z = p2.z - p1.z
	dir_vector = vector3d.vector.Vector(x, y, z).normalize()
	return dir_vector

# Getting the angle between 3D vectors
# Input
# v1: first 3D vector
# v2: second 3D vector

# Output
# angle: angle between vectors in degrees
def get_3d_angle(p1, p2, p3):
	
	v1 = get_3d_orientation_vector(p2, p1)
	v2 = get_3d_orientation_vector(p2, p3)
	z_axis = vector3d.vector.Vector(0, 0, 1)
	angle = vector3d.vector.angle(v1, v2)
	
	if not angle == 0:
	
		i = v1.y * v2.z - v1.z * v2.y
		j = v1.x * v2.z - v1.z * v2.x
		k = v1.x * v2.y - v1.y * v2.x
		v3 = vector3d.vector.Vector(i, j, k).normalize()
		z_angle = vector3d.vector.angle(v3, z_axis)
	
		if z_angle > 90:
	
			angle = - angle
	
	return angle

# Convert 3D vector to 2D
# Input
# vector_3d: 3D vector

# Output
# vect2D: 2D vector representation
def convert_3d_to_2d_vect(vector_3d):
	
	vect2d = PointGeom.Vector2d(vector_3d.x, vector_3d.y)
	return vect2d

# Converting msg to Point object
# Input
# msg: message containing the coordinates of the point

# Output
# point: Point object
def convert_to_point(msg):
	
	x = msg.x
	y = msg.y
	z = msg.z
	point = PointGeom.Point(x, y, z)
	return point

# Output
# max_curvature: path curvature
def get_path_curvature(path):
	
	max_curvature = 0
	
	for i in range(0, len(path) - 2):
	
		p1 = path[i]
		p2 = path[i + 1]
		p3 = path[i + 2]
		v1 = p1.get_dir_vector_between_points(p2)
		v2 = p2.get_dir_vector_between_points(p3)
		if not((v1.x == 0 and v1.y == 0) or (v2.x == 0 and v2.y == 0)):
		
			angle_difference = fabs(v1.get_angle_between_vectors(v2))
		
			if angle_difference > max_curvature:
		
				max_curvature = angle_difference
	
	return max_curvature

def prepare_path_msg(name, path):
	msg = Path()
	msg.path = []
	msg.robot_name = name
	
	for state in path:
	
		point = Point()
		point.x = state.x
		point.y = state.y
		point.z = state.z
		msg.path.append(point)
	
	return msg

def prepare_point_msg(p):
	
	msg = Point()
	msg.x = p.x
	msg.y = p.y
	msg.z = p.z
	return msg

def convert_to_path(msg):
	
	path = []
	
	for state in msg:
	
		x = state.x
		y = state.y
		z = state.z
		p = PointGeom.Point(x, y, z)
		path.append(p)
	
	return path
	
# Calculating path length
# Input
# path: path vertex array

# Output
# path_len: path length
def get_path_length(path):

	path_len = 0

	for i in range(0, len(path) - 1):

		current_v = path[i]
		next_v = path[i + 1]
		dist = current_v.get_distance_to(next_v)
		path_len += dist

	return path_len
	
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
	
	return new_path
	
