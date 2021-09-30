# Module for interacting with the Gazebo simulation environment

#!/usr/bin/env python

import rospy
import path_planning.Point as PointGeom
from targets_path_planning.msg import Path
import GazeboConstants as const
import time
from math import sqrt, fabs, sin, asin, pi, cos, acos
from gazebo_msgs.msg import ModelState, ODEPhysics
from geometry_msgs.msg import Pose, Twist, Quaternion, Point, Vector3
from std_msgs.msg import Float64, Time, Duration
from gazebo_msgs.srv import SetModelState, GetModelState, GetModelProperties, SpawnModel, GetWorldProperties, GetLinkState, DeleteModel
import copy
import random


		
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
	
def get_robot_orientation_vector(robot_name):

	robot_pos = get_model_position(robot_name)
	dir_point = robot_name + const.DIR_POINT_SUFFIX
	dir_point_pos = get_link_position(dir_point)
	dir_vector = robot_pos.get_dir_vector_between_points(dir_point_pos)
	return dir_vector
	
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
