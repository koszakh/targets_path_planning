# Module for interacting with the Gazebo simulation environment

#!/usr/bin/env python

import rospy
import rospkg
from path_planning.Point import Point, Vector2d
import GazeboConstants as const
from time import sleep
from math import sqrt, fabs, sin, asin, pi, cos, acos
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64, Time, Duration
from gazebo_msgs.srv import SetModelState, GetModelState, ApplyJointEffort, GetModelProperties, SpawnModel, GetWorldProperties, GetLinkState
import vector3d
import threading as thr

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
        topic_name = '/sim_' + robot_name + '/cmd_vel'
        self.vel_publisher = rospy.Publisher(topic_name, Twist, queue_size=10)
        msg = Twist()
        self.vel_publisher.publish(msg)
        self.name = robot_name
        self.dir_point = robot_name + '::dir_point'
        self.init_path = None
        self.final_path = None
        self.finished_planning = False
        self.last_goal = None

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
        dir_vector = get_orientation_vector(robot_pos, dir_point_pos)
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
        rv_2d = convert_3d_to_2d_vect(rv)
        dist_2d = goal_pos.get_2d_distance(robot_pos)
        sub_point = Point(robot_pos.x + rv_2d.x * dist_2d, robot_pos.y + rv_2d.y * dist_2d, goal_pos.z)
        new_rv = get_orientation_vector(robot_pos, sub_point)
        angle_difference = get_3d_angle(new_rv, goal_vect)
        return angle_difference

# Moving the robot to a point with a PID controller
# Input
# goal: target point
    def move_with_PID(self, goal):
        """
        Function that regulates control effect on the wheels to align trajectory
        """
        old_error = 0
        error_sum = 0
        real_error_sum = 0
        while self.get_robot_position().get_distance_to(goal) > const.DISTANCE_ERROR:
            #self.control_move_incline()
            error = self.get_angle_difference(goal)
            error_sum += error
            real_error_sum += fabs(error)
            if error_sum < const.I_MIN:
                error_sum = const.I_MIN
            elif error_sum > const.I_MAX:
                error_sum = const.I_MAX
            up = const.KP * error
            ui = const.KI * error_sum
            ud = const.KD * (error - old_error)
            old_error = error
            u = up + ui + ud
            self.movement(const.MOVEMENT_SPEED, u)
            sleep(const.PID_DELAY)
        self.stop()

# Robot movement to a point, including a preliminary rotation to the required angle
# Input
# goal: target point
    def move_to_point(self, goal):
        angle_difference = self.get_angle_difference(goal)
        if fabs(angle_difference) > const.ANGLE_ERROR:
            self.turn_to_point(goal)
        self.move_with_PID(goal)

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
            #sleep(0.1)
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

# Setting the initial path for the robot
# Input
# path: list of path points
    def set_init_path(self, path):
        self.init_path = path

# Setting the final path for the robot
# Input
# path: list of path points
    def set_final_path(self, path):
        print(self.name + ' path length: ' + str(len(path)))
        self.final_path = path

# The movement of the robot along a given final route
# Start of thread
    def run(self):
        print('Robot ' + self.name + ' started moving along the path')
        if self.final_path:
            for state in self.final_path:
                self.move_to_point(state)
        else:
            print('Path is empty!')

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
            spawn_sdf_model(vertice_name, model_directory, state)

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

# Getting the position of an object in the Gazebo environment
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
        pose = Point(x, y, z)
        return pose
    else:
        return None

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
        state = Point(x, y, z)
        return state
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return None

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

# Generation of the object described in the .sdf file in Gazebo the environment
# Input
# model_name: the name of the object in the Gazebo environment
# model_directory: path to the folder where the model is stored
# state: the position at which the object will be spawned
def spawn_sdf_model(model_name, model_directory, state):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        f = open(model_directory)
        point_xml = f.read()
        pose = make_pose_msg(state, None)
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp = spawn_model(model_name, point_xml, '', pose, 'world')
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

# Setting the object of the required position and orientation in space
# model_name: the name of the object in the Gazebo environment
# pose: position of an object in 3D space
# orient: orientation of an object in 3D space
def set_model_state(model_name, pose, orient):
    state_msg = ModelState()
    state_msg.model_name = model_name
    state_msg.pose.position.x = pose.x
    state_msg.pose.position.y = pose.y
    state_msg.pose.position.z = pose.z
    state_msg.pose.orientation.x = orient[0]
    state_msg.pose.orientation.y = orient[1]
    state_msg.pose.orientation.z = orient[2]
    state_msg.pose.orientation.w = orient[3]
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
    except:
        print "Service call failed: %s" % e

# Getting a three-dimensional vector directed from point p1 to point p2
# Input
# p1: vector origin point
# p2: vector end point

# Output
# dir_vector: vector between given points
def get_orientation_vector(p1, p2):
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
def get_3d_angle(v1, v2):
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
    vect2d = Vector2d(vector_3d.x, vector_3d.y)
    return vect2d
