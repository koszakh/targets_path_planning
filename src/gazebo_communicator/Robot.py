import rospy
from targets_path_planning.msg import Path, TargetDamage
import path_planning.Point as PointGeom
import GazeboConstants as const
import GazeboCommunicator as gc
from math import fabs
import threading as thr
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Twist

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
		self.pid_delay = rospy.Duration(0, const.PID_NSEC_DELAY * 2)
		msg = Twist()
		rospy.sleep(self.pid_delay)
		self.vel_publisher.publish(msg)
		self.set_movespeed(const.MOVEMENT_SPEED)
		self.dir_point = robot_name + const.DIR_POINT_SUFFIX
		self.path = []
		self.latitude = None
		self.longitude = None
		self.total_damage = 0
		self.path_p_count = 0
		self.local_path_dir = const.PATHS_DIR + '/paths6_local/' + self.name + '.txt'
		self.partner_name = None
		self.mode = None
	
	def get_wheel_distance(self):
	
		fl_name = self.name + "::p3at_front_left_wheel"
		fr_name = self.name + "::p3at_front_right_wheel"
		bl_name = self.name + "::p3at_back_left_wheel"
		br_name = self.name + "::p3at_back_right_wheel"
		fl_pos = gc.get_link_position(fl_name)
		fr_pos = gc.get_link_position(fr_name)
		bl_pos = gc.get_link_position(bl_name)
		br_pos = gc.get_link_position(br_name)
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
			gc.set_model_state(self.name, Point(pos.x + 100000, pos.y + 100000, pos.z + 10000), (0, 180, 0, 0))
			self.unregister_subs()
			del self
			
		else:
		
			print(self.name + ' current def prob value: ' + str(self.total_damage))


# Getting the position of the robot in 3D space in the Gazebo environment

# Output
# robot_pose: coordinates of the robot in 3D space
	def get_robot_position(self):
	
		robot_pose = gc.get_model_position(self.name)
		return robot_pose

# Getting the orientation three-dimensional vector of the robot in the Gazebo environment

# Output
# dir_vector: robot direction vector
	def get_robot_orientation_vector(self):
	
		dir_vector = gc.get_robot_orientation_vector(self.name)
		return dir_vector

# Getting the angle difference between the direction vector of the robot and the vector directed towards the point
# Input
# goal_pos: end point of the second vector

# Output
# angle_difference: angle between vectors in degrees
	def get_angle_difference(self, goal_pos):
	
		rv = self.get_robot_orientation_vector()
		robot_pos = self.get_robot_position()
		goal_vect = robot_pos.get_dir_vector_between_points(goal_pos)
		angle_difference = rv.get_angle_between_vectors(goal_vect)
		return angle_difference

# Moving the robot to a point with a PID controller
# Input
# goal: target point
	def move_with_PID(self, goal):
	
		old_error = 0
		error_sum = 0
		robot_pos = self.get_robot_position()
		
		while robot_pos.get_distance_to(goal) > const.DISTANCE_ERROR:
		
			robot_pos = self.get_robot_position()
			u, old_error, error_sum = self.calc_control_action(goal, old_error, error_sum)
			self.movement(self.ms, u)
			#self.add_path_gps('a+')
			rospy.sleep(self.pid_delay)

	def calc_control_action(self, goal, old_error, error_sum):
		error = self.get_angle_difference(goal)
		error_sum += error
		
		if error_sum < self.i_min:
		
			error_sum = self.i_min
			
		elif error_sum > self.i_max:
		
			error_sum = self.i_max

		up = self.kp * error
		ui = self.ki * error_sum
		ud = self.kd * (error - old_error)
		old_error = error
		u = up + ui + ud
		return u, old_error, error_sum

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
		print(self.name + ' path curvature: ' + str(get_path_curvature(path)))
		self.path = path
		#self.write_path()

	def follow_the_route(self):
	
		for state in self.path:
		
			self.move_with_PID(state)
	
		self.stop()
		print('The robot ' + str(self.name) + ' has finished!')

	def get_end_gps_coords(self):
	
		pos = self.path[len(self.path) - 1]
		name = self.name + '_1'
		spawn_target(name, pos, (0, 0, 0, 0))
		sub_robot = Robot(name)
		delay = rospy.Duration(1, 0)
		rospy.sleep(delay)
		sr_coords = sub_robot.get_gps_coords()
		return sr_coords
		
	def set_movespeed(self, ms):
	
		self.ms = ms
		
		self.kp = 0.0625 * self.ms
		self.ki = 0.00625 * self.ms
		self.kd = 0.02 * self.ms

		self.i_min = -(10 + self.ms * 15)
		self.i_max = 10 + self.ms * 15

	def write_path(self):
	
		f = open(self.local_path_dir, 'w+')
		f.close()
	
		if self.path:

			for state in self.path:

				self.add_path_local_coords(state)

	def get_distance_to_partner(self):
	
		self_pos = self.get_robot_position()
		partner_pos = gc.get_model_position(self.partner_name)
		dist = self_pos.get_distance_to(partner_pos)
		return dist
		
	def get_angle_difference_with_parther(self):
	
		partner_orient = get_robot_orientation_vector(self.partner_name)
		self_orient = self.get_robot_orientation_vector()
		angle_difference = self_orient.get_angle_between_vectors(partner_orient)

	def docking(self):
	
		self.set_movespeed(const.DOCKING_SPEED)
		dist = self.get_distance_to_partner()
		partner_pos = gc.get_model_position(self.partner_name)
		self.turn_to_point(partner_pos)
		old_error = 0
		error_sum = 0
		
		while dist > const.DOCKING_THRESHOLD:
		
			angle_difference = self.get_angle_difference_with_parther()
			
			dist = self.get_distance_to_partner()
			
			u, old_error, error_sum = self.calc_control_action(partner_pos, old_error, error_sum)
			self.movement(self.ms, u)

		self.stop()
		print(self.name + ' successfully connected to ' + str(self.partner_name) + '.')

	def set_docking_mode(self, partner_name):
	
		self.partner_name = partner_name
		self.mode = "docking"

# The movement of the robot along a given final route
# Start of thread
	def run(self):

		start_coords = self.get_gps_coords()
		#self.write_start_coords(start_coords)
		#self.add_path_gps('w+')
		
		if len(self.path) > 0:
				
			#end_coords = self.get_end_gps_coords()
			#self.write_coords(start_coords, end_coords)
			
			self.follow_the_route()
			
			if self.mode == "docking":
			
				self.docking()
			
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
		f.write(self.name[8:] + ': (' + str(start_coords) + ', ' + end_coords + ')\n\n')
		f.close()
		print(self.name + ' end GPS coordinates: ' + end_coords)
		
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
