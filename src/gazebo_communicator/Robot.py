import cv2
import rospy
from targets_path_planning.msg import Path, WorkPath, Charge
import path_planning.Point as PointGeom
import GazeboConstants as const
import GazeboCommunicator as gc
from math import fabs
import threading as thr
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32
import pickle
from cv_bridge import CvBridge
from aruco_middle_point_position.utils import detect_show_markers

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
		self.init_camera()
		self.pid_delay = rospy.Duration(0, const.PID_NSEC_DELAY)
		msg = Twist()
		rospy.sleep(self.pid_delay)
		self.vel_publisher.publish(msg)
		self.set_movespeed(const.MOVEMENT_SPEED)
		self.dir_point = robot_name + const.DIR_POINT_SUFFIX
		self.mode = None
		self.finished = False
		self.charging = False
		self.waiting = False
		self.dodging = False
		
	def init_topics(self):
		
		self.topic_subname = '/' + self.name
		
		self.vel_publisher = rospy.Publisher(self.topic_subname + '/cmd_vel', Twist, queue_size=10)
		
		self.paths_pub = rospy.Publisher(self.topic_subname + '/waypoints_array', WorkPath, queue_size=10)
		self.paths_sub = rospy.Subscriber(self.topic_subname + '/waypoints_array', WorkPath, self.set_path)
		
		self.workpoints_pub = rospy.Publisher(self.topic_subname + '/workpoints_array', Path, queue_size=10)
		self.workpoints_sub = rospy.Subscriber(self.topic_subname + '/workpoints_array', Path, self.set_work_points_path)
		self.dist_sub = rospy.Subscriber(self.topic_subname + "/distance", Float32, self.dist_callback)

	def unregister_subs(self):
	
		self.waypoint_sub.unregister()
		self.waypoints_sub.unregister()
		self.gps_listener.unregister()
		self.def_prob_sub.unregister()
		self.workpoints_sub.unregister()

	def init_camera(self):
		self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
		self.parameters = cv2.aruco.DetectorParameters_create()

		with open(const.PATH_CAMERA_PARAMS, 'rb') as f:
			camera_param = pickle.load(f)
		self.camera_mtx, self.dist_coefficients, _, _, _, _ = camera_param
		for k in self.dist_coefficients:
			k = 0

		self.image_sub = rospy.Subscriber(self.topic_subname + "/camera1/image_raw", Image, self.callback_image)
		self.pub = rospy.Publisher(self.topic_subname +  "/distance", Float32, queue_size=10)
		self.br = CvBridge()

	def dist_callback(self, msg_data):

		self.aruco_dist = msg_data
		print('\n>>> <<<')

	def callback_image(self, image):
		key = cv2.waitKey(1) & 0xFF

		frame_bgr = self.br.imgmsg_to_cv2(image)
		frame_grey = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

		# Aruco detection
		try:
			middle_point_pose, middle_point_orient, dist1, dist2, rvec1, rvec2 = detect_show_markers(frame_bgr, \
			frame_grey, self.aruco_dict, self.parameters, self.camera_mtx, self.dist_coefficients)
			distance = middle_point_pose[0][0][2]
			self.pub.publish(distance)
			# if distance < 0.54:
			# 	print('Coordinates of center: ', middle_point_pose)
			# 	print('Distance to left marker: ', dist1)
			# 	print('Distance to right marker: ', dist2)
			# 	print('Orientation center: ', middle_point_orient)
		# If markers was not detected:
		except Exception as e:
			pass

		frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
		cv2.imshow("camera", frame_rgb)
		cv2.waitKey(1)

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
	
		old_error = self.get_angle_difference(goal)
		error_sum = 0
		robot_pos = self.get_robot_position()
		
		while robot_pos.get_distance_to(goal) > const.DISTANCE_ERROR:
		
			robot_pos = self.get_robot_position()
			u, old_error, error_sum = self.calc_control_action(goal, old_error, error_sum)
			self.movement(self.ms, u)
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

	def move_energy_cons(self, pos, last_pos):
	
		dist = pos.get_distance_to(last_pos)
		charge_loss = -(dist * const.MOVE_CHARGE_LOSS_COEF)
		self.bt.power_change(charge_loss)

# Rotate the robot towards a point
# Input
# goal: target point
	def turn_to_point(self, goal):
	
		angle_difference = self.get_angle_difference(goal)
		
		while fabs(angle_difference) > const.ANGLE_ERROR:
		
			angle_difference = self.get_angle_difference(goal)
		
			if angle_difference > 0:
		
				self.movement(0, const.DOCKING_ROTATION_SPEED)
		
			else:
		
				self.movement(0, -const.DOCKING_ROTATION_SPEED)
		
		self.stop()
		
	def follow_the_route(self, path):
	
		self.change_mode("movement")
		self.set_movespeed(const.MOVEMENT_SPEED)

		for state in path:

			self.move_with_PID(state)
	
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
	
		msg = prepare_paths_msg(path)
		self.paths_pub.publish(msg)
		
	def workpoints_publisher(self, path):
	
		msg = prepare_path_msg(path)
		self.workpoints_pub.publish(msg)

	def set_work_points_path(self, msg_data):
	
		workpoints = convert_to_path(msg_data)
		self.workpoints = workpoints

# Setting the final path for the robot
# Input
# path: list of path points
	def set_path(self, msg_data):
	
		self.paths = []
		
		for path_msg in msg_data.paths:
				
			path = convert_to_path(path_msg)
			self.paths.append(path)
	
	def is_waiting(self):
	
		while self.waiting:
		
			pass
	
	def wait(self):
	
		#print(self.name + ' is waiting.')
		self.waiting = True
		self.stop()
		
	def stop_waiting(self):
	
		self.waiting = False
		self.change_mode("movement")

	def is_dodging(self):

		if self.dodging:
		
			while self.dodging:
		
				pass

	def stop_dodging(self):

		self.dodging = False

	def print_bt_charges(self):
	
		print('\n>>> ' + self.name + ' <<<')
		for name in self.b_trackers:
		
			self.get_robot_battery_level(name)


	def set_movespeed(self, ms):
	
		self.ms = ms
		
		self.kp = 0.0625 * self.ms
		self.ki = 0.00625 * self.ms
		self.kd = 0.02 * self.ms

		self.i_min = -(10 + self.ms * 15)
		self.i_max = 10 + self.ms * 15



# The movement of the robot along a given final route
# Start of thread
	def run(self):
			
		pass
		
		
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
	for state in msg.path:
	
		x = state.x
		y = state.y
		z = state.z
		p = PointGeom.Point(x, y, z)
		path.append(p)
	
	return path
	
def prepare_path_msg(path):
	msg = Path()
	msg.path = []
	
	for state in path:
	
		point = Point()
		point.x = state.x
		point.y = state.y
		point.z = state.z
		msg.path.append(point)
	
	return msg
	
def prepare_paths_msg(paths):

	msg = WorkPath()
	msg.paths = []
	
	for path in paths:
	
		path_msg = prepare_path_msg(path)
		msg.paths.append(path_msg)
		
	return msg
