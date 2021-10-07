import rospy
from targets_path_planning.msg import Path, WorkPath, Charge
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
	
	def __init__(self, robot_name, role, b_trackers):
	
		thr.Thread.__init__(self)
		self.name = robot_name
		self.b_trackers = b_trackers
		self.bt = self.b_trackers[self.name]
		self.id = self.name[self.name.find('t') + 1:]
		self.init_topics()
		self.pid_delay = rospy.Duration(0, const.PID_NSEC_DELAY)
		msg = Twist()
		rospy.sleep(self.pid_delay)
		self.vel_publisher.publish(msg)
		self.set_movespeed(const.MOVEMENT_SPEED)
		self.dir_point = robot_name + const.DIR_POINT_SUFFIX
		self.paths = []
		self.workpoints = []
		self.partner_name = None
		self.mode = None
		self.role = role
		self.finished = False
		self.charging = False
		self.waiting = False
		
	def init_topics(self):
		
		topic_subname = '/' + self.name
		
		self.vel_publisher = rospy.Publisher(topic_subname + '/cmd_vel', Twist, queue_size=10)
		
		self.paths_pub = rospy.Publisher(topic_subname + '/waypoints_array', WorkPath, queue_size=10)
		self.paths_sub = rospy.Subscriber(topic_subname + '/waypoints_array', WorkPath, self.set_path)
		
		self.workpoints_pub = rospy.Publisher(topic_subname + '/workpoints_array', Path, queue_size=10)
		self.workpoints_sub = rospy.Subscriber(topic_subname + '/workpoints_array', Path, self.set_work_points_path)

	def unregister_subs(self):
	
		self.waypoint_sub.unregister()
		self.waypoints_sub.unregister()
		self.gps_listener.unregister()
		self.def_prob_sub.unregister()
		self.workpoints_sub.unregister()

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
			self.robot_waiting()
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


	def follow_the_route(self, path):
	
		self.mode = "movement"
		self.set_movespeed(const.MOVEMENT_SPEED)
		last_pos = self.get_robot_position()

		for state in path:

			self.move_with_PID(state)
			dist = last_pos.get_distance_to(state)
			charge_loss = -(dist * const.MOVE_CHARGE_LOSS_COEF)
			last_pos = state
			self.bt.power_change(charge_loss)
			self.check_battery()
	
		self.stop()
	
	def robot_waiting(self):
	
		while self.waiting:
		
			print(self.name + ' fucked up')
			pass
	
	def wait(self):
	
		#print(self.name + ' is waiting.')
		self.waiting = True
		self.stop()
		
	def stop_waiting(self):
	
		self.waiting = False

	def perform_worker_mission(self, path):

		self.follow_the_route(path)
		self.perform_the_task()
		
	def perform_the_task(self):
	
		print('\n>>> ' + self.name + ' started performing task!')
		task_completion = 0
		start_time = rospy.get_time()
		last_step_time = start_time
		exec_duration = const.TASK_EXEC_SPEED
		total_consumption = 0
		
		while task_completion < 100:
		
			cur_time = rospy.get_time()
			all_time = cur_time - start_time
			task_completion = float((all_time / exec_duration) * 100)
			#print('Completion of the task by ' + self.name + ' :' + str(task_completion))
			
			step_time = cur_time - last_step_time
			last_step_time = cur_time
			power_consumption = (step_time / exec_duration) * const.TASK_ENERGY_COST
			total_consumption += power_consumption
			self.bt.power_change(-power_consumption)
			self.check_battery()
			
		print(self.name + ' task energy consumption: ' + str(total_consumption))
			
		
	def perform_charger_mission(self, path):
	
		self.follow_the_route(path)
		dock_point = self.dock_points[0]
		self.dock_points.pop(0)
		
		dock_path = self.plan_path_to_dock_point(dock_point)
		self.follow_the_route(dock_path)
		partner = self.rechargeable_robots[0]
		self.rechargeable_robots.pop(0)
		self.set_docking_mode(partner)
		self.docking()
		self.start_charging()
		
	def plan_path_to_dock_point(self, dock_point):
	
		pass
		return None
		
	def start_charging(self):

		self.mode = "charging"
		s_ch_time = rospy.get_time()
		rechargeable_bt = self.b_trackers[self.partner]
		
		while self.mode == "charging":
			
			cur_ch_time = rospy.get_time()
			time_diff = cur_ch_time - s_ch_time
			s_ch_time = cur_ch_time
			charge_received = const.CHARGING_SPEED * time_diff
			rechargeable_bt.power_change(charge_received)
			self.bt.recharge_power_change(-charge_received)
			rech_b_level = rechargeable_bt.battery
			
			if rech_b_level >= const.HIGH_LIMIT_BATTERY and self.check_recharge_battery():
			
				self.mode = None

	def print_bt_charges(self):
	
		print('\n>>> ' + self.name + ' <<<')
		for name in self.b_trackers:
		
			self.get_robot_battery_level(name)

	def get_robot_battery_level(self, name):
	
		bt = self.b_trackers[name]
		b_level = bt.battery
		re_b_level = bt.recharge_battery
		#print(name + ' move battery level: ' + str(m_level) + '% | task battery level: ' + str(t_level) + '%')
		return b_level, re_b_level

	def get_battery_level(self):

		b_level, re_b_level = self.get_robot_battery_level(self.name)
		return b_level, re_b_level

	def check_battery(self):

		b_level, re_b_level = self.get_battery_level()
		
		if b_level < const.LOWER_LIMIT_BATTERY:
		
			self.stop
			
			while b_level < const.HIGH_LIMIT_BATTERY:
			
				b_level, re_b_level = self.get_battery_level()
				
	def check_recharge_battery(self):

		b_level, re_b_level = self.get_battery_level()
		
		if re_b_level < const.LOWER_LIMIT_RECHARGE_BATTERY:
			
			return False
			
		else:
		
			return True

	def set_movespeed(self, ms):
	
		self.ms = ms
		
		self.kp = 0.0625 * self.ms
		self.ki = 0.00625 * self.ms
		self.kd = 0.02 * self.ms

		self.i_min = -(10 + self.ms * 15)
		self.i_max = 10 + self.ms * 15

	def get_distance_to_partner(self):
	
		self_pos = self.get_robot_position()
		partner_pos = gc.get_model_position(self.partner_name)
		dist = self_pos.get_distance_to(partner_pos)
		return dist
		
	def get_angle_difference_with_parther(self):
	
		partner_orient = gc.get_robot_orientation_vector(self.partner_name)
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
		self.mode = None
		print(self.name + ' successfully connected to ' + str(self.partner_name) + '.')

	def set_docking_mode(self, partner_name):
	
		self.partner_name = partner_name
		self.mode = "docking"

# The movement of the robot along a given final route
# Start of thread
	def run(self):
		
		i = 0
		for path in self.paths:
		
			i += 1
			gc.visualise_path(path, const.RED_VERTICE_PATH, self.name + '_' + str(i) + '_')
			
		#self.print_bt_charges()
		if self.role == "charger":

			for path in self.paths:
			
				self.perform_charger_mission(path)
				
			print('Charger ' + str(self.name) + ' has finished!')
				

		elif self.role == "worker":
		
			for path in self.paths:
			
				self.perform_worker_mission(path)
				
			print('Worker ' + str(self.name) + ' has finished!')
			
		else:
		
			print(self.name + ' role is unknown.')
			
		self.finished = True
		
		
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
