from targets_path_planning.msg import ArucoDist
from gazebo_communicator.Robot import Robot
import gazebo_communicator.GazeboCommunicator as gc
import GazeboConstants as const
import path_planning.Constants as pp_const
import threading as thr
import rospy
from math import fabs
import dubins
import random
import cv2
from time import sleep
import pickle
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from aruco_middle_point_position.utils import detect_show_markers

class Charger(Robot):

	def __init__(self, name, trackers):

		thr.Thread.__init__(self)
		self.name = name
		self.trackers = trackers
		self.init_topics()
		self.init_camera()
		self.init_aruco_topics()
		self.pid_delay = rospy.Duration(0, const.PID_NSEC_DELAY)
		self.bt = self.trackers[name]
		self.pre_ch_points = None
		self.to_ch_p_paths = []
		self.to_base_paths = []
		self.rech_robots = []
		self.finished = False
		self.waiting = False
		self.mode = "stop"	
		self.dock_path = []
		self.dodging = False
		self.aruco_dist = None

	def init_aruco_topics(self):

		#self.dist_sub = rospy.Subscriber(self.topic_subname + '/distance', ArucoDist, self.dist_callback)
		self.image_sub = rospy.Subscriber(self.topic_subname + '/camera1/image_raw', Image, self.callback_image)
		#self.dist_pub = rospy.Publisher(self.topic_subname + '/distance', ArucoDist, queue_size=10)
		self.br = CvBridge()
		
	def init_camera(self):
	
		self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
		self.parameters = cv2.aruco.DetectorParameters_create()

		with open(const.PATH_CAMERA_PARAMS, 'rb') as f:
			camera_param = pickle.load(f)
		self.camera_mtx, self.dist_coefficients, _, _, _, _ = camera_param
		
		for k in self.dist_coefficients:
		
			k = 0

	def dist_callback(self, msg_data):

		print('Aruco updated')
		#self.aruco_dist = msg_data.dist
		#self.left_dist = msg_data.left_dist
		#self.right_dist = msg_data.right_dist
		
	def set_aruco_dist(self, dist, left_dist, right_dist):

		#print('Aruco updated')
		self.aruco_dist = dist
		self.left_dist = left_dist
		self.right_dist = right_dist

	def callback_image(self, image):
		key = cv2.waitKey(1) & 0xFF

		frame_bgr = self.br.imgmsg_to_cv2(image)
		frame_grey = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
		# Aruco detection
		try:
			middle_point_pose, middle_point_orient, dist1, dist2 = detect_show_markers(frame_bgr, \
			frame_grey, self.aruco_dict, self.parameters, self.camera_mtx, self.dist_coefficients)
			distance = middle_point_pose[0][0][2]
			msg = prepare_aruco_dist_msg(distance, dist1, dist2)
			#print('111')
			self.set_aruco_dist(distance, dist1, dist2)
			#self.dist_pub.publish(msg)
			# print("Markers was detected!")
			# if distance < 0.54:
			# 	print('Coordinates of center: ', middle_point_pose)
			# 	print('Distance to left marker: ', dist1)
			# 	print('Distance to right marker: ', dist2)
			# 	print('Orientation center: ', middle_point_orient)
		# If markers was not detected:
		except Exception as e:
			pass

		# frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
		# cv2.imshow("camera "+self.name, frame_rgb)
		# cv2.waitKey(1)

	def change_mode(self, mode):
	
		self.mode = mode
		rospy.loginfo("Current charger " + self.name + " changed mode to: " + str(self.mode))


# Rotate the robot towards a point
# Input
# goal: target point
	def turn_to_point(self, goal):
	
		angle_difference = self.get_aruco_error()
		
		while fabs(angle_difference) > const.ARUCO_ANGLE_ERROR:
		
			angle_difference = self.get_aruco_error()
		
			if angle_difference > 0:
		
				self.movement(0, -const.DOCKING_ROTATION_SPEED)
		
			else:
		
				self.movement(0, const.DOCKING_ROTATION_SPEED)
		
		self.stop()


	def get_distance_to_partner(self):
	
		self_pos = self.get_robot_position()
		partner_pos = gc.get_model_position(self.cur_worker)
		dist = self_pos.get_distance_to(partner_pos)
		return dist
		
	def get_angle_difference_with_parther(self):
	
		partner_orient = gc.get_robot_orientation_vector(self.cur_worker)
		self_orient = self.get_robot_orientation_vector()
		angle_difference = self_orient.get_angle_between_vectors(partner_orient)

	def set_charger_data(self, ch_points, to_ch_p_paths, to_base_paths):
	
		self.pre_ch_points = []
		self.rech_robots = []
		self.ch_points = []
		
		for tup in ch_points:
		
			pre_ch_p = tup[0]
			ch_p = tup[1]
			w_name = tup[2]
			self.rech_robots.append(w_name)
			self.pre_ch_points.append(pre_ch_p)
			self.ch_points.append(ch_p)
		
		
		self.to_ch_p_paths = to_ch_p_paths
		self.to_base_paths = to_base_paths

	def get_aruco_error(self):
	
		aruco_error = self.left_dist - self.right_dist
		return aruco_error

	def dock_to_worker(self):
	
		self.set_movespeed(const.DOCKING_SPEED)
	
		while not self.aruco_dist:
		
			pass
			
		error = self.get_aruco_error()
		error_sum = 0

		while True:
		
			if self.aruco_dist <= const.DOCKING_THRESHOLD:
			
				self.stop()
				break
		
			#print('aruco_dist: ' + str(self.aruco_dist))
			old_error = error
			error = self.get_aruco_error()
			u, error_sum = self.calc_control_action(error, old_error, error_sum)
			#print(u)

			self.movement(self.ms, u)
			
		self.stop()

	def docking(self):

		self.set_movespeed(const.DOCKING_SPEED)
		self.turn_to_point(self.dock_point)
		print(self.name + ' started moving to dock point.')
		self.dock_to_worker()
		print('Charger ' + self.name + ' successfully connected to ' + str(self.cur_worker) + '.')

	def get_robot_battery_level(self, name):
	
		bt = self.trackers[name]
		b_level = bt.battery
		rech_level = bt.recharge_battery
		return b_level, rech_level

	def get_battery_level(self):

		b_level, rech_level = self.get_robot_battery_level(self.name)
		return b_level, rech_level
		
	def check_recharge_battery(self):

		b_level, re_b_level = self.get_battery_level()
		
		if re_b_level < const.LOWER_LIMIT_RECHARGE_BATTERY:
			
			return False
			
		else:
		
			return True
		
	def wait_for_worker(self):
	
		self.change_mode("waiting_for_worker")
		print('Charger ' + self.name + ' is waiting for worker ' + str(self.cur_worker))
		
		while self.mode == "waiting_for_worker":
		
			pass
			
		print(self.name + ' started docking!\n')

# Moving the robot to a point with a PID controller
# Input
# goal: target point
	def move_with_PID(self, goal, dist_error):
	
		error = self.get_angle_difference(goal)
		error_sum = 0
		robot_pos = self.get_robot_position()
		old_pos = robot_pos
		
		while robot_pos.get_distance_to(goal) > dist_error and fabs(error) < 90:
		
			robot_pos = self.get_robot_position()
			old_error = error
			error = self.get_angle_difference(goal)
			u, error_sum = self.calc_control_action(error, old_error, error_sum)
			self.movement(self.ms, u)
			self.is_waiting()
			self.is_dodging()
			self.move_energy_cons(robot_pos, old_pos)
			old_pos = robot_pos
			rospy.sleep(self.pid_delay)
		
	def follow_the_route(self, path, ms, dist_error, mode):
	
		self.change_mode(mode)
		self.set_movespeed(ms)

		#gc.visualise_path(path, const.GREEN_VERTICE_PATH, self.name + '_' + str(random.uniform(0, 1)) + '_p_')

		for state in path:

			self.move_with_PID(state, dist_error)
	
		self.stop()
		
	def perform_charger_mission(self):

		ch_path = self.get_to_ch_p_path()
		#gc.visualise_path(ch_path, const.GREEN_VERTICE_PATH, self.name + '_p_')
		self.follow_the_route(ch_path, const.MOVEMENT_SPEED, const.DISTANCE_ERROR, "movement")
		self.set_next_worker()
		self.wait_for_worker()
		self.follow_the_route(self.dock_path, const.PRE_DOCKING_SPEED, const.DOCK_DISTANCE_ERROR, "docking")
		self.docking()
		self.start_charging()
		b_path = self.get_to_base_path()
		self.move_back(3)
		self.is_waiting()
		#print(self.name + ' moving back to base.')
		self.follow_the_route(b_path, const.MOVEMENT_SPEED, const.DISTANCE_ERROR, "movement")
		self.recharge_batteries()
		
	def recharge_batteries(self):
	
		b_level, r_level = self.get_battery_level()
		s_time = rospy.get_time()
		
		while b_level < const.HIGH_LIMIT_BATTERY and r_level < const.HIGH_LIMIT_BATTERY:
		
			b_level, r_level = self.get_battery_level()
			cur_time = rospy.get_time()
			time_diff = cur_time - s_time
			s_time = cur_time
			charge_received = const.CHARGING_SPEED * time_diff
			self.bt.recharge_power_change(charge_received)
			self.bt.power_change(charge_received)
			rospy.sleep(self.pid_delay)

	def move_back(self, sec):
	
		dur = rospy.Duration(sec, 0)
		self.movement(-const.PRE_DOCKING_SPEED, 0)
		rospy.sleep(dur)
		self.stop

	def set_next_worker(self):
	
		w_name = self.rech_robots[0]
		self.rech_robots.pop(0)
		self.cur_worker = w_name

	def get_to_ch_p_path(self):
	
		ch_path = self.to_ch_p_paths[0]
		self.to_ch_p_paths.pop(0)
		return ch_path

	def get_to_base_path(self):
	
		b_path = self.to_base_paths[0]
		self.to_base_paths.pop(0)
		return b_path
				
	def start_charging(self):

		self.change_mode("charging")
		print('Charger ' + self.name + ' started charging ' + self.cur_worker + '.')
		s_ch_time = rospy.get_time()
		rechargeable_bt = self.trackers[self.cur_worker]
		w_charged = False
		
		while not w_charged:
			
			cur_ch_time = rospy.get_time()
			time_diff = cur_ch_time - s_ch_time
			s_ch_time = cur_ch_time
			charge_received = const.CHARGING_SPEED * time_diff
			rechargeable_bt.power_change(charge_received)
			self.bt.recharge_power_change(-charge_received)
			rech_b_level = rechargeable_bt.battery
			rospy.sleep(self.pid_delay)
			
			if rech_b_level >= const.HIGH_LIMIT_BATTERY:
			
				w_charged = True

		self.change_mode("finished_charging")
		print(self.name + ' finished charging worker ' + self.cur_worker)
		self.ch_points.pop(0)
				
	def run(self):
	
		for ch_p in self.pre_ch_points:
		
			self.perform_charger_mission()

		print('Charger ' + self.name + ' has finished!')
		self.change_mode("finished")
		
def prepare_aruco_dist_msg(dist, left_dist, right_dist):

	msg = ArucoDist()
	msg.dist = dist
	msg.left_dist = left_dist
	msg.right_dist = right_dist
	return msg
