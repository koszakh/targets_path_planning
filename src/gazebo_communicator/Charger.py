from gazebo_communicator.Robot import Robot
import gazebo_communicator.GazeboCommunicator as gc
import GazeboConstants as const
import path_planning.Constants as pp_const
import threading as thr
import rospy
from math import fabs
import dubins
import random

class Charger(Robot):

	def __init__(self, name, trackers):

		thr.Thread.__init__(self)
		self.name = name
		self.trackers = trackers
		self.init_topics()
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

	def change_mode(self, mode):
	
		self.mode = mode
		rospy.loginfo("Current charger " + self.name + " changed mode to: " + str(self.mode))


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
		
		for tup in ch_points:
		
			pre_ch_p = tup[0]
			w_name = tup[2]
			self.rech_robots.append(w_name)
			self.pre_ch_points.append(pre_ch_p)
		
		
		self.to_ch_p_paths = to_ch_p_paths
		self.to_base_paths = to_base_paths

	def docking(self):

		self.change_mode("docking")
		self.set_movespeed(const.DOCKING_SPEED)
		dist = self.get_distance_to_partner()
		partner_pos = gc.get_model_position(self.cur_worker)
		self.turn_to_point(partner_pos)
		self.move_with_PID(self.dock_point, const.DOCK_DISTANCE_ERROR)
		self.stop()
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
		#print(self.name + ' error: ' + str(error))
		error_sum = 0
		robot_pos = self.get_robot_position()
		old_pos = robot_pos
		
		while robot_pos.get_distance_to(goal) > dist_error and fabs(error) < 90:
		
			robot_pos = self.get_robot_position()
			u, error, error_sum = self.calc_control_action(goal, error, error_sum)
			self.movement(self.ms, u)
			self.is_waiting()
			self.is_dodging()
			self.move_energy_cons(robot_pos, old_pos)
			old_pos = robot_pos
			rospy.sleep(self.pid_delay)
		
	def follow_the_route(self, path, ms, dist_error):
	
		self.change_mode("movement")
		self.set_movespeed(ms)

		#gc.visualise_path(path, const.GREEN_VERTICE_PATH, self.name + '_' + str(random.uniform(0, 1)) + '_p_')

		for state in path:

			self.move_with_PID(state, dist_error)
	
		self.stop()
		
	def perform_charger_mission(self):

		ch_path = self.get_to_ch_p_path()
		#gc.visualise_path(ch_path, const.GREEN_VERTICE_PATH, self.name + '_p_')
		self.follow_the_route(ch_path, const.MOVEMENT_SPEED, const.DISTANCE_ERROR)
		self.set_next_worker()
		self.wait_for_worker()
		self.follow_the_route(self.dock_path, const.PRE_DOCKING_SPEED, const.DOCK_DISTANCE_ERROR)
		self.docking()
		self.start_charging()
		b_path = self.get_to_base_path()
		self.follow_the_route(b_path, const.MOVEMENT_SPEED, const.DISTANCE_ERROR)
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
		
		while self.mode == "charging":
			
			cur_ch_time = rospy.get_time()
			time_diff = cur_ch_time - s_ch_time
			s_ch_time = cur_ch_time
			charge_received = const.CHARGING_SPEED * time_diff
			rechargeable_bt.power_change(charge_received)
			self.bt.recharge_power_change(-charge_received)
			rech_b_level = rechargeable_bt.battery
			rospy.sleep(self.pid_delay)
			print('self battery: ' + str(self.bt.recharge_battery))
			print('worker battery: ' + str(rechargeable_bt.battery))
			
			if rech_b_level >= const.HIGH_LIMIT_BATTERY:
			
				self.change_mode(None)
				
		print(self.name + ' finished charging worker ' + self.cur_worker)
				
	def run(self):
	
		for ch_p in self.pre_ch_points:
		
			self.perform_charger_mission()

		print('Charger ' + self.name + ' has finished!')
		self.change_mode("finished")
