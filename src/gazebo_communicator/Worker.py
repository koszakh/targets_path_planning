from gazebo_communicator.Robot import Robot
import gazebo_communicator.GazeboCommunicator as gc
import threading as thr
import GazeboConstants as const
import path_planning.Constants as pp_const
from math import fabs
import rospy

class Worker(Robot):

	def __init__(self, name, trackers):

		thr.Thread.__init__(self)
		self.name = name
		self.trackers = trackers
		self.init_topics()
		self.pid_delay = rospy.Duration(0, const.PID_NSEC_DELAY)
		self.bt = self.trackers[name]
		self.paths = []
		self.workpoints = None
		self.charge_points = None
		self.finished = False
		self.waiting = False
		self.mode = "stop"
		self.dodging = False
		robot_pos = self.get_robot_position()
		gc.spawn_sdf_model(robot_pos, const.BIG_GREEN_VERTICE_PATH, self.name + '_s')

	def change_mode(self, mode):
	
		self.mode = mode
		rospy.loginfo("Current worker " + self.name + " changed mode to: " + str(self.mode))

	def perform_worker_mission(self, path):

		#gc.visualise_path(path, const.RED_VERTICE_PATH, self.name + '_p_')
		self.follow_the_route(path)
		self.perform_the_task()

		
# Moving the robot to a point with a PID controller
# Input
# goal: target point
	def move_with_PID(self, goal):
	
		error = self.get_angle_difference(goal)
		error_sum = 0
		robot_pos = self.get_robot_position()
		old_pos = robot_pos
		
		while robot_pos.get_distance_to(goal) > const.DISTANCE_ERROR and fabs(error) < 90:
		
			robot_pos = self.get_robot_position()
			u, error, error_sum = self.calc_control_action(goal, error, error_sum)
			self.movement(self.ms, u)
			self.is_waiting()
			self.is_dodging()
			self.check_ch_p_reach()
			self.move_energy_cons(robot_pos, old_pos)
			old_pos = robot_pos
			rospy.sleep(self.pid_delay)

	def check_ch_p_reach(self):
	
		if len(self.charge_points) > 0:

			ch_p = self.charge_points[0]
			self.check_dist_to_ch_p(ch_p)
			
	def check_dist_to_ch_p(self, ch_p):
	
		dist = self.get_robot_position().get_distance_to(ch_p)
		
		if dist < const.DISTANCE_ERROR:
		
			self.stop()
			self.wait_for_charger()
			self.charge_points.pop(0)

	def get_robot_battery_level(self, name):
	
		bt = self.trackers[name]
		b_level = bt.battery
		return b_level

	def get_battery_level(self):

		b_level = self.get_robot_battery_level(self.name)
		return b_level

	def wait_for_charger(self):
	
		self.change_mode("waiting_for_charger")
		print('Worker ' + self.name + ' is waiting for charge.')

		b_level = self.get_battery_level()
		#print(self.name, b_level)
			
		while b_level < const.HIGH_LIMIT_BATTERY:
		
			b_level = self.get_battery_level()
				
		self.change_mode("movement")
		print('Worker ' + self.name + ' is charged.')

	def set_worker_data(self, w_paths, w_points, w_ch_points):

		if w_paths:
		
			self.paths = w_paths[:len(w_paths) - 1]
			self.to_base_path = w_paths[len(w_paths) - 1]
			self.workpoints = w_points
			gc.visualise_path(self.workpoints, const.GREEN_VERTICE_PATH, self.name + '_task')
			self.charge_points = [ch_p[0] for ch_p in w_ch_points]
				
			gc.visualise_path(self.charge_points, const.BLUE_VERTICE_PATH, self.name + '_ch_p_')

			self.tasks_left = len(w_points)
			
		else:
		
			self.workpoints = []

	def perform_the_task(self):
		
		if self.tasks_left > 0:
		
			print('\n>>> ' + self.name + ' started performing task!')
			b_level = self.get_battery_level()
			self.change_mode("task_performing")
			self.workpoints.pop(0)
			if b_level > const.TASK_ENERGY_COST:

				task_completion = 0
				start_time = rospy.get_time()
				last_step_time = start_time
				exec_duration = const.TASK_EXEC_DURATION
				total_consumption = 0
				
				while task_completion < 100:
				
					cur_time = rospy.get_time()
					all_time = cur_time - start_time
					task_completion = float((all_time / exec_duration) * 100)
					step_time = cur_time - last_step_time
					last_step_time = cur_time
					rospy.sleep(self.pid_delay)
				
				self.bt.power_change(-const.TASK_ENERGY_COST)
				self.tasks_left -= 1
				
			else:
			
				print(self.name + ' has insufficient charge to complete the task.')

	def run(self):
	
		if self.workpoints:

			for path in self.paths:
				
				self.perform_worker_mission(path)

			self.follow_the_route(self.to_base_path)
				
		print('Worker ' + str(self.name) + ' has finished!')

		self.change_mode("finished")
