from gazebo_communicator.Robot import Robot
import threading as thr
import GazeboConstants as const
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

	def perform_worker_mission(self, path):

		self.follow_the_route(path)
		self.perform_the_task()

		
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
			self.check_ch_p_reach()
			rospy.sleep(self.pid_delay)

	def check_ch_p_reach(self):
	
		dist = self.get_robot_position().get_distance_to(self.charge_points[0])
		
		if dist < const.DISTANCE_ERROR:
		
			self.stop()
			self.wait_for_charge()
			self.charge_points.pop(0)

	def wait_for_charge(self):
	
		self.mode = "charge_waiting"

		b_level, re_b_level = self.get_battery_level()
		
		if b_level < const.LOWER_LIMIT_BATTERY:
			
			while b_level < const.HIGH_LIMIT_BATTERY:
			
				b_level, re_b_level = self.get_battery_level()
				
		self.mode = "movement"

	def set_worker_data(self, w_paths, w_points, w_ch_points):

		self.paths = w_paths
		self.workpoints = w_points
		self.charge_points = w_ch_points
		self.tasks_left = len(w_points)

	def perform_the_task(self):
		
		if self.tasks_left > 0:
		
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
			self.tasks_left -= 1

	def run(self):

		for path in self.paths:
			
			self.perform_worker_mission(path)
			
		print('Worker ' + str(self.name) + ' has finished!')
		self.finished = True
