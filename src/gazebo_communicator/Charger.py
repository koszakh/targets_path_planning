from gazebo_communicator.Robot import Robot
import GazeboConstants as const
import threading as thr
import rospy

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


	def get_distance_to_partner(self):
	
		self_pos = self.get_robot_position()
		partner_pos = gc.get_model_position(self.partner_name)
		dist = self_pos.get_distance_to(partner_pos)
		return dist
		
	def get_angle_difference_with_parther(self):
	
		partner_orient = gc.get_robot_orientation_vector(self.partner_name)
		self_orient = self.get_robot_orientation_vector()
		angle_difference = self_orient.get_angle_between_vectors(partner_orient)

	def set_charger_data(self, ch_points, to_ch_p_paths, to_base_paths):
	
		self.pre_ch_points = []
		self.rech_robots = []
		
		for ch_p in ch_points:
		
			p = ch_p[0]
			w_name = ch_p[1]
			self.rech_robots.append(w_name)
			self.pre_ch_points.append(p)
		
		self.to_ch_p_paths = to_ch_p_paths
		self.to_base_paths = to_base_paths

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
		
	def waiting_fork_worker(self):
	
		self.mode = "waiting_for_worker"
		
		while self.mode == "waiting_for_worker":
		
			pass

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
			self.robot_waiting()
			rospy.sleep(self.pid_delay)
		
	def perform_charger_mission(self):

		ch_path = self.to_ch_p_paths[0]
		self.to_ch_p_paths.pop(0)
		self.follow_the_route(ch_path)
		self.waiting_for_worker()
		self.cur_worker = self.rech_robots[0]
		self.rech_robots.pop(0)
		self.waiting_for_worker()
		rech_pos = gc.get_robot_position(partner)
		rech_vect = gc.get_robot_orientation_vector(partner)
		dock_point = calc_dock_point(rech_pos, rech_vect, const.ROBOT_RADIUS * 2)
		dock_path = self.plan_path_to_dock_point(dock_point, rech_vect)
		self.follow_the_route(dock_path)
		self.rechargeable_robots.pop(0)
		self.set_docking_mode(partner)
		self.docking()
		self.start_charging()
		self.follow_the_route(path[2])
		
	def plan_path_to_dock_point(self, dock_point, end_vect):
	
		robot_pos = self.get_robot_position()
		vect = self.get_robot_orientation_vector()
		
		path = calc_dubins_path(robot_pos, vect, dock_point, end_vect)
		
		return path
		
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
				
	def run(self):
	
		for ch_p in self.pre_ch_points:
		
			self.perform_charger_mission

		self.finished = True
				
def calc_dock_point(pos, orient, offset):

	rev_orient = orient.get_rotated_vector(180)
	p = pos.get_point_in_direction(rev_orient, offset)
	
	return p

def calc_dubins_path(s_pos, s_vect, e_pos, e_vect):

	q0 = (s_pos.x, s_pos.y, s_vect.vector_to_radians())
	q1 = (e_pos.x, e_pos.y, e_vect.vector_to_radians())
	solution = dubins.shortest_path(q0, q1, const.ROBOT_RADIUS)
	configurations, _ = solution.sample_many(const.DISTANCE_ERROR)
	path = convert_tup_to_point3d(configurations)
	return path
