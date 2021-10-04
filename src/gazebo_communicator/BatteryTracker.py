import GazeboCommunicator as gc
import GazeboConstants as const
import random


class BatteryTracker:

	def __init__(self, name):
	
		self.name = name
		self.dir_point = self.name + const.DIR_POINT_SUFFIX
		self.battery = random.uniform(const.LOW_CHARGE_BOUND, const.HIGH_CHARGE_BOUND)
		self.est_energy_cost = 0
		self.last_p_id = None
		self.last_vect = None
		self.tasks_count = 0
		self.path_costs = {}
		
	def power_change(self, charge):

		self.battery += charge

	def add_task_cost(self, value):
	
		self.est_energy_cost += value
		self.tasks_count += 1
		
# Getting the position of the robot in 3D space in the Gazebo environment

# Output
# robot_pose: coordinates of the robot in 3D space
	def get_robot_position(self):
	
		robot_pose = get_model_position(self.name)
		return robot_pose

	def get_robot_orientation_vector(self):

		vect = gc.get_robot_orientation_vector(self.name)
		return vect
		
def get_battery_trackers(names):

	b_trackers = {}
	
	for name in names:
	
		bt = BatteryTracker(name)
		b_trackers[name] = bt
		
	return b_trackers
