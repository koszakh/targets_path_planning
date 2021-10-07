import GazeboCommunicator as gc
import GazeboConstants as const
import random


class BatteryTracker:

	def __init__(self, name):
	
		self.name = name
		self.dir_point = self.name + const.DIR_POINT_SUFFIX
		self.battery = 100#random.uniform(const.LOW_CHARGE_BOUND, const.HIGH_CHARGE_BOUND)
		self.est_energy_cost = 0
		self.last_p_id = None
		self.last_vect = None
		self.tasks_count = 0
		self.path_costs = {}
		
	def power_change(self, charge):

		self.battery += charge

		if self.battery > 100:

			self.battery = 100

		if self.battery < 0:

			self.battery = 0

	def add_task_cost(self, value):
	
		self.est_energy_cost += value
		self.tasks_count += 1
		
def get_battery_trackers(names):

	b_trackers = {}
	
	for name in names:
	
		bt = BatteryTracker(name)
		b_trackers[name] = bt
		
	return b_trackers
