import GazeboCommunicator as gc
import GazeboConstants as const
import random


class BatteryTracker:

	def __init__(self, name, role):
	
		self.name = name
		self.dir_point = self.name + const.DIR_POINT_SUFFIX
		self.battery = 100
		
		if role == "charger":
		
			self.recharge_battery = 100
		
		else:
		
			self.recharge_battery = None
			
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

	def recharge_power_change(self, charge):

		if self.role == "charger":
		
			self.recharge_battery += charge

			if self.recharge_battery > 100:

				self.recharge_battery = 100

			if self.recharge_battery < 0:

				self.recharge_battery = 0

	def add_task_cost(self, value):
	
		self.est_energy_cost += value
		self.tasks_count += 1
		
def get_battery_trackers(w_names, c_names):
	
	w_trackers = {name: BatteryTracker(name, "worker") for name in w_names}
	c_trackers = {name: BatteryTracker(name, "charger") for name in c_names}
	b_trackers = dict(w_trackers.items() + c_trackers.items())
	for key in b_trackers:

		print(key, b_trackers[key])

	return b_trackers
