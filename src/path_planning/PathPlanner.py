# Module for planning the path of targets on a height map

import Constants as const
from Point import Point, Vector2d
from PIL import Image
from math import sqrt, fabs, sin, cos, pi, asin, acos
import copy
import random
import time
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
import cv2
from scipy.spatial.transform import Rotation
from numpy import array
from matplotlib.path import Path

# A class describing the heightmap cells formed by its vertices.
# Objects of this class are used when planning collision-free trajectories

# cell_id: cell key, which is identical to the identification number of its upper left vertex
# corners: the identification numbers of the vertices located on the cell boundaries
# pos: cell center position
# obstacle: a boolean variable that determines whether the cell is an obstacle or not
class Cell:
	
	def __init__(self, cell_id, planes):
		self.id = cell_id
		self.planes = planes
		self.make_edges()
		
	def make_edges(self):
		self.edges = []
		names = []
		
		for plane in self.planes:
		
			for edge in plane.edges:
				
				
				e1 = edge[0]
				e2 = edge[1]
				#print(e1.name, e2.name)
				
				name1 = [e1.name, e2.name]
				name2 = [e2.name, e1.name]
				
				if not (name1 in names) and not (name2 in names):
					#print(True)
					self.edges.append(edge)
					names.append(name1)
					
		#print(str(self.id) + ' cell edges count: ' + str(len(self.edges)))
		
		
	def find_z_on_cell(self, x, y):
		
		z = None
		
		for plane in self.planes:
			
			if plane.poly.contains_point((x, y)):
				#print('Plane ' + str(plane.id) + ' contains point (' + str(x) + ', ' + str(y) + ')')
				z = plane.find_z(x, y)
				break
					
		if not z and not (isinstance(z, float) or isinstance(z, int)):
			
			for edge in self.edges:
			
				p1 = edge[0]
				p2 = edge[1]
				new_z = p1.find_z_coord(p2, x, y)
				new_p = Point(x, y, new_z)
				dist1_2 = p1.get_distance_to(p2)
				dist1_3 = p1.get_distance_to(new_p)
				dist2_3 = p2.get_distance_to(new_p)
				dist_sum = round(dist1_2 - dist1_3 - dist2_3, 3)
				#print('p1: ' + str(p1.name) + ' | p2: ' + str(p2.name) + ' | dist_sum: ' + str(dist_sum) + ' | z: ' + str(new_z))
				if dist_sum == 0:
					#print('>>> Success <<<')
					z = new_z
					break
		
		if not z:
		
			print('\np_id: ' + str(p_id))
			new_p = Point(x, y, new_z)
			gc.spawn_sdf_model(new_p, gc_const.VERTICE_PATH, 'p' + str(x) + str(y))
			print('z: ' + str(z) + '\n')
		
		return z			

class Plane:
	def __init__(self, v_id, num, points):
		self.id = str(v_id[0]) + '-' + str(v_id[1]) + '-' + str(num)
		self.v_id = v_id
		self.points = points
		self.poly = self.make_poly()
		self.edges = self.make_edges()
		sum_x = 0
		sum_y = 0
		for p in self.points:
			sum_x += p.x
			sum_y += p.y
		avg_x = sum_x / len(self.points)
		avg_y = sum_y / len(self.points)
		self.a, self.b, self.c, self.d = points[0].get_plane_equation_coeffs(points[1].x, points[1].y, points[1].z, points[2].x, points[2].y, points[2].z)
		z = self.find_z(avg_x, avg_y)
		self.center = Point(avg_x, avg_y, z)
		#gc.spawn_sdf_model(self.center, gc_const.GREEN_VERTICE_PATH, 'center_' + self.id)
		
	def find_z(self, x, y):
		z = float((-self.a * x - self.b * y - self.d) / self.c)
		return z
	
	def visualise_poly(self):
		gc.visualise_path(self.points, gc_const.RED_VERTICE_PATH, 'poly_p_' + self.id)
		
	def make_poly(self):
		poly_points = []
		for p in self.points:
			poly_p = (p.x, p.y)
			poly_points.append(poly_p)
		return Path(array(poly_points))		

	def make_edges(self):
		
		edges = []
		
		for i in range(len(self.points) - 1):
			
			p1 = self.points[i]
			p2 = self.points[i + 1]
			edge = (p1, p2)
			edges.append(edge)
			
		edge = (self.points[len(self.points) - 1], self.points[0])
		edges.append(edge)
		return edges
		
		
# A class that implements the functions needed to plan the path of the target

# map: a dictionary containing all vertices of the heightmap
# obstacles: list of keys of impassable vertices
# height: heightmap height (at vertices)
# width: heightmap width (at vertices)
# grid_range: the size of the range of cells for which the riskiness parameter is calculated
# x_step: distance between adjacent vertices of the height map in x
# y_step: distance between adjacent vertices of the height map in y
# start_id: starting vertex key from map dictionary
# open: list of keys of vertices available for visiting
# closed: list of keys of visited vertices
# closed_goals: list of keys of occupied goal vertices

class PathPlanner:
	def __init__(self, heightmap, l_scale, w_scale, grid_range, x_step, y_step, step_count):
		self.heightmap = heightmap
		self.obstacles = []
		self.min_col = const.MIN_COL
		self.max_col = const.MAX_COL
		self.min_row = const.MIN_ROW
		self.max_row = const.MAX_ROW
		self.l_scale = l_scale
		self.w_scale = w_scale
		self.grid_range = int(grid_range)
		self.x_step = x_step
		self.y_step = y_step
		self.step_count = step_count
		self.start_id = None
		self.open = []
		self.closed = []
		self.closed_goals = []
		self.closed_start_points = []
		self.calc_xy_bounds()
		print('grid_range: ' + str(self.grid_range))

# Deleting vertices lying outside the boundaries of the height map and clearing them from lists of neighboring vertices
	def false_neighbors_deleting(self):
		tmp_map = copy.copy(self.heightmap)

		for key in tmp_map.keys():

			if self.heightmap.get(key):

				v = self.heightmap[key]

				for n_key in v.neighbors_list.keys():

					n_id = v.neighbors_list[n_key]
					num1 = float(n_id[0])
					num2 = float(n_id[1])

					if num1 < self.min_col or num2 < self.min_row or num1 >= self.max_col or num2 >= self.max_row:

						v.neighbors_list.pop(n_key)

# Marking the vertices lying on the border of the height map as obstacles
	def boundary_cells_marking(self):

		for i in range(self.min_col, self.max_col + 1):
		
			for j in range(self.min_row, self.max_row + 1):

				id_1 = (str(float(i)), str(float(self.min_row)))
				id_2 = (str(float(i)), str(float(self.max_row - 1)))
				id_3 = (str(float(self.min_col)), str(float(j)))
				id_4 = (str(float(self.max_col - 1)), str(float(j)))
				self.mark_as_obstacle(id_1)
				self.mark_as_obstacle(id_2)
				self.mark_as_obstacle(id_3)
				self.mark_as_obstacle(id_4)

# Analysis of the height map and search for impassable areas on it
	def detect_obstacles(self):
		tmp_map = copy.copy(self.heightmap)

		for key in tmp_map.keys():

			if self.heightmap.get(key):
				
				#print('key:' + str(key))
				vertice = self.heightmap[key]

				if not vertice.obstacle:

					max_height_diff = self.calc_max_height_difference(key)
					local_roughness = self.calc_local_roughness(key)

					if max_height_diff > const.HIGH_BOUND_HEIGHT_DIFF or local_roughness > const.MAX_ROUGHNESS:

						self.mark_as_obstacle(key)

					else:

						vertice.set_max_height_diff(max_height_diff)
						vertice.set_local_roughness(local_roughness)

		print('Obstacle-vertices count: ' + str(len(self.obstacles)))

		#for vertice_id in self.obstacles:

			#self.calc_local_riskiness(vertice_id)

# Marking a vertex as an obstacle
# Input
# key: key of the vertice
	def mark_as_obstacle(self, key):
		vertice = self.heightmap[key]
		vertice.obstacle = True

		if not key in self.obstacles:

			self.obstacles.append(key)

# Calculating the weight of an edge between vertices
# Input
# v1_id, v2_id: keys of the vertices

# Output
# edge_cost: weight of the edge between vertices with keys v1_id and v2_id
	def calc_edge_cost(self, v1_id, v2_id):
		v1 = self.heightmap[v1_id]
		v2 = self.heightmap[v2_id]
		edge_cost = (const.ROUGHNESS_COEF * max(v1.local_roughness, v2.local_roughness) + 1) * \
(const.HD_COEF * max(v1.max_height_diff, v2.max_height_diff) + 1) * \
v1.get_distance_to(v2) #+ fabs(v1.riskiness - v2.riskiness)
		return edge_cost

# Calculating the weights of all edges of the heightmap
	def calc_all_edge_costs(self):

		for key in self.heightmap.keys():

			vertice = self.heightmap[key]

			for neighbor_id in vertice.neighbors_list.values():

				if not vertice.edges.get(neighbor_id):

					neighbor = self.heightmap[neighbor_id]
					edge_cost = self.calc_edge_cost(key, neighbor_id)
					vertice.set_edge_cost(neighbor, edge_cost)

# Calculation of the risk parameter for vertices located at a short distance from the obstacle
# Input
# vertice_id: obstacle vertex number
	def calc_local_riskiness(self, vertice_id):
		ids = self.calc_grid_range(vertice_id, self.grid_range)
		vertice = self.heightmap[vertice_id]

		for v_id in ids:

			v = self.heightmap[v_id]

			if not v.obstacle:

				dist = vertice.get_distance_to(v)

				if dist < const.INNER_RADIUS:

					v.obstacle = True

				if dist > const.INNER_RADIUS and dist < const.OUTER_RADIUS:

					riskiness = self.calc_riskiness(v_id)
					v.set_riskiness(riskiness)

# Finding a list of keys for vertices that are in the desired range
# Input
# vertice_id: starting vertex number

# ids: list of vertex keys lying in the desired range
	def calc_grid_range(self, vertice_id, grid_range):
		col = int(vertice_id[0])
		row = int(vertice_id[1])
		ids = []
		min_col = col - grid_range
		max_col = col + grid_range + 1
		min_row = row - grid_range
		max_row = row + grid_range + 1

		if min_col < self.min_col:

			min_col = self.min_col

		if max_col > self.max_col:

			max_col = self.max_col

		if min_row < self.min_row:

			min_row = self.min_row

		if max_row > self.max_row:

			max_row = self.max_row

		for i in range(min_col, max_col):

			for j in range(min_row, max_row):

				v_id = (str(i), str(j))
				ids.append(v_id)

		ids.remove(vertice_id)
		return ids

	def calc_xy_bounds(self):
		x1 = float(-self.l_scale / 2 + self.min_row * self.x_step) 
		y1 = float(self.w_scale / 2 - self.min_col * self.y_step)
		x2 = float(-self.l_scale / 2 + self.max_row * self.x_step) 
		y2 = float(self.w_scale / 2 - self.max_col * self.y_step)
		self.min_x = min(x1, x2)
		self.min_y = min(y1, y2)
		self.max_x = max(x1, x2)
		self.max_y = max(y1, y2)

# Calculation of the riskiness parameter for a specific vertex
# Input
# vertice_id: id of this vertex

# Output
# riskiness: riskiness parameter value
	def calc_riskiness(self, vertice_id):
		d_lethal, c_max = self.calc_dist_to_obstacle(vertice_id)
		riskiness = c_max * (cos((d_lethal - const.INNER_RADIUS) * pi / (const.OUTER_RADIUS - const.INNER_RADIUS)) + 1)
		return riskiness

# Calculating the distance to the nearest obstacle for a specific vertex
# Input
# vertice_id: id of this vertex

# Output
# min_dist: distance to nearest obstacle
# obst_cost: weight of this obstacle
	def calc_dist_to_obstacle(self, vertice_id):
		min_dist = float('inf')
		ids = self.calc_grid_range(vertice_id, self.grid_range)
		vertice = self.heightmap[vertice_id]

		for v_id in ids:

			v = self.heightmap[v_id]

			if v.obstacle:

				dist = vertice.get_distance_to(v)
	
				if dist < min_dist:

					min_dist = dist
					obst_cost = self.calc_obst_cost(v_id)

		return min_dist, obst_cost

# Calculating the weight of an obstacle
# Input
# vertice_id: obstacle vertex id

# Output
# obst_cost: weight of this obstacle
	def calc_obst_cost(self, vertice_id):
		max_height_diff = self.calc_max_height_difference(vertice_id)
		local_roughness = self.calc_local_roughness(vertice_id)
		obst_cost = const.ROUGHNESS_COEF * local_roughness * const.HD_COEF * max_height_diff
		return obst_cost

# Calculation of the local roughness parameter for a vertex
# Input
# vertice_id: vertex key

# Output
# roughness: local roughness parameter value
	def calc_local_roughness(self, vertice_id):
		vertice = self.heightmap[vertice_id]
		ln_count = len(vertice.neighbors_list)
		sum_angles = 0
		#print('\nvertice_id: ' + str(vertice_id))
		#print(vertice.neighbors_list.values())
		for v_id in vertice.neighbors_list.values():

			v = self.heightmap[v_id]
			#print(v_id)
			surf_angle = self.calc_heightmap_surf_angle(vertice_id, v_id)
			sum_angles += fabs(surf_angle)

		roughness = fabs(sum_angles / ln_count)
		return roughness

	def calc_heightmap_surf_angle(self, v1_id, v2_id):
		v1 = self.heightmap[v1_id]
		v2 = self.heightmap[v2_id]
		surf_angle = self.calc_surf_angle(v1, v2)
		return surf_angle

# Calculation of the slope of the surface between two vertices
# Input
# v1, v2: coordinates of the vertices

# Output
# surf_angle: angle of inclination of the surface between the vertices
	def calc_surf_angle(self, v1, v2):

		z1 = v1.z
		z2 = v2.z
		cathet = z1 - z2
		hypotenuse = v1.get_distance_to(v2)
		sin_a = cathet / hypotenuse
		surf_angle = asin(sin_a) * 180 / pi
		return surf_angle

# Calculation of the largest height difference between a vertex and one of its neighboring vertices
# Input
# vertice_id: vertex key

# Output
# max_height_diff: the value of the maximum height difference between the vertices (in meters)
	def calc_max_height_difference(self, vertice_id):
		vertice = self.heightmap[vertice_id]
		z = vertice.z
		max_height_diff = 0
		#print('vertice_id: ' + str(vertice_id) + '\n')
		for v_id in vertice.neighbors_list.values():

			#print(v_id)
			v = self.heightmap[v_id]
			z1 = v.z
			height_diff = fabs(z1 - z)

			if height_diff > max_height_diff:

				max_height_diff = height_diff

		return max_height_diff

# Clearing path cost values for each vertex and clearing open and closed arrays
	def clear_path_costs(self):

		for key in self.heightmap.keys():

			v = self.heightmap[key]
			v.path_cost = None
			v.set_predecessor(None)
			v.edges = {}

		self.open = []
		self.closed = []

# Clearing the list of neighbors of a vertex
# Input
# v_id: vertex key
	def clear_neighbors_list(self, v_id):
		vertice = self.heightmap[v_id]
		neighbor_ids = vertice.neighbors_list.values()

		for neighbor_id in neighbor_ids:

			if self.heightmap.get(neighbor_id):

				neighbor = self.heightmap[neighbor_id]
				vertice.delete_neighbor(neighbor)

			else:

				vertice.delete_false_neighbor(neighbor_id)

# Selecting the vertex with the lowest total path cost from the list of unvisited vertices
# Input
# goal: goal vertice position

# Output
# closest_id: best vertex key
	def best_vertice_choice(self, goal):
		min_path_cost = float('inf')
		closest_id = None

		for v_id in self.open:

			v = self.heightmap[v_id]

			if not v.obstacle:

				dist = v.get_distance_to(goal)
				total_path_cost = v.path_cost + dist

				if total_path_cost < min_path_cost:

					min_path_cost = total_path_cost
					closest_id = v_id

		if closest_id and closest_id in self.open:

			self.open.remove(closest_id)

		return closest_id

# Creating Plane class objects and marking impassable cells
	def cells_maker(self):
		self.cells = {}

		for i in range(self.min_col, self.max_col):
		
			for j in range(self.min_row, self.max_row):
				
				col = float(i)
				row = float(j)
				
				id1 = (str(col), str(row))
				
				#print('id: ' + str(id1))
				
				id2 = (str(col), str(row + 1))
				id3 = (str(col + 1), str(row))
				id4 = (str(col + 1), str(row + 1))

				p1 = self.heightmap[id1]
				p2 = self.heightmap[id2]
				p3 = self.heightmap[id3]
				p4 = self.heightmap[id4]
				
				p1.name = 'p1'
				p2.name = 'p2'
				p3.name = 'p3'
				p4.name = 'p4'

				init_x = p1.x
				init_y = p1.y
				end_x = p4.x
				end_y = p4.y
				
				avg_x = (p1.x + p2.x + p3.x + p4.x) / 4
				avg_y = (p1.y + p2.y + p3.y + p4.y) / 4
				
				
				dist1_2 = p1.get_distance_to(p2)
				dist1_3 = p1.get_distance_to(p3)
				dist2_4 = p2.get_distance_to(p4)
				dist3_4 = p3.get_distance_to(p4)
				
				mid_col = str(i) + '.' + str(self.step_count / 2)
				mid_row = str(j) + '.' + str(self.step_count / 2)
				p_mid_id = (mid_col, mid_row)
				
				
				p1_2 = p1.get_point_at_distance_and_angle(p2, dist1_2 / 2)
				p1_3 = p1.get_point_at_distance_and_angle(p3, dist1_3 / 2)
				p2_4 = p2.get_point_at_distance_and_angle(p4, dist2_4 / 2)
				p3_4 = p3.get_point_at_distance_and_angle(p4, dist3_4 / 2)
				
				
				p1_2_name = 'p1_2'
				p1_3_name = 'p1_3'
				p2_4_name = 'p2_4'
				p3_4_name = 'p3_4'
				
				p1_2.name = p1_2_name
				p1_3.name = p1_3_name
				p2_4.name = p2_4_name
				p3_4.name = p3_4_name
				
				if i % 2 == 0:
					
					#print('>>> i % 2 == 0 <<<')
					mid_z = find_z_on_plane(avg_x, avg_y, p3, p3_4, p1)
					p_mid = Point(avg_x, avg_y, mid_z)
					p_mid.set_id(p_mid_id)
					p_mid.name = 'p_mid'
					self.heightmap[p_mid_id] = p_mid
					
					plane1 = Plane(id1, 1, [p3, p1, p_mid, p3_4])
					plane2 = Plane(id1, 2, [p2, p2_4, p_mid, p1])
					plane3 = Plane(id1, 3, [p_mid, p2_4, p3_4])
					plane4 = Plane(id1, 4, [p4, p3_4, p2_4])
					
					planes = [plane1, plane2, plane3, plane4]
					
					cell = Cell(id1, planes)
					
					self.cells[id1] = cell
				
				else:
				
					#print('>>> i % 2 == 1 <<<')
					
					mid_z = find_z_on_plane(avg_x, avg_y, p1, p1_2, p3)
					p_mid = Point(avg_x, avg_y, mid_z)
					p_mid.set_id(p_mid_id)
					p_mid.name = 'p_mid'
					self.heightmap[p_mid_id] = p_mid
					
					plane1 = Plane(id1, 1, [p1, p1_2, p_mid, p3])
					plane2 = Plane(id1, 2, [p4, p3, p_mid, p2_4])
					plane3 = Plane(id1, 3, [p_mid, p1_2, p2_4])
					plane4 = Plane(id1, 4, [p2, p2_4, p1_2])
					planes = [plane1, plane2, plane3, plane4]
					
					cell = Cell(id1, planes)
					
					self.cells[id1] = cell
					
				for l in range(0, self.step_count):
						
					for k in range(0, self.step_count):
						
						new_x = init_x + k * const.GRID_SIZE
						new_y = init_y - l * const.GRID_SIZE
						new_p_id = (str(i) + '.' + str(l), str(j) + '.' + str(k))
						
						if new_p_id in self.heightmap.keys() or (l == 0 and k == 0):
						
							continue
						
						else:
						
							new_z = cell.find_z_on_cell(new_x, new_y)
							

						new_p = Point(new_x, new_y, new_z)
						
						new_p.set_id(new_p_id)
						new_p.set_z(new_z)
						self.heightmap[new_p_id] = new_p
						

	def make_heightmap_neighbors(self):
		
		for key in self.heightmap.keys():
			#print(key)
			v = self.heightmap[key]
			v.set_neighbors_list(self.step_count)

# Finding the height value of an adjacent point on a height map
# Input
# x: point x-coordinate value 
# y: point y-coordinate value

# Output
# z: calculated point z-coordinate value

	def find_z(self, x, y):
		p = Point(x, y, 0)
		cell_id = self.get_current_cell_id(p)
		cell = self.cells[cell_id]
		z = cell.find_z_on_cell(x, y)
		return z


# Finding the closest vertex of the heightmap to a given position
# Choosing a random target vertex
# Input
# pos: initial robot position
# orient: initial robot orieantation

# Output
# start_id: starting vertex key
# goal_id: target vertex key
	def get_start_and_goal_id(self, pos, orient):
		start_id = self.get_start_vertice_id(pos, orient)
		if start_id:
			goal_id = self.get_random_goal_id(start_id, orient)
		else:
			print('Path planning from the point ' + str(pos) + ' is impossible.')
			return None, None
		return start_id, goal_id

		

# Path planning with an LRLHD* algorithm
# Input:
# start_id: starting vertex key
# goal_id: target vertex key

# Output
# path: path vertex list (None if path cannot be built)
	def find_path(self, start_id, goal_id, start_orient):
		start_v = self.heightmap[start_id]
		goal_v = self.heightmap[goal_id]
		current_v = start_v
		current_v.path_cost = 0
		current_neighbors = current_v.neighbors_list.values()
		self.closed.append(current_v.id)
		iter = 0
		current_v.dir_vect = start_orient

		while not current_v.id == goal_id and iter < len(self.heightmap) / 10:

			iter += 1

			for v_id in current_neighbors:

				v = self.heightmap[v_id]
				vect = current_v.get_dir_vector_between_points(v)
				angle_difference = fabs(current_v.dir_vect.get_angle_between_vectors(vect))

				if not v.obstacle and angle_difference < const.ORIENT_BOUND:
					v.dir_vect = vect

					if not v_id in self.open and not v_id in self.closed:

						self.open.append(v_id)

					if not v.edges.get(current_v.id):

						edge_cost = self.calc_edge_cost(current_v.id, v_id)
						v.set_edge_cost(current_v, edge_cost)

					new_path_cost = current_v.path_cost + current_v.edges[v_id]

					if not v.path_cost or new_path_cost < v.path_cost:

						v.path_cost = new_path_cost
						v.set_predecessor(current_v.id)

			current_v_id = self.best_vertice_choice(goal_v)

			if current_v_id == None or len(self.open) == 0:

				break

			current_v = self.heightmap[current_v_id]
			current_neighbors = copy.copy(current_v.neighbors_list.values())
			self.closed.append(current_v_id)

			if current_v_id in self.open:

				self.open.remove(current_v_id)

		if not goal_v.get_predecessor() == None:

			print('Path was found!')
			current_v = goal_v
			path = []
			path_ids = []
			path.insert(0, current_v)
			path_ids.insert(0, current_v.id)

			while not current_v == start_v:

				predecessor_id = current_v.get_predecessor()
				current_v = self.heightmap[predecessor_id]
				path.insert(0, current_v)
				path_ids.insert(0, predecessor_id)

			path_length = get_path_length(path)
			path_cost = goal_v.path_cost
			path_curvature = self.get_path_curvature_ids(path_ids)
			print('Path cost: ' + str(path_cost))
			print('Path length: ' + str(path_length))
			print('Path curvature: ' + str(path_curvature))
			print('Number of vertices = ' + str(len(path)))
			self.clear_path_costs()
			return path, path_ids, path_cost

		else:

			self.clear_path_costs()
			print('Path from ' + str(start_id) + ' to ' + str(goal_id) + ' vertex cannot be found.')
			return None, None, None

	def get_random_start_pos(self):

		while True:

			x = random.uniform(self.min_x, self.max_x)
			y = random.uniform(self.min_y, self.max_y)
			z = self.find_z(x, y)
			p = Point(x, y, z)
			new_p_id = self.get_nearest_vertice_id(p)
			new_p = self.heightmap[new_p_id]

			if not new_p.obstacle and not new_p_id in self.closed_start_points:

				self.closed_start_points.append(new_p_id)
				roll, pitch = self.get_start_orientation(new_p_id)
				rot = Rotation.from_euler('xyz', [roll, pitch, 0], degrees=True)
				quat = rot.as_quat()
				break

		gc.spawn_sdf_model(new_p, gc_const.BLUE_VERTICE_PATH,'v' + str(new_p_id))

		return new_p, quat

	def get_spawn_height(self, p_id):
		p = self.heightmap[p_id]
		neighbors = p.neighbors_list.values()
		max_z = p.z

		for v_id in neighbors:

			v = self.heightmap[v_id]
			if v.z > max_z:

				max_z = v.z

		return max_z

	def get_start_orientation(self, v_id):
		v = self.heightmap[v_id]
		neighbors = v.neighbors_list.values()
		min_col, max_col, min_row, max_row = self.get_min_max_indices(neighbors)
		cols = [min_col, max_col]
		rows = [min_row, max_row]
		points = []
		sum_z = 0

		for col in cols:

			for row in rows:

				close_p_id = (str(col), str(row))
				points.append(close_p_id)

		line_count = 0
		sum_angle = 0

		for i in range(0, len(points) / 2):

			p1_id = points[i]
			p2_id = points[i + 2]
			p1 = self.heightmap[p1_id]
			p2 = self.heightmap[p2_id]
			#gc.spawn_sdf_model(new_p1, gc_const.GREEN_VERTICE_PATH, 'p' + str(new_p1))
			#gc.spawn_sdf_model(new_p2, gc_const.GREEN_VERTICE_PATH, 'p' + str(new_p2))
			angle = self.calc_surf_angle(p1, p2)
			sum_angle += angle
			line_count += 1

		roll = float(sum_angle / line_count)
		#print('roll: ' + str(roll))
		line_count = 0
		sum_angle = 0

		for i in range(0, len(points), 2):

			p1_id = points[i]
			p2_id = points[i + 1]
			p1 = self.heightmap[p1_id]
			p2 = self.heightmap[p2_id]
			angle = self.calc_surf_angle(p1, p2)
			sum_angle += angle
			line_count += 1

		pitch = float(sum_angle / line_count)
		#print('pitch: ' + str(pitch))
		return roll, pitch

	def get_min_max_indices(self, id_list):
		min_col = float('inf')
		max_col = 0
		min_row = float('inf')
		max_row = 0

		for v_id in id_list:

			col = float(v_id[0])
			row = float(v_id[1])

			if col < min_col:

				min_col = col

			elif col > max_col:

				max_col = col

			if row < min_row:

				min_row = row

			elif row > max_row:

				max_row = row

		return min_col, max_col, min_row, max_row


	def get_nearest_vertice_id(self, point):
		x = point.x
		y = point.y
		j = int((x + (self.l_scale / 2)) // self.x_step)
		i = int(((self.w_scale / 2) - y) // self.y_step)
		j_mod = (x + (self.l_scale / 2)) % self.x_step
		i_mod = ((self.w_scale / 2) - y) % self.y_step
		l = i_mod // const.GRID_SIZE
		k = j_mod // const.GRID_SIZE
		
		p_id = (str(i) + '.' + str(int(l)), str(j) + '.' + str(int(k)))
		return p_id
		
	def get_current_cell_id(self, point):
		x = point.x
		y = point.y
		j = int((x + (self.l_scale / 2)) // self.x_step)
		i = int(((self.w_scale / 2) - y) // self.y_step)
		
		cell_id = (str(float(i)), str(float(j)))
		return cell_id
			

	def calc_start_ids_range(self, v_id):
		ids_range = []
		v = self.heightmap[v_id]
		for n_id in v.neighbors_list.values():
			n = self.heightmap[n_id]
			for n_n_id in n.neighbors_list.values():
				if not n_n_id in ids_range:
					ids_range.append(n_n_id)
		ids_range.remove(v_id)
		return ids_range

# Finding the closest vertice of the heightmap to a given point
# Input
# point: point coordinates

# Output
# selected_key: closest vertex key
	def get_start_vertice_id(self, point, robot_vect):
		p_id = self.get_nearest_vertice_id(point)
		p = self.heightmap[p_id]
		min_angle = 360
		points = []
		ids = self.calc_start_ids_range(p_id)
		current_id = None
		for v_id in ids:

			v = self.heightmap[v_id]
			new_vect = point.get_dir_vector_between_points(v)
			angle_difference = fabs(robot_vect.get_angle_between_vectors(new_vect))
			#print(v_id, angle_difference, v.obstacle)
			
			if angle_difference < const.ORIENT_BOUND and angle_difference < min_angle and not v.obstacle:
					#print(' >>> Start_id was found!')
					min_angle = angle_difference
					current_id = v_id

			points.append(v)

		#print('Start vertice angle difference: ' + str(min_angle))
		return current_id

# Finding a random target vertex for path planning
# Input
# start_id: starting vertex key
# start_orient: the vector of the initial orientation of the robot

# Output
# goal_id: target vertex key
	def get_random_goal_id(self, start_id, start_orient):
		goal_id = random.choice(list(self.heightmap.keys()))
		goal_v = self.heightmap[goal_id]
		start_v = self.heightmap[start_id]
		dist = goal_v.get_distance_to(start_v)
		iter_count = 0

		while True:

			goal_id = random.choice(list(self.heightmap.keys()))
			goal_v = self.heightmap[goal_id]
			dist = goal_v.get_distance_to(start_v)

			if not(start_v.obstacle or goal_v.obstacle or goal_id == start_id or dist > const.GOAL_VERTICE_DIST or goal_id in self.closed_goals):

				iter_count += 1
				path, path_ids, path_cost = self.find_path(start_id, goal_id, start_orient)

				if path:

					self.closed_goals.append(goal_id)
					break

			if iter_count > const.MAX_ITER_COUNT:
			
				print(str(start_id) + ' vertice is isolated.')
				goal_id = None
				break

		return goal_id


	def get_goal_id(self, x, y, offset):
		p = Point(x, y, 0)
		p_id = self.get_nearest_vertice_id(p)
		i = int(p_id[0])
		j = int(p_id[1])

		while True:

			l = random.randint(i - offset, i + offset + 1)
			k = random.randint(j - offset, j + offset + 1)
			goal_id = (str(l), str(k))
			goal_v = self.heightmap[goal_id]

			if not (goal_v.obstacle or goal_id in self.closed_goals):

				break

		return goal_id

	def get_start_id(self, x, y, offset):
		p = Point(x, y, 0)
		p_id = self.get_nearest_vertice_id(p)
		i = int(p_id[0])
		j = int(p_id[1])

		while True:

			l = random.randint(i - offset, i + offset + 1)
			k = random.randint(j - offset, j + offset + 1)
			start_id = (str(l), str(k))
			start_v = self.heightmap[start_id]

			if not (start_v.obstacle or start_id in self.closed_start_points):

				break

		return start_id

# Analysis of the heightmap, search for obstacles on it and outlining heightmap boundaries
	def gridmap_preparing(self):
		print('\nLRLHD-A* path planning has begun.')
		
		start_time = time.time()
		print('\n>>> Gridmap preparing <<<\n')
		self.cells_maker()
		finish_time = time.time()
		
		print('Cell generation run time: ' + str(finish_time - start_time))
	
		start_time = time.time()	
		print('\n>>> Creating lists of neighboring vertices <<<\n')
		self.make_heightmap_neighbors()
		finish_time = time.time()
		
		print('Neighbors generation run time: ' + str(finish_time - start_time))
		
		if ('20.0', '20.0') in self.heightmap.keys():
			print(True)
			
		else:
		
			print(False)
		print('\n>>> Boundary cells marking <<<\n')
		self.boundary_cells_marking()
		
		print('\n>>> False neighbors deleting <<<\n')
		self.false_neighbors_deleting()

		print('\n>>> Detecting obstacles <<<\n')
		self.detect_obstacles()

# Calculating path curvature
# Input
# path_ids: array of path vertex ids

# Output
# max_curvature: path curvature
	def get_path_curvature_ids(self, path_ids):
		curvature_sum = 0
		max_curvature = 0
		path = []

		for v_id in path_ids:

			v = self.heightmap[v_id]
			path.append(v)

		for i in range(0, len(path_ids) - 2):

			p1 = self.heightmap[path_ids[i]]
			p2 = self.heightmap[path_ids[i + 1]]
			p3 = self.heightmap[path_ids[i + 2]]
			v1 = p1.get_dir_vector_between_points(p2)
			v2 = p2.get_dir_vector_between_points(p3)
			angle_difference = fabs(v1.get_angle_between_vectors(v2))
			curvature_sum += angle_difference

			if angle_difference > max_curvature:

				max_curvature = angle_difference

		return max_curvature

# Calculating path length
# Input
# path: path vertex array

# Output
# path_len: path length
def get_path_length(path):
	path_len = 0

	for i in range(0, len(path) - 1):

		current_v = path[i]
		next_v = path[i + 1]
		dist = current_v.get_distance_to(next_v)
		path_len += dist

	return path_len

# Calculating path curvature
# Input
# path: path vertex array

# Output
# max_curvature: path curvature
def get_path_curvature(path):
	max_curvature = 0

	for i in range(0, len(path) - 2):

		p1 = path[i]
		p2 = path[i + 1]
		p3 = path[i + 2]
		v1 = p1.get_dir_vector_between_points(p2)
		v2 = p2.get_dir_vector_between_points(p3)
		angle_difference = fabs(v1.get_angle_between_vectors(v2))

		if angle_difference > max_curvature:

			max_curvature = angle_difference

	return max_curvature
	
def find_z_on_plane(x, y, p1, p2, p3):
	z = p1.get_height_on_3d_plane(p2.x, p2.y, p2.z, p3.x, p3.y, p3.z, x, y)
	return z
