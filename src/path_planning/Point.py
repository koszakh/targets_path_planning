# The module includes classes for working with points in three-dimensional space and vectors

from math import sqrt, acos, fabs, pi, cos, sin

# 2D vector class

# x: vector x-coordinate
# y: vector y-coordinate
class Vector2d:
	def __init__(self, x, y):
		vector_mod = sqrt(x ** 2 + y ** 2)

		if vector_mod == 0:

			self.x = 0
			self.y = 0

		else:
		
			self.x = x / vector_mod
			self.y = y / vector_mod


	def __str__(self):

		return '(' + str(self.x) + ', ' + str(self.y) + ')'

# Convert vector to angle

# Output
# angle: vector value in degrees
	def vector_to_angle(self):

		rad_angle = self.vector_to_radians()
		angle = radians_to_degrees(rad_angle)
		return angle

	def vector_to_radians(self):

		if not self.y == 0:

			rad_angle = acos(self.x) * fabs(self.y) / self.y

		else:

			rad_angle = acos(self.x)

		return rad_angle

# Calculating the angle between vectors
# Input
# v: second vector

# Output
# angle: angle between vectors in degrees
	def get_angle_between_vectors(self, v):
		v_angle = v.vector_to_angle()
		self_angle = self.vector_to_angle()
		angle = v_angle - self_angle
		if angle > 180:
			angle = -(360 - angle)
		elif angle < -180:
			angle = 360 + angle
		#print('angle: ' + str(angle))
		return angle
	
	def __str__(self):
		return str(self.x) + ", " + str(self.y)

	def get_rotated_vector(self, angle):
		self_angle = self.vector_to_angle()
		v_angle = self_angle + angle
		if v_angle > 180:
			v_angle = -(360 - v_angle)
		elif angle < -180:
			v_angle = 360 + v_angle
		vect = angle_to_vector(v_angle)
		return vect

# Class of points in three-dimensional space
# x, y, z: point coordinates
# max_height_diff: the value of the parameter for the maximum local height difference
# local_roughness: The value of the parameter for the local roughness
# edges: a dictionary of edges incident to a given vertex
# path_cost: the current value of the path cost from the starting vertex
# predecessor: predecessor vertex key
# obstacle: a flag whose true value means that the path through the given vertex cannot be built
# id: vertex id
# edge_attach: a boolean that determines if the vertex is on an edge
# init_point: the vertex of the starting path corresponding to the given point
class Point:
	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z
		self.max_height_diff = 0
		self.local_roughness = 0
		self.riskiness = 0
		self.edges = {}
		self.path_cost = None
		self.predecessor = None
		self.obstacle = False
		self.id = None
		self.edge_attach = None
		self.init_point = None
		self.label = None
		self.last_vect = None
		self.trav_dist = 0

	def __str__(self):
		return str(self.x) + " " + str(self.y) + " " + str(self.z)

	def __repr__(self):
		return str(self.x) + " " + str(self.y) + " " + str(self.z)

	def __call__(self, type_of=int):
		return type_of(self.x), type_of(self.y), type_of(self.z)

# Assigning x-coordinate to a point
# Input
# x: x-coordinate value
	def set_x(self, x):
		self.x = x

# Assigning y-coordinate to a point
# Input
# y: y-coordinate value
	def set_y(self, y):
		self.y = y

# Assigning z-coordinate to a point
# Input
# z: z-coordinate value
	def set_z(self, z):
		self.z = z

# Assigning xyz-coordinated to a point
# Input
# x: x-coordinate value
# y: y-coordinate value
# z: z-coordinate value
	def set_xyz(self, p):
		self.x = p.x
		self.y = p.y
		self.z = p.z

# Getting point xy-coordinates

# Output
# (x, y): tuple of xy-coordiantes
	def get_xy(self):
		return (self.x, self.y)

# Getting distance between points
# Input
# point: coordinates of the second point

# Output
# dist: distance between two points
	def get_distance_to(self, point):
		dist = sqrt((point.x - self.x) ** 2 + (point.y - self.y) ** 2 + (point.z - self.z) ** 2)
		return dist

# Getting the distance between points in two-dimensional space
# Input
# point: coordinates of the second point

# Output
# dist: distance between two points
	def get_2d_distance(self, point):
		dist = sqrt((point.x - self.x) ** 2 + (point.y - self.y) ** 2)
		return dist

# Calculating the vector between points
# Input
# point: coordinates of the second point

# Output
# direction_vector: vector value between points
	def get_dir_vector_between_points(self, point):
		#print('self: ' + str(self) + ' | self.id: ' + str(self.id) + ' | point: ' + str(point) + ' | point.id: ' + str(point.id))
		direction_vector = Vector2d(float(point.x - self.x), float(point.y - self.y))
		return direction_vector

# Calculating the vector between points (as an angle)
# Input
# p: coordinates of the second point

# Output
# angle: vector between points expressed as an angle
	def get_angle_between_points(self, p):
		direction_vector = Vector2d(p.x - self.x, p.y - self.y)
		if direction_vector.y != 0:
			angle = acos(direction_vector.x) * 180 / pi * fabs(direction_vector.y) / \
				direction_vector.y
		else:
			angle = acos(direction_vector.x) * 180 / pi
		return angle

# Calculating the vector between points (as an angle between 0 and 360 degrees)
# Input
# p: coordinates of the second point

# Output
# angle: vector between points expressed as an angle
	def get_360_angle(self, p):
		angle = self.get_angle_between_points(p)
		if angle < 0:
			angle = 360 + angle
		return angle

# Getting the key of the predecessor vertex (during path planning)

# Output
# predecessor: predecessor key
	def get_predecessor(self):
		return self.predecessor

# Finding the z-coordinate of a point lying on a straight line (self, p)
# Input
# x: x-coordinate of the point lying on a straight line
# y: y-coordinate of the point lying on a straight line

# Output
# z: z-coordinate of the point lying on a straight line
	def find_z_coord(self, p, x, y):
		k_x = float(p.x - self.x)
		k_y = float(p.y - self.y)
		k2_z = p.z - self.z
		
		if not k_x == 0:
		
			k_z = self.z * p.x - p.z * self.x
			z = float((k2_z * x + k_z) / k_x)
		
		elif not k_y == 0:
		
			k_z = self.z * p.y - p.z * self.y
			z = float((k2_z * y + k_z) / k_y)
		
		else:
		
			z = self.z
		
		return z

# Setting the key of the predecessor vertex
# Input
# vertice_id: predecessor key
	def set_predecessor(self, vertice_id):
		self.predecessor = vertice_id

# Removing a vertex from the list of neighbors
# Input
# vertice: neighboring vertex object
	def delete_neighbor(self, vertice):
		for key in self.neighbors_list.keys():
			neighbor_id = self.neighbors_list[key]
			if neighbor_id == vertice.id:
				self.neighbors_list.pop(key)
		for key in vertice.neighbors_list.keys():
			v_id = vertice.neighbors_list[key]
			if v_id == self.id:
				vertice.neighbors_list.pop(key)

# Removing the erroneous vertex (that doesn't exist) from the list of neighbors
# Input
# vertice_id: erroneous neighboring vertex key
	def delete_false_neighbor(self, vertice_id):
		for key in self.neighbors_list.keys():
			neighbor = self.neighbors_list[key]
			if neighbor == vertice_id:
				self.neighbors_list.pop(key)

# Setting the Local Roughness Parameter Value
# Input
# value: local roughness paramater value 
	def set_local_roughness(self, value):
		self.local_roughness = value

# Setting the Max Height Difference Parameter Value
# Input
# value: max height difference paramater value 
	def set_max_height_diff(self, value):
		self.max_height_diff = value

	def set_riskiness(self, value):
	
		self.riskiness = value

# Setting unique point id and filling the list of neighbors
# Input
# value: unique point id value
	def set_id(self, value):
	
		self.id = value

	def set_neighbors_list(self, step_count):
	
		col = self.id[0]
		s_col = col.split('.')
		col_ind = int(s_col[0])
		det_col = int(s_col[1])
		
		row = self.id[1]
		s_row = row.split('.')
		row_ind = int(s_row[0])
		det_row = int(s_row[1])
		
		if det_col == 0:
			
			min_col = str(col_ind - 1) + '.' + str(step_count - 1)
			max_col = str(col_ind) + '.' + str(1)
		
		elif det_col == step_count - 1:
			
			min_col = str(col_ind) + '.' + str(int(step_count - 2))
			max_col = str(col_ind + 1) + '.0'
			
		else:
		
			min_col = str(col_ind) + '.' + str(int(det_col) - 1)
			max_col = str(col_ind) + '.' + str(int(det_col) + 1)
			
		if det_row == 0:
			
			min_row = str(row_ind - 1) + '.' + str(step_count - 1)
			max_row = str(row_ind) + '.' + str(1)
		
		elif det_row == step_count - 1:
			
			min_row = str(row_ind) + '.' + str(int(step_count - 2))
			max_row = str(row_ind + 1) + '.0'
			
		else:
		
			min_row = str(row_ind) + '.' + str(int(det_row) - 1)
			max_row = str(row_ind) + '.' + str(int(det_row) + 1)
		
		self.neighbors_list = {
			'1' : (str(col), str(min_row)),
			'2' : (str(min_col), str(min_row)),
			'3' : (str(min_col), str(row)),
			'4' : (str(min_col), str(max_row)),
			'5' : (str(col), str(max_row)),
			'6' : (str(max_col), str(max_row)),
			'7' : (str(max_col), str(row)),
			'8': (str(max_col), str(min_row))
		}
		
		#for value in self.neighbors_list.values():
		
			#print(value)
		
		
# Establishing the weight of an edge between a given vertex and another vertex
# Input
# vertice: vertex objcects
# value: edge cost value
	def set_edge_cost(self, vertice, value):
		self.edges[vertice.id] = vertice.edges[self.id] = value

# Finding the point of intersection of straight lines in three-dimensional space
# Input
# self, p2: points forming the first line
# p3, p4: points forming the second line

# Output
# p: intersection point of lines
	def find_intersection_of_lines(self, p2, p3, p4):
		k_a = float(p2.y - self.y)
		k1_a = float(self.y * p2.x - p2.y * self.x)
		k2_a = float(p2.x - self.x)
		k_b = float(p4.y - p3.y)
		k1_b = float(p3.y * p4.x - p4.y * p3.x)
		k2_b = float(p4.x - p3.x)
		if k_a == 0 and k2_b == 0:
			x = p3.x
			y, z = self.get_yz_coords(p2, x)
			p = Point(x, y, z)
		elif k_b == 0 and k2_a == 0:
			x = self.x
			y, z = p3.get_yz_coords(p4, x)
			p = Point(x, y, z)
		elif k_a == 0:
			des_x = p3.reverse_straight_line_equation(p4, self.y)
			y, z = p3.get_yz_coords(p4, des_x)
			p = Point(des_x, y, z)
		elif k2_a == 0:
			des_y = p3.straight_line_equation(p4, self.x)
			x, z = p3.get_xz_coords(p4, des_y)
			p = Point(x, des_y, z)
		elif k_b == 0:
			des_x = self.reverse_straight_line_equation(p2, p3.y)
			y, z = p3.get_yz_coords(p4, des_x)
			p = Point(des_x, y, z)
		elif k2_b == 0:
			des_y = self.straight_line_equation(p2, p3.x)
			x, z = p3.get_xz_coords(p4, des_y)
			p = Point(x, des_y, z)
		else:
			x = (k2_a * k1_b - k2_b * k1_a) / (k2_b * k_a - k2_a * k_b)
			y, z = p3.get_yz_coords(p4, x)
			p = Point(x, y, z)
		return p

# Calculating the y,z-coordinate of a point lying on a straight line in three-dimensional space
# Input
# p: the second point forming a straight line
# z: z-coordinate of the desired point

# Output
# y: y-coordinate of the desired point
# z: z-coordinate of the desired point
	def get_yz_coords(self, p, x):
		k1 = float(p.x - self.x)
		k_y = self.y * p.x - p.y * self.x
		k2_y = p.y - self.y
		y = (k2_y * x + k_y) / k1
		k_z = self.z * p.x - p.z * self.x
		k2_z = p.z - self.z
		z = (k2_z * x + k_z) / k1
		return y, z

# Calculating the x,z-coordinate of a point lying on a straight line in three-dimensional space
# Input
# p: the second point forming a straight line
# y: y-coordinate of the desired point

# Output
# x: x-coordinate of the desired point
# z: z-coordinate of the desired point
	def get_xz_coords(self, p, y):
		k1 = float(p.y - self.y)
		k_x = self.x * p.y - p.x * self.y
		k2_x = p.x - self.x
		x = (k2_x * y + k_x) / k1
		k_z = self.z * p.y - p.z * self.y
		k2_z = p.z - self.z
		z = (k2_z * y + k_z) / k1
		return x, z

# Calculating the y-coordinate of a point lying on a straight line in two-dimensional space
# Input
# p: the second point forming a straight line
# x: x-coordinate of the desired point

# Output
# y: y-coordinate of the desired point
	def straight_line_equation(self, p, x):
		k1 = float(p.x - self.x)
		k_y = self.y * p.x - p.y * self.x
		k2_y = p.y - self.y
		y = (k2_y * x + k_y) / k1
		return y

# Calculating the x-coordinate of a point lying on a straight line in two-dimensional space
# Input
# p: the second point forming a straight line
# y: y-coordinate of the desired point

# Output
# x: x-coordinate of the desired point
	def reverse_straight_line_equation(self, p, y):
		k1 = float(p.y - self.y)
		k_x = self.x * p.y - p.x * self.y
		k2_x = p.x - self.x
		x = (k2_x * y + k_x) / k1
		return x

	def get_point_in_direction(self, vect, distance):

		v_x = vect.x * distance
		v_y = vect.y * distance
		new_p = Point(self.x + v_x, self.y + v_y, self.z)
		return new_p

# Calculating the position of a point lying on a straight line between points at a certain distance
# Input
# p: second point of the straight line
# distance: the distance from the new point to the first point forming a straight line

# Output
# new_p: position of the desired point
	def get_point_at_distance_and_angle(self, p, distance):
		delta_x = p.x - self.x
		delta_y = p.y - self.y
		delta_z = p.z - self.z
		mod_v = sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2)
		v = Point(delta_x / mod_v, delta_y / mod_v, delta_z / mod_v)
		x = self.x + distance * v.x
		y = self.y + distance * v.y
		z = self.z + distance * v.z
		new_p = Point(x, y, z)
		return new_p

# Calculating the position of a point lying on a straight line between points at a certain distance in 2d environment
# Input
# p: second point of the straight line
# distance: the distance from the new point to the first point forming a straight line

# Output
# new_p: position of the desired point
	def get_point_at_distance_and_angle_2d(self, p, distance):
		delta_x = p.x - self.x
		delta_y = p.y - self.y
		mod_v = sqrt(delta_x ** 2 + delta_y ** 2)
		v = Point(delta_x / mod_v, delta_y / mod_v, 0)
		x = self.x + distance * v.x
		y = self.y + distance * v.y
		z = self.z + distance * v.z
		new_p = Point(x, y, self.z)
		return new_p

	def get_height_on_3d_plane(self, x2, y2, z2, x3, y3, z3, new_x, new_y):
	 	a, b, c, d = self.get_plane_equation_coeffs(x2, y2, z2, x3, y3, z3)
	 	z = float((-a * new_x - b * new_y - d) / c)
		return z

	def get_plane_equation_coeffs(self, x2, y2, z2, x3, y3, z3):
		a1 = float(x2 - self.x)
		b1 = float(y2 - self.y)
		c1 = float(z2 - self.z)
		a2 = float(x3 - self.x)
		b2 = float(y3 - self.y)
		c2 = float(z3 - self.z)
		a = b1 * c2 - b2 * c1
		b = a2 * c1 - a1 * c2
		c = a1 * b2 - b1 * a2
		d = (- a * self.x - b * self.y - c * self.z)
		return a, b, c, d


# Convert an angle value from radians to degrees
# Input
# radians: angle value in radians

# Output
# angle: angle value in degrees
def radians_to_degrees(radians):
	angle = radians * 180 / pi
	return angle

def degrees_to_radians(angle):
	radians = angle * pi / 180
	return radians

def angle_to_vector(angle):
	rad_angle = degrees_to_radians(angle)
	vector = Vector2d(cos(rad_angle), sin(rad_angle))
	return vector
