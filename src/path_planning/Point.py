# The module includes classes for working with points in three-dimensional space and vectors

from math import sqrt, acos, fabs, pi

# 2D vector class

# x: vector x-coordinate
# y: vector y-coordinate
class Vector2d:
    def __init__(self, x, y):
        vector_mod = sqrt(x ** 2 + y ** 2)
        self.x = x / vector_mod
        self.y = y / vector_mod

# Convert vector to angle

# Output
# angle: vector value in degrees
    def vector_to_angle(self):
        if not self.y == 0:
            rad_angle = acos(self.x) * fabs(self.y) / self.y
        else:
            rad_angle = acos(self.x)
        angle = radians_to_degrees(rad_angle)
        # angle = get_360_angle(angle)
        return angle

# Calculating the angle between vectors
# Input
# v: second vector

# Output
# angle: angle between vectors in degrees
    def get_angle_between_vectors(self, v):
        v_angle = v.vector_to_angle()
        self_angle = self.vector_to_angle()
        angle = self_angle - v_angle
        if angle > 180:
            angle = -(360 - angle)
        elif angle < -180:
            angle = 360 + angle
        return angle
    
    def __str__(self):
        return str(self.x) + ", " + str(self.y)

# Class of points in three-dimensional space
# x, y, z: point coordinates
# max_height_diff: the value of the parameter for the maximum local height difference
# local_roughness: The value of the parameter for the local roughness
# edges: a dictionary of edges incident to a given vertex
# path_cost: the current value of the path cost from the starting vertex
# predecessor: predecessor vertex key
# obstacle: a flag whose true value means that the path through the given vertex cannot be built
# id: vertex id
class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.max_height_diff = 0
        self.local_roughness = 0
        self.edges = {}
        self.path_cost = None
        self.predecessor = None
        self.obstacle = False
        self.id = None

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
        direction_vector = Vector2d(point.x - self.x, point.y - self.y)
        return direction_vector

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
        if not k_x == 0:
            k_z = self.z * p.x - p.z * self.x
            k2_z = p.z - self.z
            z = (k2_z * x + k_z) / k_x
        elif not k_y == 0:
            k_z = self.z * p.y - p.z * self.y
            k2_z = p.z - self.z
            z = (k2_z * y + k_z) / k_y
        else:
            z = 0
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

# Setting unique point id and filling the list of neighbors
# Input
# value: unique point id value
    def set_id(self, value):
        self.id = value
        v1 = int(value[0])
        v2 = int(value[1])
        self.neighbors_list = {
            '1' : (str(v1), str(v2 - 1)),
            '2' : (str(v1 - 1), str(v2 - 1)),
            '3' : (str(v1 - 1), str(v2)),
            '4' : (str(v1 - 1), str(v2 + 1)),
            '5' : (str(v1), str(v2 + 1)),
            '6' : (str(v1 + 1), str(v2 + 1)),
            '7' : (str(v1 + 1), str(v2)),
            '8': (str(v1 + 1), str(v2 - 1)),
        }

# Establishing the weight of an edge between a given vertex and another vertex
# Input
# vertice: vertex objcects
# value: edge cost value
    def set_edge_cost(self, vertice, value):
        self.edges[vertice.id] = vertice.edges[self.id] = value

# Convert an angle value from radians to degrees
# Input
# radians: angle value in radians

# Output
# angle: angle value in degrees
def radians_to_degrees(radians):
    angle = radians * 180 / pi
    return angle
