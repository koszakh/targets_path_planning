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

# A class describing the heightmap cells formed by its vertices.
# Objects of this class are used when planning collision-free trajectories

# cell_id: cell key, which is identical to the identification number of its upper left vertex
# corners: the identification numbers of the vertices located on the cell boundaries
# pos: cell center position
# obstacle: a boolean variable that determines whether the cell is an obstacle or not
class Cell:
    def __init__(self, cell_id, pos):
        self.cell_id = cell_id
        col = int(cell_id[0])
        row = int(cell_id[1])
        self.corners = []
        for i in range(2):
            for j in range(2):
                v_id = (str(col + i), str(row + j))
                self.corners.append(v_id)
        self.edges = [(self.corners[0], self.corners[1]), (self.corners[1], self.corners[3]), (self.corners[3], self.corners[2]), (self.corners[2], self.corners[0])]
        self.pos = pos
        self.obstacle = False

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
    def __init__(self, heightmap, length, width, l_scale, w_scale, grid_range, x_step, y_step):
        self.heightmap = heightmap
        self.obstacles = []
        self.length = length
        self.width = width
	self.l_scale = l_scale
	self.w_scale = w_scale
	self.grid_range = int(grid_range)
        self.x_step = x_step
        self.y_step = y_step
        self.start_id = None
        self.open = []
        self.closed = []
        self.closed_goals = []
        self.closed_start_points = []
	print('length: ' + str(self.length))
	print('width: ' + str(self.width))
	print('grid_range: ' + str(self.grid_range))

# Deleting vertices lying outside the boundaries of the height map and clearing them from lists of neighboring vertices
    def false_neighbors_deleting(self):
        tmp_map = copy.copy(self.heightmap)
        for key in tmp_map.keys():
            if self.heightmap.get(key):
                v = self.heightmap[key]
                for n_key in v.neighbors_list.keys():
                    n_id = v.neighbors_list[n_key]
                    num1 = int(n_id[0])
                    num2 = int(n_id[1])
                    if num1 < 0 or num2 < 0 or num1 >= self.length or num2 >= self.width:
                        v.neighbors_list.pop(n_key)

# Marking the vertices lying on the border of the height map as obstacles
    def boundary_cells_marking(self):
        for i in range(self.length - 1):
            id_1 = (str(i), str(0))
            id_2 = (str(i), str(self.width - 1))
            id_3 = (str(0), str(i))
            id_4 = (str(self.length - 1), str(i))
            self.mark_as_obstacle(id_1)
            self.mark_as_obstacle(id_2)
            self.mark_as_obstacle(id_3)
            self.mark_as_obstacle(id_4)

# Analysis of the height map and search for impassable areas on it
    def detect_obstacles(self):
        tmp_map = copy.copy(self.heightmap)
        for key in tmp_map.keys():
            if self.heightmap.get(key):
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
        for vertice_id in self.obstacles:
            self.calc_local_riskiness(vertice_id)

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
	ids = self.calc_grid_range(vertice_id)
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
    def calc_grid_range(self, vertice_id):
        col = int(vertice_id[0])
	row = int(vertice_id[1])
	ids = []
        min_col = col - self.grid_range
        max_col = col + self.grid_range + 1
        min_row = row - self.grid_range
        max_row = row + self.grid_range + 1
        if min_col < 0:
            min_col = 0
        if max_col > self.length:
            max_col = self.length
        if min_row < 0:
            min_row = 0
        if max_row > self.width:
            max_row = self.width
        for i in range(min_col, max_col):
            for j in range(min_row, max_row):
                v_id = (str(i), str(j))
                ids.append(v_id)
        ids.remove(vertice_id)
	return ids

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
	ids = self.calc_grid_range(vertice_id)
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
        for v_id in vertice.neighbors_list.values():
            v = self.heightmap[v_id]
            surf_angle = self.calc_surf_angle(vertice_id, v_id)
            sum_angles += fabs(surf_angle)
        roughness = fabs(sum_angles / ln_count)
        return roughness

# Calculation of the slope of the surface between two vertices
# Input
# v1, v2: coordinates of the vertices

# Output
# surf_angle: angle of inclination of the surface between the vertices
    def calc_surf_angle(self, v1, v2):
        if isinstance(v1, tuple):
            v1_id = v1
	    v1 = self.heightmap[v1_id]
        if isinstance(v2, tuple):
	    v2_id = v2
	    v2 = self.heightmap[v2_id]
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
        for v_id in vertice.neighbors_list.values():
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



# Creating Cell class objects and marking impassable cells
    def cell_maker(self):
        self.cells = {}
        for i in range(self.length - 2):
            for j in range(self.width - 2):
                p1_id = (str(i), str(j))
                p2_id = (str(i), str(j + 1))
                p3_id = (str(i + 1), str(j))
                p4_id = (str(i + 1), str(j + 1))
                p_ids = [p1_id, p2_id, p3_id, p4_id]
                obst_count = 0
                sum_x = 0
                sum_y = 0
                sum_z = 0
                for v_id in p_ids:
                    v = self.heightmap[v_id]
                    sum_x += v.x
                    sum_y += v.y
                    sum_z += v.z
                    if v.obstacle:
                        obst_count += 1
                avg_x = float(sum_x) / 4
                avg_y = float(sum_y) / 4
                avg_z = float(sum_z) / 4
                cell_pos = Point(avg_x, avg_y, avg_z)
                new_cell = Cell(p1_id, cell_pos)
                if obst_count >= 2:
                    new_cell.obstacle = True
                self.cells[p1_id] = new_cell

# Finding the height value of an adjacent point on a height map
# Input
# x: point x-coordinate value 
# y: point y-coordinate value

# Output
# z: calculated point z-coordinate value
    def find_z(self, x, y):
        p = Point(x, y, 0)
        j = (x + (self.l_scale / 2)) // self.x_step
        i = ((self.w_scale / 2) - y) // self.y_step
        cell_id = (str(int(i)), str(int(j)))
        if cell_id in self.cells.keys():
          n_cell = self.cells[cell_id]
          edge = self.find_nearest_edge(p, n_cell)
          p3 = self.heightmap[edge[0]]
          p4 = self.heightmap[edge[1]]
          new_p = n_cell.pos.find_intersection_of_lines(p, p3, p4)
          z = n_cell.pos.find_z_coord(new_p, p.x, p.y)
          return z
        else:
          print(p, cell_id)
          print('Cell id doesnt exist!')
          return None

# Finding the edge closest to a point on a height map
# Input
# p3: initial point
# cell: closest cell to the point

# Output
# current_edge: closest edge id
    def find_nearest_edge(self, p3, cell):
        current_edge = None
        min_dist = float('inf')
        for edge in cell.edges:
            p1 = self.heightmap[edge[0]]
            p2 = self.heightmap[edge[1]]
            k = ((p2.y - p1.y) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.y - p1.y)) / ((p2.y - p1.y) ** 2 + (p2.x - p1.x) ** 2)
	    x4 = p3.x - k * (p2.y - p1.y)
	    y4 = p3.y + k * (p2.x - p1.x)
            z4 = p1.find_z_coord(p2, x4, y4)
            p4 = Point(x4, y4, z4)
            dist = p4.get_2d_distance(p3)
            if dist < min_dist:
                min_dist = dist
                current_edge = edge
        return current_edge

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
        goal_id = self.get_random_goal_id(start_id, orient)
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
            print('Path to this vertex cannot be found.')
            return None, None, None

    def get_random_start_pos(self):
        while True:
            x = random.uniform(-self.l_scale / 2, self.l_scale / 2)
            y = random.uniform(-self.w_scale / 2, self.w_scale / 2)
            z = self.find_z(x, y)
            p = Point(x, y, z)
            i, j = self.get_nearest_vertice_id(p)
            new_p_id = (str(i), str(j))
            new_p = self.heightmap[new_p_id]
            if not new_p.obstacle and not new_p_id in self.closed_start_points:
                self.closed_start_points.append(new_p_id)
		#z = self.get_spawn_height(new_p_id)
                roll, pitch = self.get_start_orientation(new_p_id, p)
		#p.set_z(new_z)
                rot = Rotation.from_euler('xyz', [roll, pitch, 0], degrees=True)
                quat = rot.as_quat()
                print('Quaternion: ' + str(quat))
                break
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

    def get_start_orientation(self, v_id, p):
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
                #close_p = self.heightmap[close_p_id]
		#sum_z += close_p.z
                points.append(close_p_id)
        #avg_z = float(sum_z / len(points)) + const.SPAWN_HEIGHT_OFFSET
        line_count = 0
        sum_angle = 0
        print('\nRolling')
        for i in range(0, len(points) / 2):
	    p1_id = points[i]
	    p2_id = points[i + 2]
            p1 = self.heightmap[p1_id]
	    p2 = self.heightmap[p2_id]
            new_p1 = v.get_point_at_distance_and_angle_2d(p1, const.ROBOT_RADIUS)
            new_p1_z = self.find_z(new_p1.x, new_p1.y)
	    new_p1.set_z(new_p1_z)
	    new_p2 = v.get_point_at_distance_and_angle_2d(p2, const.ROBOT_RADIUS)
            new_p2_z = self.find_z(new_p2.x, new_p2.y)
	    new_p2.set_z(new_p2_z)
	    gc.spawn_sdf_model(new_p1, gc_const.GREEN_VERTICE_PATH, 'p' + str(new_p1))
            gc.spawn_sdf_model(new_p2, gc_const.GREEN_VERTICE_PATH, 'p' + str(new_p2))
            angle = self.calc_surf_angle(new_p1, new_p2)
            print('angle: ' + str(angle))
            sum_angle += angle
            line_count += 1
        print('sum_angle: ' + str(sum_angle))
        roll = float(sum_angle / line_count)
        print('roll: ' + str(roll))
        line_count = 0
        sum_angle = 0
        print('\nPitching')
        for i in range(0, len(points), 2):
	    p1_id = points[i]
	    p2_id = points[i + 1]
            p1 = self.heightmap[p1_id]
	    p2 = self.heightmap[p2_id]
            new_p1 = v.get_point_at_distance_and_angle_2d(p1, const.ROBOT_RADIUS)
            new_p1_z = self.find_z(new_p1.x, new_p1.y)
	    new_p1.set_z(new_p1_z)
	    new_p2 = v.get_point_at_distance_and_angle_2d(p2, const.ROBOT_RADIUS)
            new_p2_z = self.find_z(new_p2.x, new_p2.y)
	    new_p2.set_z(new_p2_z)
            angle = self.calc_surf_angle(new_p1, new_p2)
            print('angle: ' + str(angle))
            sum_angle += angle
            line_count += 1
        print('sum_angle: ' + str(sum_angle))
        pitch = float(sum_angle / line_count)
        print('pitch: ' + str(pitch))

        return roll, pitch

    def get_min_max_indices(self, id_list):
        min_col = float('inf')
        max_col = 0
        min_row = float('inf')
        max_row = 0
        for v_id in id_list:
            col = int(v_id[0])
            row = int(v_id[1])
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
        if j_mod > self.x_step / 2:
            j += 1
        if i_mod > self.y_step / 2:
            i += 1
        p_id = (str(i), str(j))
        return p_id

# Finding the closest vertice of the heightmap to a given point
# Input
# point: point coordinates

# Output
# selected_key: closest vertex key
    def get_start_vertice_id(self, point, robot_vect):
        p_id = self.get_nearest_vertice_id(point)
        i = int(p_id[0])
        j = int(p_id[1])
        min_dist = float('inf')
        min_angle = 360
        points = []
        for l in range(-2, 3):
            for k in range(-2, 3):
                new_id = (str(i + l), str(j + k))
                v = self.heightmap[new_id]
                new_vect = point.get_dir_vector_between_points(v)
                angle_difference = fabs(robot_vect.get_angle_between_vectors(new_vect))
                if angle_difference < const.ORIENT_BOUND and not v.obstacle:
                    dist = point.get_2d_distance(v)
                    if angle_difference < min_angle:
                        min_angle = angle_difference
                        min_dist = dist
                        current_id = new_id
                points.append(v)
        current_p = self.heightmap[current_id]
        print('Min dist: ' + str(min_dist) + ' | Angle difference: ' + str(min_angle))
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
        while 1:
            goal_id = random.choice(list(self.heightmap.keys()))
            goal_v = self.heightmap[goal_id]
            dist = goal_v.get_distance_to(start_v)
            if not(start_v.obstacle or goal_v.obstacle or goal_id == start_id or dist > 5 or goal_id in self.closed_goals):
                iter_count += 1
	        path, path_ids, path_cost = self.find_path(start_id, goal_id, start_orient)
		if path:
                    self.closed_goals.append(goal_id)
		    break
            if iter_count > 100:
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
