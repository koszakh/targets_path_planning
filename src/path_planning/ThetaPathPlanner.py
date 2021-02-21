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
        self.pos = pos
        self.obstacle = False

# A class that implements the functions needed to plan the path of the target

# map: a dictionary containing all vertices of the heightmap
# obstacles: list of keys of impassable vertices
# height: heightmap height (at vertices)
# width: heightmap width (at vertices)
# start_id: starting vertex key from map dictionary
# open: list of keys of vertices available for visiting
# closed: list of keys of visited vertices

class PathPlanner:
    def __init__(self, heightmap, height, width):
        self.map = heightmap
        self.obstacles = []
        self.height = height
        self.width = width
        #print('Map vertices count: ' + str(len(self.map)))
        self.start_id = None
        self.open = []
        self.closed = []

# Deleting vertices lying outside the boundaries of the height map and clearing them from lists of neighboring vertices
    def false_neighbors_deleting(self):
        tmp_map = copy.copy(self.map)
        for key in tmp_map.keys():
            if self.map.get(key):
                v = self.map[key]
                for n_key in v.neighbors_list.keys():
                    n_id = v.neighbors_list[n_key]
                    num1 = int(n_id[0])
                    num2 = int(n_id[1])
                    if num1 < 0 or num2 < 0 or num1 >= self.height or num2 >= self.width:
                        v.neighbors_list.pop(n_key)
                    #if self.map.get(n_id):
                    #    n = self.map[n_id]
                    #    if n.obstacle:
                    #        self.clear_neighbors_list(n_id)

# Marking the vertices lying on the border of the height map as obstacles
    def boundary_cells_marking(self):
        for i in range(self.height - 1):
            id_1 = (str(i), str(0))
            id_2 = (str(i), str(self.width - 1))
            id_3 = (str(0), str(i))
            id_4 = (str(self.height - 1), str(i))
            self.mark_as_obstacle(id_1)
            self.mark_as_obstacle(id_2)
            self.mark_as_obstacle(id_3)
            self.mark_as_obstacle(id_4)

# Analysis of the height map and search for impassable areas on it
    def detect_obstacles(self):
        tmp_map = copy.copy(self.map)
        for key in tmp_map.keys():
            if self.map.get(key):
                vertice = self.map[key]
                if not vertice.obstacle:
                    max_height_diff = self.calc_max_height_difference(key)
                    local_roughness = self.calc_local_roughness(key)
                    if max_height_diff > const.HIGH_BOUND_HEIGHT_DIFF or local_roughness > const.MAX_ROUGHNESS:
                        self.mark_as_obstacle(key)
                    else:
                        vertice.set_max_height_diff(max_height_diff)
                        vertice.set_local_roughness(local_roughness)
        print('Obstacle-vertices count: ' + str(len(self.obstacles)))

# Marking a vertex as an obstacle
# Input
# key: key of the vertice
    def mark_as_obstacle(self, key):
        vertice = self.map[key]
        vertice.obstacle = True
        if not key in self.obstacles:
            self.obstacles.append(key)

# Calculating the weight of an edge between vertices
# Input
# v1_id, v2_id: keys of the vertices

# Output
# edge_cost: weight of the edge between vertices with keys v1_id and v2_id
    def calc_edge_cost(self, v1_id, v2_id):
        v1 = self.map[v1_id]
        v2 = self.map[v2_id]
        edge_cost = (const.ROUGHNESS_COEF * max(v1.local_roughness, v2.local_roughness) + 1) * \
                    (const.HD_COEF * max(v1.max_height_diff, v2.max_height_diff) + 1) * \
                    v1.get_distance_to(v2)
        return edge_cost

# Calculation of the local roughness parameter for a vertex
# Input
# vertice_id: vertex key

# Output
# roughness: local roughness parameter value
    def calc_local_roughness(self, vertice_id):
        vertice = self.map[vertice_id]
        ln_count = len(vertice.neighbors_list)
        sum_angles = 0
        for v_id in vertice.neighbors_list.values():
            v = self.map[v_id]
            surf_angle = self.calc_surf_angle(vertice, v)
            sum_angles += fabs(surf_angle)
        roughness = fabs(sum_angles / ln_count)
        return roughness

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
        vertice = self.map[vertice_id]
        z = vertice.z
        max_height_diff = 0
        for v_id in vertice.neighbors_list.values():
            v = self.map[v_id]
            z1 = v.z
            height_diff = fabs(z1 - z)
            if height_diff > max_height_diff:
                max_height_diff = height_diff
        return max_height_diff

# Clearing path cost values for each vertex and clearing open and closed arrays
    def clear_path_costs(self):
        for key in self.map.keys():
            v = self.map[key]
            v.path_cost = None
            v.set_predecessor(None)
        self.open = []
        self.closed = []

# Clearing the list of neighbors of a vertex
# Input
# v_id: vertex key
    def clear_neighbors_list(self, v_id):
        vertice = self.map[v_id]
        neighbor_ids = vertice.neighbors_list.values()
        for neighbor_id in neighbor_ids:
            if self.map.get(neighbor_id):
                neighbor = self.map[neighbor_id]
                vertice.delete_neighbor(neighbor)
            else:
                vertice.delete_false_neighbor(neighbor_id)
        # del_item = self.map.pop(v_id)

# Selecting the vertex with the lowest total path cost from the list of unvisited vertices
# Input
# goal: goal vertice position

# Output
# closest_id: best vertex key
    def best_vertice_choice(self, goal):
        min_path_cost = float('inf')
        closest_id = None
        for v_id in self.open:
            v = self.map[v_id]
            if not v.obstacle:
                dist = v.get_distance_to(goal)
                total_path_cost = v.path_cost + dist
                if total_path_cost < min_path_cost:
                    min_path_cost = total_path_cost
                    closest_id = v_id
        if closest_id and closest_id in self.open:
            self.open.remove(closest_id)
        return closest_id

# Finding a random target vertex for path planning
# Input
# start_id: starting vertex key

# Output
# goal_id: target vertex key
    def get_random_goal_id(self, start_id):
        goal_id = random.choice(list(self.map.keys()))
        goal_v = self.map[goal_id]
        start_v = self.map[start_id]
        dist = goal_v.get_distance_to(start_v)
        while start_v.obstacle or goal_v.obstacle or goal_id == start_id or dist > 20:
            goal_id = random.choice(list(self.map.keys()))
            goal_v = self.map[goal_id]
            dist = goal_v.get_distance_to(start_v)
        return goal_id

# Creating Cell class objects and marking impassable cells
    def cell_maker(self):
        self.cells = {}
        for i in range(self.height - 2):
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
                    v = self.map[v_id]
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

# Finding the closest vertex of the heightmap to a given position
# Choosing a random target vertex
# Input
# initial_pos: initial robot position

# Output
# start_id: starting vertex key
# goal_id: target vertex key
    def get_start_and_goal_id(self, initial_pos):
        start_id = self.get_nearest_vertice(initial_pos)
        goal_id = self.get_random_goal_id(start_id)
        return start_id, goal_id

# Path planning with an Theta* algorithm
# Input:
# start_id: starting vertex key
# goal_id: target vertex key

# Output
# path: path vertex list (None if path cannot be built)
    def find_theta_path(self, start_id, goal_id):
        self.start_id = start_id
        print('start_id: ' + str(start_id))
        start = self.map[self.start_id]
        start.path_cost = 0
        start.set_predecessor(None)
        self.open.append(self.start_id)
        goal = self.map[goal_id]
        while len(self.open) > 0:
            s_id = self.best_vertice_choice(goal)
            if not s_id:
                print('Open mas is empty!\n')
                break
            s = self.map[s_id]
            if s_id == goal_id:
                print('Path was found!\n')
                break
            self.closed.append(s_id)
            self.update_bounds(s_id)
            for v_id in s.neighbors_list.values():
                v = self.map[v_id]
                #print(v_id, v.obstacle, v.local_roughness, v.max_height_diff)
                if not v_id in self.closed and not v.obstacle:
                    if not v_id in self.open:
                        v.path_cost = float('inf')
                        v.set_predecessor(None)
                    self.update_vertex(s_id, v_id)

        if goal.get_predecessor():
            current_v = goal
            path = []
            path_ids = []
            path.insert(0, current_v)
            path_ids.insert(0, current_v.id)
            while not current_v.id == self.start_id:
                predecessor_id = current_v.get_predecessor()
                current_v = self.map[predecessor_id]
                path.insert(0, current_v)
                path_ids.insert(0, predecessor_id)
            print('Path cost: ' + str(goal.path_cost))
            print('Number of vertices = ' + str(len(path)))
            self.clear_path_costs()
            return path
        else:
            self.clear_path_costs()
            print('Path to this vertex cannot be found.')
            return None


# Updating the value of the path cost parameter for a vertex
# Input
# s_id: current vertex key
# new_s_id: next vertex key
    def update_vertex(self, s_id, new_s_id):
        s = self.map[s_id]
        angle = self.calc_theta_angle(s_id, new_s_id)
        new_s = self.map[new_s_id]
        if not (s_id == self.start_id) and s.lb <= angle <= s.ub:
            parent_s_id = s.get_predecessor()
            parent_s = self.map[parent_s_id]
            edge_cost = self.calc_edge_cost(parent_s_id, new_s_id)
            parent_s.set_edge_cost(new_s, edge_cost)
            if parent_s.path_cost + edge_cost < new_s.path_cost:
                new_s.path_cost = parent_s.path_cost + edge_cost
                new_s.set_predecessor(parent_s_id)
                if new_s_id in self.open:
                    self.open.remove(new_s_id)
                self.open.append(new_s_id)
        else:
            edge_cost = self.calc_edge_cost(s_id, new_s_id)
            s.set_edge_cost(new_s, edge_cost)
            if s.path_cost + edge_cost < new_s.path_cost:
                new_s.path_cost = s.path_cost + edge_cost
                new_s.set_predecessor(s_id)
                if new_s_id in self.open:
                    self.open.remove(new_s_id)
                self.open.append(new_s_id)

# Calculating the angle between vectors (parent_s, s_id) and (parent_s, new_s_id)
# Input
# s_id: current vertex key
# new_s_id: next vertex key

# Output
# angle: angle between vectors (parent_s, s_id) and (parent_s, new_s_id)
    def calc_theta_angle(self, s_id, new_s_id):
        s = self.map[s_id]
        new_s = self.map[new_s_id]
        parent_s_id = s.get_predecessor()
        if parent_s_id:
            parent_s = self.map[parent_s_id]
            v1 = parent_s.get_dir_vector_between_points(s)
            v2 = parent_s.get_dir_vector_between_points(new_s)
            angle = v2.get_angle_between_vectors(v1)
        else:
            angle = 0
        return angle

# Updating the range of angles in which you can search for free vertices during path planning
# Input
# s_id: current vertex key
    def update_bounds(self, s_id):
        s = self.map[s_id]
        s.lb = float('-inf')
        s.ub = float('inf')
        if not (s_id == self.start_id):
            adj_block_cells = self.get_adjacent_blocked_cells(s_id)
            parent_s_id = s.get_predecessor()
            parent_s = self.map[parent_s_id]
            s_edge_cost = s.edges[parent_s_id]
            for c_id in adj_block_cells:
                cell = self.cells[c_id]
                flag_lb = True
                flag_ub = True
                for v_id in cell.corners:
                    if not v_id == s_id and not v_id == parent_s_id:
                        v = self.map[v_id]
                        angle = self.calc_theta_angle(s_id, v_id)
                        v_edge_cost = self.calc_edge_cost(parent_s_id, v_id)
                        parent_s.set_edge_cost(v, v_edge_cost)
                        if not(v_id == s.get_predecessor() or angle < 0 or (angle == 0 and v_edge_cost <= s_edge_cost)):
                            flag_lb = False
                        if not(v_id == s.get_predecessor() or angle > 0 or (angle == 0 and v_edge_cost <= s_edge_cost)):
                            flag_ub = False
                if flag_lb:
                    s.lb = 0
                if flag_ub:
                    s.ub = 0
            for n_id in s.neighbors_list.values():
                n = self.map[n_id]
                if not n_id == parent_s_id and not n.obstacle:
                    n_pred = n.get_predecessor()
                    angle = self.calc_theta_angle(s_id, n_id)
                    if n_id in self.closed and n_pred == parent_s_id and not n_id == self.start_id:
                        if n.lb + angle <= 0:
                            s.lb = max(s.lb, n.lb + angle)
                        if n.ub + angle >= 0:
                            s.ub = min(s.ub, n.ub + angle)
                    n_edge_cost = self.calc_edge_cost(parent_s_id, n_id)
                    parent_s.set_edge_cost(n, n_edge_cost)
                    if n_edge_cost < s_edge_cost and not parent_s_id == n_id and (
                            not n_id in self.closed or not n.get_predecessor() == parent_s_id):
                        if angle < 0:
                            s.lb = max(s.lb, angle)
                        elif angle > 0:
                            s.ub = min(s.ub, angle)

# Getting the list of blocked cells incident to a vertex with a s_id key
# Input
# v_id: vertex key

# Output
# blocked_cells: blocked cell list
    def get_adjacent_blocked_cells(self, v_id):
        col = int(v_id[0])
        row = int(v_id[1])
        id1 = v_id
        id2 = (str(col - 1), str(row))
        id3 = (str(col), str(row - 1))
        id4 = (str(col - 1), str(row - 1))
        cells_ids = [id1, id2, id3, id4]
        blocked_cells = []
        for c_id in cells_ids:
            if c_id in self.cells.keys():
                cell = self.cells[c_id]
                if cell.obstacle:
                    blocked_cells.append(c_id)
        return blocked_cells

# Finding the closest vertice of the heightmap to a given point
# Input
# point: point coordinates

# Output
# selected_key: closest vertex key
    def get_nearest_vertice(self, point):
        min_dist = float('inf')
        selected_key = None
        for key in self.map.keys():
            vertice = self.map[key]
            dist = vertice.get_distance_to(point)
            if dist < min_dist and not vertice.obstacle:
                min_dist = dist
                selected_key = key
        return selected_key

# Analysis of the heightmap, search for obstacles on it and outlining heightmap boundaries
    def gridmap_preparing(self):
        print('\n>>> Boundary cells marking <<<\n')
        self.boundary_cells_marking()
        print('\n>>> False neighbors deleting <<<\n')
        self.false_neighbors_deleting()
        print('\n>>> Detecting obstacles <<<\n')
        self.detect_obstacles()
        print('\n>>> Building cells <<<\n')
        self.cell_maker()
