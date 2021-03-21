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

    def calc_all_edge_costs(self):
        for key in self.map.keys():
            vertice = self.map[key]
            for neighbor_id in vertice.neighbors_list.values():
                if not vertice.edges.get(neighbor_id):
                    neighbor = self.map[neighbor_id]
                    edge_cost = self.calc_edge_cost(key, neighbor_id)
                    vertice.set_edge_cost(neighbor, edge_cost)

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
            v.edges = {}
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
        start_id = self.get_nearest_vertice_id(initial_pos)
        goal_id = self.get_random_goal_id(start_id)
        return start_id, goal_id

# Path planning with an LRLHD* algorithm
# Input:
# start_id: starting vertex key
# goal_id: target vertex key

# Output
# path: path vertex list (None if path cannot be built)
    def find_path(self, start_id, goal_id):
        start_v = self.map[start_id]
        goal_v = self.map[goal_id]
        current_v = start_v
        current_v.path_cost = 0
        current_neighbors = current_v.neighbors_list.values()
        self.closed.append(current_v.id)
        iter = 0
        while not current_v.id == goal_id and iter < len(self.map) / 10:
            iter += 1
            for v_id in current_neighbors:
                v = self.map[v_id]
                if not v.obstacle:
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
            current_v = self.map[current_v_id]
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
                current_v = self.map[predecessor_id]
                path.insert(0, current_v)
                path_ids.insert(0, predecessor_id)
            path_length = get_path_length(path)
            path_cost = goal_v.path_cost
            print('Path cost: ' + str(path_cost))
            print('Path length: ' + str(path_length))
            print('Number of vertices = ' + str(len(path)))
            self.clear_path_costs()
            return path, path_ids, path_cost
        else:
            self.clear_path_costs()
            print('Path to this vertex cannot be found.')
            return None, None, None

# Finding the closest vertice of the heightmap to a given point
# Input
# point: point coordinates

# Output
# selected_key: closest vertex key
    def get_nearest_vertice_id(self, point):
        min_dist = float('inf')
        selected_key = None
        for key in self.map.keys():
            vertice = self.map[key]
            dist = vertice.get_distance_to(point)
            if dist < min_dist and not vertice.obstacle:
                min_dist = dist
                selected_key = key
        return selected_key

# Smoothing the trajectory of the robot
# Input
# path_ids: path vertex ids

# Output
# smoothed_path: an array of smoothed path points
    def curve_smoothing(self, path_ids):
        start_time = time.time()
        smoothed_path_ids = []
        tmp_path_ids = copy.copy(path_ids)
        path_curvature = self.get_path_curvature_ids(path_ids)
        print('Initial curvature: ' + str(path_curvature))
        previous_curvature = path_curvature
        goal_id = path_ids[len(path_ids) - 1]
        max_dist_from_path = 10
        iteration = 0
        while path_curvature > const.CURVATURE_THRESHOLD:
            iteration += 1
            new_path = []
            p1 = path_ids[0]
            new_path.append(p1)
            i = 2
            while True:
                p2 = tmp_path_ids[i - 1]
                p3 = tmp_path_ids[i]
                vertices = [p1, p2, p3]
                current_v = self.map[p2]
                if not current_v.edge_attach:
                    sub_curve_ids = self.vertice_smoothing(vertices)
                else:
                    sub_curve_ids = self.edge_smoothing(vertices)
                for v_id in sub_curve_ids:
                    new_path.append(v_id)
                p1 = new_path[len(new_path) - 1]
                if p3 == goal_id:
                    break
                i += 1
            new_path.append(goal_id)
            new_path = self.delete_doubled_vertices(new_path)
            new_path = self.path_loops_deleting(new_path)
            path_curvature = self.get_path_curvature_ids(new_path)
            max_dist_from_path = self.get_max_dist_from_path(path_ids, new_path)
            tmp_path_ids = copy.copy(new_path)
            if path_curvature < previous_curvature:
                previous_curvature = path_curvature
                smoothed_path_ids = copy.copy(new_path)
            print('New path length: ' + str(len(new_path)))
            print('New path curvature: ' + str(path_curvature))
            print('Current smoothed path curvature: ' + str(previous_curvature))
            print('New max dist: ' + str(max_dist_from_path))
            if iteration == const.MAX_ITERATIONS_COUNT:
                print(str(iteration) + ' iterations passed')
                break
        if smoothed_path_ids:
            curve_smoothing_time = time.time() - start_time
            last_path = []
            for v_id in new_path:
                v = self.map[v_id]
                last_path.append(v)
            init_path = []
            for v_id in path_ids:
                v = self.map[v_id]
                init_path.append(v)
            smoothed_path_ids = self.delete_doubled_vertices(smoothed_path_ids)
            smoothed_path_ids = self.path_loops_deleting(smoothed_path_ids)
            max_dist = self.get_max_dist_from_path(path_ids, smoothed_path_ids)
            path_curvature = self.get_path_curvature_ids(smoothed_path_ids)
            print('\nPath length: ' + str(len(smoothed_path_ids)) + '\nPath_curvature: ' + str(path_curvature) + '\nMax_dist: ' + str(max_dist) + '\nNumber of iterations: ' + str(iteration))
            print('Curve smoothing time: ' + str(curve_smoothing_time))
            self.curve_averaging(smoothed_path_ids)
            smoothed_path_ids.pop(0)
            smoothed_path = []
            for v_id in smoothed_path_ids:
                v = self.map[v_id]
                smoothed_path.append(v)
            return smoothed_path
        else:
            print('This path cant be smoothed.')
            return None

# Smoothing a section of the path, the midpoint of which corresponds to the vertice of the heightmap
# Input
# vertice_ids: path segment vertex numbers

# Output
# new_points: new positions of points of this segment of the path
    def vertice_smoothing(self, vertice_ids):
        p1 = self.map[vertice_ids[0]]
        p2 = self.map[vertice_ids[1]]
        p3 = self.map[vertice_ids[2]]
        v1 = p1.get_dir_vector_between_points(p3)
        v2 = p1.get_dir_vector_between_points(p2)
        new_points = []
        neighbors = p2.neighbors_list.values()
        angle = v1.get_angle_between_vectors(v2)
        if angle > 0:
            N = self.get_up_neighbors(p1, p2, p3)
        else:
            N = self.get_down_neighbors(p1, p2, p3)
        last_p = p1
        sorted_N = self.sort_by_dist(p3, N)
        for neighbor_id in sorted_N:
            neighbor = self.map[neighbor_id]
            new_p = last_p.find_intersection_of_lines(p3, p2, neighbor)
            dist = new_p.get_distance_to(p2)
            if dist > const.DIST_FROM_PATH:
                new_p = p2.get_point_at_distance_and_angle(new_p, const.DIST_FROM_PATH)
                dist = new_p.get_distance_to(p2)
            new_p.init_point = p2.id
            id1 = p2.id
            id2 = neighbor.id
            new_p.edge_attach = (id1, id2)
            new_id = self.get_new_edge_id(new_p)
            new_p.set_id(new_id)
            self.map[new_id] = new_p
            new_points.append(new_id)
            last_p = new_p
        return new_points

# Smoothing a segment of a path whose midpoint is on an edge of the graph
# Input
# vertice_ids: path segment vertex numbers

# Output
# new_points: new positions of points of this segment of the path  
    def edge_smoothing(self, vertice_ids):
        p1 = self.map[vertice_ids[0]]
        p2 = self.map[vertice_ids[1]]
        p3 = self.map[vertice_ids[2]]
        new_points = []
        E = p2.edge_attach
        p_e1 = self.map[E[0]]
        p_e2 = self.map[E[1]]
        new_p = p1.find_intersection_of_lines(p3, p_e1, p_e2)
        init_point = self.map[p2.init_point]
        dist = new_p.get_distance_to(init_point)
        if dist > const.DIST_FROM_PATH:
            new_p = init_point.get_point_at_distance_and_angle(new_p, const.DIST_FROM_PATH)
        new_p.init_point = p2.init_point
        new_p.edge_attach = E
        new_id = self.get_new_edge_id(new_p)
        new_p.set_id(new_id)
        self.map[new_id] = new_p
        new_points.append(new_p.id)
        return new_points

# Final averaging of the smoothed trajectory
# Input
# curve_ids: smoothed path point ids
    def curve_averaging(self, curve_ids):
        start_time = time.time()
        init_curvature = self.get_path_curvature_ids(curve_ids)
        init_path = []
        for v_id in curve_ids:
            v = self.map[v_id]
            init_path.append(v)
        path_length = get_path_length(init_path)
        print('Init curvature: ' + str(init_curvature))
        for i in range(5, len(curve_ids) - 1):
            sub_curve = []
            for j in range(i - 5, i + 1):
                p = self.map[curve_ids[j]]
                sub_curve.append(p)
            p3 = sub_curve[2]
            p4 = sub_curve[3]
            e3 = p3.edge_attach
            e4 = p4.edge_attach
            if e3 and e4:
                id3 = p3.id
                id4 = p4.id
                p1_e3 = self.map[e3[0]]
                p2_e3 = self.map[e3[1]]
                p1_e4 = self.map[e4[0]]
                p2_e4 = self.map[e4[1]]
                p3_init_id = p3.init_point
                p3_init = self.map[p3_init_id]
                p4_init_id = p4.init_point
                p4_init = self.map[p4_init_id]
                p3_1 = p3.get_point_at_distance_and_angle(p1_e3, const.CURVE_AVG_STEP)
                p3_2 = p3.get_point_at_distance_and_angle(p1_e3, const.CURVE_AVG_STEP * 2)
                p3_3 = p3.get_point_at_distance_and_angle(p2_e3, const.CURVE_AVG_STEP)
                p3_4 = p3.get_point_at_distance_and_angle(p2_e3, const.CURVE_AVG_STEP * 2)

                p4_1 = p4.get_point_at_distance_and_angle(p1_e4, const.CURVE_AVG_STEP)
                p4_2 = p4.get_point_at_distance_and_angle(p1_e4, const.CURVE_AVG_STEP * 2)
                p4_3 = p4.get_point_at_distance_and_angle(p2_e4, const.CURVE_AVG_STEP)
                p4_4 = p4.get_point_at_distance_and_angle(p2_e4, const.CURVE_AVG_STEP * 2)

                p3_alts = [p3, p3_1, p3_2, p3_3, p3_4]
                p4_alts = [p4, p4_1, p4_2, p4_3, p4_4]
                best_p3 = p3
                best_p4 = p4
                current_curve = [sub_curve[0], sub_curve[1], p3, p4, sub_curve[4], sub_curve[5]]
                min_curvature = get_path_curvature(current_curve)
                for new_p3 in p3_alts:
                    for new_p4 in p4_alts:
                        new_sub_curve = [sub_curve[0], sub_curve[1], new_p3, new_p4, sub_curve[4], sub_curve[5]]
                        path_curvature = get_path_curvature(new_sub_curve)
                        p3_init_dist = p3_init.get_distance_to(new_p3)
                        p4_init_dist = p4_init.get_distance_to(new_p4)
                        if path_curvature < min_curvature and p3_init_dist < const.DIST_FROM_PATH and p4_init_dist < const.DIST_FROM_PATH:
                            min_curvature = path_curvature
                            best_p3 = new_p3
                            best_p4 = new_p4
                self.map[id3].set_xyz(best_p3)
                self.map[id4].set_xyz(best_p4)
        finish_time = time.time()
        print('Curve averaging time: ' + str(finish_time - start_time))
        final_curvature = self.get_path_curvature_ids(curve_ids)
        perc_reduction = 100 - (final_curvature / (init_curvature / 100))
        print('Reducing the curvature of the path in percent: ' + str(perc_reduction))
        print('Final curvature: ' + str(final_curvature))

# Getting a list of neighboring vertices located on "top" of the vertices
# Input
# p1, p2, p3: three consecutive waypoints

# Output
# up_neighbors: list of neighboring vertices        
    def get_up_neighbors(self, p1, p2, p3):
        angle1 = p2.get_angle_between_points(p1)
        angle2 = p2.get_angle_between_points(p3)
        neighbors = p2.neighbors_list.values()
        up_neighbors = []
        closed_neighbors = []
        if angle1 < angle2:
            angle1 = p2.get_360_angle(p1)
            angle2 = p2.get_360_angle(p3)
            for neighbor_id in neighbors:
                neighbor = self.map[neighbor_id]
                current_angle = p2.get_360_angle(neighbor)
                if current_angle < angle1 and current_angle > angle2:
                    up_neighbors.append(neighbor_id)
        else:
            for neighbor_id in neighbors:
                neighbor = self.map[neighbor_id]
                current_angle = p2.get_angle_between_points(neighbor)
                if current_angle < angle1 and current_angle > angle2:
                    up_neighbors.append(neighbor_id)
        return up_neighbors

# List of vertices located below the given vertices 
# Input
# p1, p2, p3: three consecutive waypoints

# Output
# down_neighbors: list of neighboring vertices
    def get_down_neighbors(self, p1, p2, p3):
        angle1 = p2.get_angle_between_points(p1)
        angle2 = p2.get_angle_between_points(p3)
        angle = p2.get_360_angle(p3)
        neighbors = p2.neighbors_list.values()
        down_neighbors = []
        closed_neighbors = []
        if angle2 < angle1:
            angle1 = p2.get_360_angle(p1)
            angle2 = p2.get_360_angle(p3)
            for neighbor_id in neighbors:
                neighbor = self.map[neighbor_id]
                current_angle = p2.get_360_angle(neighbor)
                if current_angle > angle1 and current_angle < angle2:
                    down_neighbors.append(neighbor_id)
        else:
            for neighbor_id in neighbors:
                neighbor = self.map[neighbor_id]
                current_angle = p2.get_angle_between_points(neighbor)
                if current_angle > angle1 and current_angle < angle2:
                    down_neighbors.append(neighbor_id)
        return down_neighbors

# Obtaining an id for a vertex located on an edge of a graph
# Input
# p: vertex on an edge

# Output
# new_id: new vertex number
    def get_new_edge_id(self, p):
        e = p.edge_attach
        edge_id = str(e[0] + e[1])
        i = 0
        new_id = edge_id
        while new_id in self.map.keys():
            i += 1
            new_id = edge_id + str(i)
        return new_id

# Sorting an array of vertices by distance from some vertex
# Input
# p: vertex, the distance to which is calculated
# mas: original vertex array

# Output
# sorted_mas: sorted array
    def sort_by_dist(self, p, mas):
        sorted_mas = []
        while len(mas) > 0:
            max_dist = 0
            for v_id in mas:
                v = self.map[v_id]
                dist = p.get_distance_to(v)
                if dist > max_dist:
                    max_dist = dist
                    current_v_id = v_id
            sorted_mas.append(current_v_id)
            mas.remove(current_v_id)
        return sorted_mas

# Calculating the maximum distance of the found path from the original path
# Input
# path_ids: initial path
# curve_ids: smoothed path

# Output
# max_dist: maximum distance from the initial path
    def get_max_dist_from_path(self, path_ids, curve_ids):
        max_dist = 0
        for path_id in path_ids:
            min_dist = 999
            init_vertice = self.map[path_id]
            for curve_id in curve_ids:
                vertice = self.map[curve_id]
                dist = vertice.get_distance_to(init_vertice)
                if dist < min_dist:
                    min_dist = dist
            if min_dist > max_dist:
                max_dist = min_dist
        return max_dist

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
            v = self.map[v_id]
            path.append(v)
        for i in range(0, len(path_ids) - 2):
            p1 = self.map[path_ids[i]]
            p2 = self.map[path_ids[i + 1]]
            p3 = self.map[path_ids[i + 2]]
            v1 = p1.get_dir_vector_between_points(p2)
            v2 = p2.get_dir_vector_between_points(p3)
            angle_difference = fabs(v1.get_angle_between_vectors(v2))
            curvature_sum += angle_difference
            if angle_difference > max_curvature:
                max_curvature = angle_difference
        return max_curvature

# Deleting vertices creating loops
# Input
# path_ids: initial path vertex ids

# Output
# new_path_ids: final path vertex ids
    def path_loops_deleting(self, path_ids):
        new_path_ids = copy.copy(path_ids)
        i = len(path_ids) - 1
        goal = path_ids[0]
        while True:
            p1 = self.map[path_ids[i - 2]]
            p2 = self.map[path_ids[i - 1]]
            p3 = self.map[path_ids[i]]
            dist_a = p1.get_2d_distance(p2)
            dist_b = p2.get_2d_distance(p3)
            if dist_a == 0 or dist_b == 0:
                new_path_ids.remove(p2.id)
                i -= 2
            else:
                dist1 = p1.get_2d_distance(p3)
                dist2 = p2.get_2d_distance(p3)
                v1 = p1.get_dir_vector_between_points(p2)
                v2 = p2.get_dir_vector_between_points(p3)
                angle_difference = fabs(v1.get_angle_between_vectors(v2))
                if dist1 < dist2 or angle_difference > 90:
                    if p2.id in new_path_ids:
                        new_path_ids.remove(p2.id)
                    i -= 2
                else:
                    i -= 1
            if p1.id == goal:
                break
            if i < 2:
                i = 2
        return new_path_ids

# Deleting invalid path vertices
# Input
# path_ids: initial path vertex ids

# Output
# new_path_ids: final path vertex ids
    def delete_doubled_vertices(self, path_ids):
        new_path_ids = copy.copy(path_ids)
        i = 1
        goal = path_ids[len(path_ids) - 1]
        while True:
            id1 = path_ids[i - 1]
            id2 = path_ids[i]
            p1 = self.map[id1]
            p2 = self.map[id2]
            dist = p1.get_2d_distance(p2)
            if dist < 0.01 or id1 == id2:
                new_path_ids.remove(p2.id)
                i += 2
            else:
                i += 1
            if p2.id == goal:
                break
            if i >= len(path_ids):
                i = len(path_ids) - 1
        return new_path_ids

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
