# The module for calculating the path, taking into account the avoidance of potential collisions between targets

import path_planning.Constants as const
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
from path_planning.Point import Point
import copy
import rvo2

# A class that implements the calculation of non-collisionless trajectories of a group of target objects

# ms: initial robot speed
# sim: group of targets movement simulator instance
# robots: Target Management Object Dictionary in Gazebo
# agents: dictionary of agents used in sim
# final_paths: dictionary of final routes of robots
# init_paths: dictionary of initial routes of robots
class ORCAsolver:
    def __init__(self, heightmap, cells, x_step, y_step):
        self.ms = gc_const.MOVEMENT_SPEED
        self.sim = rvo2.PyRVOSimulator(gc_const.ORCA_TIME_STEP, gc_const.ORCA_NEIGHBOR_DIST, \
gc_const.ORCA_MAX_NEIGHBORS, gc_const.ORCA_TIME_HORIZON, gc_const.ORCA_TIME_HORIZON_OBST, \
gc_const.ORCA_RADIUS, self.ms)
	print('ORCA time step: ' + str(gc_const.ORCA_TIME_STEP))
        self.heightmap = heightmap
        self.cells = cells
        self.x_step = x_step
        self.y_step = y_step
        self.robots = {}
        self.agents = {}
        self.final_paths = {}
        self.init_paths = {}

# Adding an agent to the group movement simulation
# Input
# robot_name: target object name used in Gazebo
# path: list of points of the original path of the target
    def add_agent(self, robot_name, path):
        self.robots[robot_name] = gc.Robot(robot_name)
        robot_pos = self.robots[robot_name].get_robot_position()
        init_dist = robot_pos.get_distance_to(path[0])
        if init_dist < gc_const.DISTANCE_ERROR:
            path.pop(0)
        self.robots[robot_name].set_init_path(path)
        if robot_name == 'p3at1':
            gc.visualise_path(path, gc_const.VERTICE_PATH, 'v' + str(robot_name) + '_')
        elif robot_name == 'p3at2':
            gc.visualise_path(path, gc_const.RED_VERTICE_PATH, 'v' + str(robot_name) + '_')
        elif robot_name == 'p3at3':
            gc.visualise_path(path, gc_const.GREEN_VERTICE_PATH, 'v' + str(robot_name) + '_')
        print('Robot ' + robot_name + ' initial path length: ' + str(len(path)))
        self.robots[robot_name].current_goal = path[0]
        self.init_paths[robot_name] = copy.copy(self.robots[robot_name].init_path)
        self.robots[robot_name].last_goal = robot_pos
        robot_pos_2d = robot_pos.get_xy()
        self.agents[robot_name] = self.sim.addAgent(robot_pos_2d)
        vel_vect = self.calc_vel_vect_2d(robot_name, robot_pos)
        self.sim.setAgentPrefVelocity(self.agents[robot_name], vel_vect)
        self.final_paths[robot_name] = []

# Calculate a 2D velocity vector that takes into account the slope of the surface
# Input
# robot_name: target object name used in Gazebo

# Output
# vel_vect: 2d velocity vector (tuple)
    def calc_vel_vect_3d(self, robot_name, robot_pos):
        robot = self.robots[robot_name]
        current_goal = robot.current_goal
        robot_vect = gc.convert_3d_to_2d_vect(gc.get_orientation_vector(robot_pos, current_goal))
        goal_dist = current_goal.get_distance_to(robot_pos)
        goal_dist_2d = current_goal.get_2d_distance(robot_pos)
        current_ms = float(goal_dist_2d / goal_dist) * self.ms
        vel_vect = (robot_vect.x * current_ms, robot_vect.y * current_ms)
        return vel_vect

    def calc_vel_vect_2d(self, robot_name, robot_pos):
        robot = self.robots[robot_name]
        current_goal = robot.current_goal
        robot_vect = gc.convert_3d_to_2d_vect(gc.get_orientation_vector(robot_pos, current_goal))
        vel_vect = (robot_vect.x * self.ms, robot_vect.y * self.ms)
        return vel_vect


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

    def find_z(self, p):
        x = p.x
        y = p.y
        j = (x + (const.MAP_HEIGHT / 2)) // self.x_step
        i = ((const.MAP_WIDTH / 2) - y) // self.y_step
        cell_id = (str(int(i)), str(int(j)))
        if cell_id:
          n_cell = self.cells[cell_id]
          edge = self.find_nearest_edge(p, n_cell)
          p3 = self.heightmap[edge[0]]
          p4 = self.heightmap[edge[1]]
          new_p = n_cell.pos.find_intersection_of_lines(p, p3, p4)
          z = n_cell.pos.find_z_coord(new_p, p.x, p.y)
          return z
        else:
          return None

# Checking the reachability of the target point by the robot
# Input
# robot_name: target object name used in Gazebo
# robot_pos: current calculated position of the robot
    def goal_achievement_check(self, robot_name, robot_pos):
        robot = self.robots[robot_name]
        current_goal = robot.current_goal
        dist_2d = robot_pos.get_2d_distance(current_goal)
        if dist_2d < gc_const.DISTANCE_ERROR:
            if len(self.init_paths[robot_name]) > 1:
                robot.last_goal = current_goal
                self.init_paths[robot_name].pop(0)
                new_goal = self.init_paths[robot_name][0]
                robot.current_goal = new_goal
                vel_vect = self.calc_vel_vect_3d(robot_name, robot_pos)
                self.sim.setAgentPrefVelocity(self.agents[robot_name], vel_vect)
            else:
                self.sim.setAgentPrefVelocity(self.agents[robot_name], (0, 0))
                robot.finished_planning = True
                print('Robot ' + robot_name + ' reached the target point. Path length: ' + str(len(self.final_paths[robot_name])))
        else:
            vel_vect = self.calc_vel_vect_2d(robot_name, robot_pos)
            self.sim.setAgentPrefVelocity(self.agents[robot_name], vel_vect)

    def goal_achievement_check_2d(self, robot_name, robot_pos):
        robot = self.robots[robot_name]
        current_goal = robot.current_goal
        dist_2d = robot_pos.get_2d_distance(current_goal)
        if dist_2d < gc_const.DISTANCE_ERROR:
            if len(self.init_paths[robot_name]) > 1:
                robot.last_goal = current_goal
                self.init_paths[robot_name].pop(0)
                new_goal = self.init_paths[robot_name][0]
                robot.current_goal = new_goal
                vel_vect = self.calc_vel_vect_2d(robot_name, robot_pos)
                self.sim.setAgentPrefVelocity(self.agents[robot_name], vel_vect)
            else:
                self.sim.setAgentPrefVelocity(self.agents[robot_name], (0, 0))
                robot.finished_planning = True
                print('Robot ' + robot_name + ' reached the target point. Path length: ' + str(len(self.final_paths[robot_name])))
        else:
            vel_vect = self.calc_vel_vect_2d(robot_name, robot_pos)
            self.sim.setAgentPrefVelocity(self.agents[robot_name], vel_vect)

# Running a simulation of the movement of a group of targets
    def run_orca(self):
        print('\nORCA3D for ' + str(len(self.agents)) + ' agents is running.')
        cont_flag = True
        while cont_flag:
            cont_flag = False
            self.sim.doStep()
            for key in self.robots.keys():
                if not self.robots[key].finished_planning:
                    cont_flag = True
                    pos = self.sim.getAgentPosition(self.agents[key])
                    p1 = self.robots[key].last_goal
                    p2 = self.robots[key].current_goal
                    z = self.find_z(p2)
                    robot_pos = Point(pos[0], pos[1], z)
                    goal = self.robots[key].goal_point
                    dist = robot_pos.get_distance_to(goal)
                    vel = self.sim.getAgentVelocity(self.agents[key])
                    #print(key + ' dist to goal: ' + str(dist))
                    #print(self.robots[key].name + ' velocity: ' + str(vel))
                    #gc.spawn_sdf_model('v_' + key + '_' + str(len(self.final_paths[key])), gc_const.VERTICE_PATH, robot_pos)
                    self.final_paths[key].append(robot_pos)
                    self.goal_achievement_check(key, robot_pos)
        print('ORCA for ' + str(len(self.agents)) + ' agents is completed!')
        #f = open("/root/catkin_ws/src/targets_path_planning/orca_3d_data.txt", "a+")
        #f.write('\n\nORCA3D\n')
        #f.close()
        for key in self.robots.keys():
            self.robots[key].set_final_path(self.final_paths[key])
            #f = open("/root/catkin_ws/src/targets_path_planning/orca_3d_data.txt", "a+")
	    #f.write(str(key) + ': [')
            #f.close()
            #for state in self.final_paths[key]:
            #    f = open("/root/catkin_ws/src/targets_path_planning/orca_3d_data.txt", "a+")
	    #    f.write('Point(' + str(state.x) + ', ' + str(state.y) + ', ' + str(state.z) + '), ')
            #    f.close()
            #f = open("/root/catkin_ws/src/targets_path_planning/orca_3d_data.txt", "a+")
	    #f.write(']\n\n')
            #f.close()
            self.robots[key].start()

    def run_orca_2d(self):
        print('\nORCA for ' + str(len(self.agents)) + ' agents is running.')
        cont_flag = True
        while cont_flag:
            cont_flag = False
            self.sim.doStep()
            for key in self.robots.keys():
                if not self.robots[key].finished_planning:
                    cont_flag = True
                    pos = self.sim.getAgentPosition(self.agents[key])
                    p1 = self.robots[key].last_goal
                    p2 = self.robots[key].current_goal
                    z = self.find_z(p2)
                    robot_pos = Point(pos[0], pos[1], z)
                    goal = self.robots[key].goal_point
                    dist = robot_pos.get_distance_to(goal)
                    vel = self.sim.getAgentVelocity(self.agents[key])
                    #print(self.robots[key].name + ' velocity: ' + str(vel))
                    #gc.spawn_sdf_model('v_' + key + '_' + str(len(self.final_paths[key])), gc_const.VERTICE_PATH, robot_pos)
                    self.final_paths[key].append(robot_pos)
                    self.goal_achievement_check_2d(key, robot_pos)
        print('ORCA for ' + str(len(self.agents)) + ' agents is completed!')
        f = open("/root/catkin_ws/src/targets_path_planning/orca_data.txt", "a+")
        f.write('\n\nORCA\n')
        f.close()
        for key in self.robots.keys():
            self.robots[key].set_final_path(self.final_paths[key])
            f = open("/root/catkin_ws/src/targets_path_planning/orca_data.txt", "a+")
	    f.write(str(key) + ': [')
            f.close()
            for state in self.final_paths[key]:
                f = open("/root/catkin_ws/src/targets_path_planning/orca_data.txt", "a+")
	        f.write('Point(' + str(state.x) + ', ' + str(state.y) + ', ' + str(state.z) + '), ')
                f.close()
            f = open("/root/catkin_ws/src/targets_path_planning/orca_data.txt", "a+")
	    f.write(']\n\n')
            f.close()
            #self.robots[key].start()
