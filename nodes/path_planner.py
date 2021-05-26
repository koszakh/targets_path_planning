#!/usr/bin/env python

# Path planning node for ground targets
import path_planning.PathPlanner as pp
import rospy
import copy
from targets_path_planning.msg import Point3d, AllPaths, AllRobotsPos, Path, RobotPos
from path_planning.Point import Point
from path_planning.Heightmap import Heightmap
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
from path_planning.ORCA import ORCAsolver

def callback(msg_data):
    paths_msg = AllPaths()
    paths_msg.path_list = []
    hm = Heightmap()
    hmap, height, width, x_step, y_step, grid_step = hm.prepare_heightmap()
    map_handler = pp.PathPlanner(hmap, height, width, grid_step)
    map_handler.cell_maker()
    cells = map_handler.cells
    map_handler.gridmap_preparing()
    orca = ORCAsolver(hmap, cells, x_step, y_step)
    #f = open("/root/catkin_ws/src/targets_path_planning/paths_data.txt", "w+")
    #f.close()
    smoothed_paths = {}
    robot_names = []
    for msg in msg_data.pos_list:
        robot_name = msg.robot_name
        robot_names.append(robot_name)
        robot_pos = convert_to_point(msg.pos)
        start_id, goal_id = map_handler.get_start_and_goal_id(robot_pos)
        print('\nPath planning for ' + robot_name + ' has begun!')
        path, path_ids, path_cost = map_handler.find_path(start_id, goal_id)
        if path:
            smoothed_paths[robot_name] = map_handler.curve_smoothing(path_ids)
            gc.visualise_path(smoothed_paths[robot_name], gc_const.VERTICE_PATH, 'v_' + str(robot_name))
	    #f = open("/root/catkin_ws/src/targets_path_planning/paths_data.txt", "a+")
	    #f.write(str(robot_name) + ': ' + str(smoothed_path) + '\n\n')
            #f.close()
        if smoothed_paths[robot_name]:
            orca.add_agent(robot_name, smoothed_paths[robot_name])
        print('Current agents count: ' + str(orca.sim.getNumAgents()))
    orca.run_orca()

def convert_to_point(msg):
    x = msg.x
    y = msg.y
    z = msg.z
    point = Point(x, y, z)
    return point

def prepare_path_msg(path, robot_name):
    msg = Path()
    msg.robot_name = robot_name
    msg.path = []
    for state in path:
        point = Point3d()
        point.x = state.x
        point.y = state.y
        point.z = state.z
        msg.path.append(point)
    return msg

rospy.init_node('path_planner')
robot_pos_sub = rospy.Subscriber('robots_pos_data', AllRobotsPos, callback)
rospy.spin()
