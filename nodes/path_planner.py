#!/usr/bin/env python

# Path planning node for ground targets
import path_planning.PathPlanner as pp
import rospy
import copy
from targets_path_planning.msg import Point3d, AllPaths, Path, AllRobotsPos
from path_planning.Point import Point, Vector2d
from path_planning.Heightmap import Heightmap
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
from path_planning.ORCA import ORCAsolver

def group_path_planning():
    paths_pub = rospy.Publisher('all_paths_data', AllPaths, queue_size=10)
    hm = Heightmap()
    hmap, height, width, x_step, y_step, grid_step = hm.prepare_heightmap()
    map_handler = pp.PathPlanner(hmap, height, width, grid_step, x_step, y_step)
    map_handler.cell_maker()
    cells = map_handler.cells
    map_handler.gridmap_preparing()
    orca = ORCAsolver(map_handler.map, cells, x_step, y_step)
    smoothed_paths = {}
    robot_names = []
    for name in gc_const.ROBOT_NAMES:
        robot_names.append(name)
        model_directory = gc_const.ROBOT_MODEL_PATH + name[len(name) - 1] + ".urdf"
        robot_pos = map_handler.get_random_start_pos()
        gc.spawn_urdf_model(name, model_directory, robot_pos)
        robot_orient = gc.get_robot_orientation_vector(name)
        start_id, goal_id = map_handler.get_start_and_goal_id(robot_pos, robot_orient)
        print('\nPath planning for ' + name + ' has begun!')
        path, path_ids, path_cost = map_handler.find_path(start_id, goal_id, robot_orient)
        if path:
            orca.add_agent(name, path)
        print('Current agents count: ' + str(orca.sim.getNumAgents()))
    paths = orca.run_orca()
    msg = prepare_paths_msg(paths.keys(), paths)
    paths_pub.publish(msg)

def prepare_paths_msg(names, paths):
    msg = AllPaths()
    msg.path_list = []
    for name in names:
        path = paths[name]
        path_msg = prepare_path_msg(name, path)
        msg.path_list.append(path_msg)
    return msg

def prepare_path_msg(name, path):
    msg = Path()
    msg.path = []
    msg.robot_name = name
    for state in path:
        point = Point3d()
        point.x = state.x
        point.y = state.y
        point.z = state.z
        msg.path.append(point)
    return msg

def convert_to_vector(msg_data):
    x = msg_data.x
    y = msg_data.y
    point = Vector2d(x, y)
    return point

def convert_to_point(msg_data):
    x = msg_data.x
    y = msg_data.y
    z = msg_data.z
    point = Point(x, y, z)
    return point

rospy.init_node('path_planner')
group_path_planning()
rospy.spin()
