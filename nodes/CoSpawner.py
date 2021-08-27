#!/usr/bin/env python
import threading
import multiprocessing
import rospy
from targets_path_planning.msg import AllPaths, Path
from Point import Point, Vector2d
from geometry_msgs.msg import Pose, Twist
import GazeboCommunicator as gc
import GazeboConstants as gc_const
import PathPlanner as pp
import Constants as const
from ORCA import ORCAsolver
from Heightmap import Heightmap
import copy
from math import sqrt, asin, fabs, pi
from time import sleep
from time import time
import random
from RpcClient import RpcClient
# from threading import Thread
from Point import Point as PointGeom, Vector2d
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose, Twist, Quaternion, Point
from GridHeightmap import GridHeightmapHandler


class CoSpawner(multiprocessing.Process):

    def __init__(self, *args, **kwargs):
        multiprocessing.Process.__init__(self)
        self.topleft = kwargs.get("TL", None)
        self.bottomright = kwargs.get("BR", None)
        self.id = kwargs.get("id")
        self.waypoint = kwargs.get("waypoint")
        self.spawn_point = kwargs.get("spawn_point", None)
        self.robot = None
        self.rover_name = "sim_p3at" + str(self.id)
        self.grid_map_handler = kwargs.get("grid_map_handler")

    def group_path_planning(self):
        paths_pub = rospy.Publisher('all_paths_data', AllPaths, queue_size=10)
        # print(self.rover_name)
        print('spawning ' + self.rover_name)
        if self.topleft and self.bottomright:
            robot_pos = self.grid_map_handler.map_handler.get_random_point_from_rectangle(
                int(self.topleft["x"]),
                int(self.bottomright["x"]),
                int(self.topleft["y"]),
                int(self.bottomright["y"])
            )

        else:
            start_id = self.grid_map_handler.map_handler.get_start_id(int(self.spawn_point["x"]),int(self.spawn_point["y"]), 10)
            robot_pos = self.grid_map_handler.map_handler.heightmap[start_id]

        print("robot_pos", robot_pos)
        gc.spawn_target(self.rover_name, robot_pos, [0,0,0,0])
        robot_orient = gc.get_robot_orientation_vector(self.rover_name)
        goal_id = self.grid_map_handler.map_handler.get_goal_id(self.waypoint["x"], self.waypoint["y"], 10)
        print('\nPath planning for ' + self.rover_name + ' has begun!')
        try:
            path, path_ids, path_cost = self.grid_map_handler.map_handler.find_path(start_id, goal_id, robot_orient)
            if path:
                print("PATH!!!!")
                self.grid_map_handler.orca.add_agent(self.rover_name, path)
        except Exception:
            print("cannot_find_path")
        print('Current agents count: ' + str(self.grid_map_handler.orca.sim.getNumAgents()))


    def spawn_robot(self):

        print('Finish!')
        self.group_path_planning()
