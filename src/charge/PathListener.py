#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading as thr
from targets_path_planning.msg import Path, Paths

class PathListener(thr.Thread):

    def __init__(self, robot_name):
        thr.Thread.__init__(self)

        self.paths = Paths()
        self.workpoints = Path()

        self.paths_sub = rospy.Subscriber(robot_name + '/waypoints_array', Paths, self.waypoints_callback)
        self.workpoints_sub = rospy.Subscriber(robot_name + '/workpoints_array', Path, self.workpoints_callback)

    def waypoints_callback(self, msg):
        self.paths = msg

    def workpoints_callback(self, msg):
        self.workpoints = msg