#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading as thr
from targets_path_planning.msg import Path, NamesList

class MsgListener(thr.Thread):

    def __init__(self):
        thr.Thread.__init__(self)
        self.c_names = []
        self.c_names_sub = rospy.Subscriber('chargers_names', NamesList, self.c_names_callback)

    def c_names_callback(self, msg):
        self.c_names = msg.names

