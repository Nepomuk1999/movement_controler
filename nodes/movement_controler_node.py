#!/usr/bin/env python

import os
import sys
import time
import traceback
import actionlib
import matplotlib.pyplot as plt
import numpy as np
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

if os.name == 'nt':
    pass
else:
    import termios

GOAL_MIN_DIST_TO_WALL = 5


class MovementController:

    def __init__(self):
        self._client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self._client.wait_for_server()
        self._labyrinth_explorer = rospy.Subscriber('/explorer_goal_pos', MoveBaseGoal, self.labyrinth_explorer_callback)
        self._status = 'mapping'
        self._current_goal_msg = None
        while self._current_goal_msg is None:
            time.sleep(2)

    def labyrinth_explorer_callback(self, data):
        self._current_goal_msg = data

    def control_loop(self):
        print 'movment_controler start loop'
        while not rospy.is_shutdown():
            if self._status == 'mapping':
                self._client.send_goal(self._current_goal_msg)


def main():
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('movement_controler')
    try:
        mc = MovementController()
        mc.control_loop()
    except Exception as e:
        print e
        traceback.print_exc()


if __name__ == "__main__":
    main()
