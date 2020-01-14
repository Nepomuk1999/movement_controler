#!/usr/bin/env python

import os
import sys
import time
import traceback
import actionlib
import matplotlib.pyplot as plt
import numpy as np
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

if os.name == 'nt':
    pass
else:
    import termios

class MovementController:

    def __init__(self):
        self._move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()
        print 'move base server connected'
        self._labyrinth_explorer = rospy.Subscriber('/explorer_goal_pos_result', MoveBaseGoal,
                                                    self.labyrinth_explorer_callback)

        self._labyrinth_explorer_clint = actionlib.SimpleActionClient('/explorer_goal_pos', MoveBaseAction)
        self._labyrinth_explorer_clint.wait_for_server()
        print 'labirynth explore server connected'
        self._status = 'mapping'
        self._old_goal_msg = None
        self._current_goal_msg = None

    def labyrinth_explorer_callback(self, data):
        self._old_goal_msg = self._current_goal_msg
        self._current_goal_msg = data

    def control_loop(self):
        print 'movment_controler start loop'
        while not rospy.is_shutdown():
            if self._status == 'mapping':
                self._labyrinth_explorer_clint.send_goal(MoveBaseGoal())
                self._labyrinth_explorer_clint.wait_for_result()
                self._move_base_client.send_goal(self._current_goal_msg)
                self._move_base_client.wait_for_result()



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
