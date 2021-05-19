#!/usr/bin/env python

import rospy

from framework.system import CSystem

if __name__ == "__main__":
    system = CSystem()
    try:
        system.Run()
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROSInterruptException: %s", e.message)
    system.Exit()