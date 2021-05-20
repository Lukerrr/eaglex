#!/usr/bin/env python

import rospy

## Returns False if node is shutting down
def ok():
    return not rospy.is_shutdown()

## Returns current time
def now():
    return rospy.Time.now()