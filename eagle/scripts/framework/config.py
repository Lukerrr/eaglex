#!/usr/bin/env python

################################################################################
## Contains config variables for the solution.
##
##
## Vars:
##  deltaSeconds - working loop step in seconds
##  tkoffVelocity - takeoff vertical velocity (in m/s)
##  tkoffHeight - takeoff height (distance to ground) in meters
##  landVelocity - landing vertical velocity (in m/s)
##  landHeight - height (distance to ground) when drone is landed (in meters)
################################################################################
class CSolutionConfig():
    def __init__(self):
        self.deltaSeconds       = 0.02

## Global variable to refer to
g_config = CSolutionConfig()