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

        self.tkoffVelocity      = 0.5
        self.tkoffHeight        = 2.0

        self.landVelocity       = 0.5
        self.landHeight         = 0.12

## Global variable to refer to
g_config = CSolutionConfig()