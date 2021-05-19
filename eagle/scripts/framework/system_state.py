#!/usr/bin/env python

from enum import Enum

class ESystemState(Enum):
    DISCONNECTED = 0
    SETUP_MODE = 1
    IDLE = 2
    
    # Working states
    TAKEOFF = 3
    WORKING = 4
    LANDING = 5