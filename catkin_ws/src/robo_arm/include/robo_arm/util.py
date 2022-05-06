import numpy as np
from geometry_msgs.msg import Quaternion
import math
import copy

class Rotation():
    def __init__(self, roll,pitch,yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def asRadians(self):
        return math.radians(self.roll), math.radians(self.pitch), math.radians(self.yaw)

    def asQuaternion(self):
        (roll, pitch, yaw) = self.asRadians()
        x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return x,y,z,w

    def asMoveitQuaternion(self):
        (x,y,z,w) = self.asQuaternion()
        return Quaternion(x,y,z,w)

class WaypointList:
    def __init__(self):
        self.waypoints = []

    def addWaypoint(self, pose):
        self.waypoints.append(copy.deepcopy(pose))

    def addWaypoints(self, points):
        for waypoint in points:
            self.addWaypoint(waypoint)

    def __add__(self, other):
        result = WaypointList()
        result.addWaypoints(self.waypoints)
        result.addWaypoints(other.waypoints)
        return result

    def getInverted(self):
        inverted = self.waypoints[::-1]
        result = WaypointList()
        result.addWaypoints(inverted)
        return result