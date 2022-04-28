import copy
class WaypointList():
    def __init__(self):
        self.waypoints = []

    def addWaypoint(self, pose):
        self.waypoints.append(copy.deepcopy(pose))