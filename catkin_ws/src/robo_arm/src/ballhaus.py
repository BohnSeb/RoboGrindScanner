#!/usr/bin/env python
from robo_arm.ur import UR
import numpy as np
import math
from robo_arm.util import Rotation
from robo_arm.util import WaypointList
import copy

def curve(distance):
    pass

def calcTargetPoint(pos, rotation, distance):
    pass

def startPos(ur):
    ur.setJointsWithAngle(0,-45,90,-135,90,-90)

def createLine(start_pose, dx, dy, dz):
    wp = WaypointList()
    pose = start_pose
    pose.position.x += dx
    pose.position.y += dy
    pose.position.z += dz
    wp.addWaypoint(pose)
    return wp

def createLineWithRotation(start_pose, dx, dy, dz, r, p, y):
    rotation = Rotation(r,p,y)
    pose = start_pose
    pose.orientation = rotation.asMoveitQuaternion()
    return createLine(pose, dx, dy, dz)



def createBallhaus(starting_pose, distance, degrees, direction, offset=90):
    wp = WaypointList()
    pose = copy.deepcopy(starting_pose)
    for a in range(0,degrees + 1):
        x = distance * math.cos(math.radians(a)) * -1 + distance
        z = distance * math.sin(math.radians(a))
        pose.position.x = starting_pose.position.x + x
        pose.position.z = starting_pose.position.z + z * direction # direction needs to be -1 or 1
        rotation = Rotation(0,90 + a * direction,0)
        pose.orientation = rotation.asMoveitQuaternion()
        wp.addWaypoint(pose)
    return wp

def calcReverseCirclePosition(radius, angle):
    x = radius * math.cos(math.radians(angle)) * -1 + radius
    y = radius * math.sin(math.radians(angle))
    return x,y

def calcReverseSphere(radius, horizontal_angle, vertical_angle):
    x,z = calcReverseCirclePosition(radius, vertical_angle)
    x2,y = calcReverseCirclePosition(radius, horizontal_angle)
    x += x2
    return x,y,z

def create3dBallhausPose(starting_pose, distance, horizontal_angle, vertical_angle, vertical_rotation_offset):
    x,y,z = calcReverseSphere(distance, horizontal_angle, vertical_angle)
    rotation = Rotation(0,vertical_angle  + vertical_rotation_offset, -1 *horizontal_angle)
    pose = copy.deepcopy(starting_pose)
    pose.position.x += x
    pose.position.y += y
    pose.position.z += z
    pose.orientation = rotation.asMoveitQuaternion()
    return pose

def create3dBallhaus(starting_pose, distance, angle, vertical_rotation_offset):
    wp = WaypointList()
    pose = create3dBallhausPose(starting_pose, distance, angle, angle, vertical_rotation_offset)
    wp.addWaypoint(pose)
    pose = create3dBallhausPose(starting_pose, distance, angle, angle * -1, vertical_rotation_offset)
    wp.addWaypoint(pose)
    pose = create3dBallhausPose(starting_pose, distance, angle * -1, angle * -1, vertical_rotation_offset)
    wp.addWaypoint(pose)
    pose = create3dBallhausPose(starting_pose, distance, angle * -1, angle, vertical_rotation_offset)
    wp.addWaypoint(pose)
    pose = create3dBallhausPose(starting_pose, distance, 0, 0, vertical_rotation_offset)
    wp.addWaypoint(pose)
    return wp
    


def main():
    distance = 0.1 #in meter
    degrees = 15 # in degrees (goes up and down)
    ur = UR()
    startPos(ur)
    ballhaus_startpos = createLineWithRotation(ur.getPose(), -0.18, 0, -0.1, 0, 90, 0)
    ur.travelPath(ballhaus_startpos)
    bh = create3dBallhaus(ur.getPose(), distance, degrees, 90)
    ur.travelPath(bh)
    return

    startPos(ur)
    #wp = createCurve(ur.getPose(), distance)
    ballhaus_startpos = createLineWithRotation(ur.getPose(), -0.18, 0, -0.1, 0, 90, 0)
    ur.travelPath(ballhaus_startpos)

    #create test paths for ballhaus
    ballhaus_up = createBallhaus(ur.getPose(), distance, degrees, 1)
    ballhaus_back = ballhaus_up.getInverted()
    ballhaus_down = createBallhaus(ur.getPose(), distance, degrees, -1)
    ballhaus_back_from_down = ballhaus_down.getInverted()
    #execute them
    ur.travelPath(ballhaus_up)
    ur.travelPath(ballhaus_back)
    ur.travelPath(ballhaus_down)
    ur.travelPath(ballhaus_back_from_down)


if __name__ == "__main__":
    main()