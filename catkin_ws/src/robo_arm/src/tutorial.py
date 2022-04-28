#!/usr/bin/env python
from robo_arm.ur import UR
from robo_arm.waypointlist import WaypointList

def moveUp(robot, distance):
    pose_goal = robot.getPose()
    pose_goal.position.z += distance
    robot.planAndMove(pose_goal)

def moveDown(robot, distance):
    moveUp(robot, -1 * distance)

def main2():
    robot = UR()
    #go to a starting position
    robot.setJointsWithAngle(0,-45,90,-225,-90,0)

    #go up and down
    #moveDown(robot, 0.1)
    #moveUp(robot, 0.2)
    #moveDown(robot, 0.1)

    #same in carthesian path
    wp = WaypointList()
    pose = robot.getPose()
    startpose = pose
    pose.position.z -= 0.05
    wp.addWaypoint(pose)
    pose.position.z += 0.1
    wp.addWaypoint(pose)
    pose.position.z -= 0.05
    pose.position.x += 0.05
    pose.position.y += 0.05
    wp.addWaypoint(pose)
    pose.position.x -= 0.1
    wp.addWaypoint(pose)
    pose.position.y -= 0.1
    wp.addWaypoint(pose)
    pose.position.x += 0.1
    wp.addWaypoint(pose)
    pose.position.x -= 0.05
    pose.position.y += 0.05
    wp.addWaypoint(pose)
    #wp.addWaypoint(startpose)
    #execute cartesian path
    print("============ waypoints ============")
    print(wp.waypoints)
    robot.travelPath(wp)

    #move in a collision space
    robot.setJointsWithAngle(0,0,180,0,0,0)

if __name__ == "__main__":
    main2()