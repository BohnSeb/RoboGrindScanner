#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
import geometry_msgs.msg
import copy
from robo_arm.positioning import create3dBallhausPose, create3dBallhaus
from robo_arm.util import Rotation, WaypointList

class PoseException(Exception):
    def __init__(self, msg):
        self.message = msg

class PathPublisher:
    def __init__(self, base_topic, queue_size=10):
        self.start_publisher = rospy.Publisher(base_topic + "/start", Bool, queue_size=1, latch=False)
        self.pose_publisher = rospy.Publisher(base_topic + "/pose", geometry_msgs.msg.Pose, queue_size=queue_size, latch=False)
        self.finish_publisher = rospy.Publisher(base_topic + "/finish", Bool, queue_size=1, latch=False)
        self.waypoints = WaypointList()
        self.ok = False
        rospy.Subscriber(base_topic+"/ok", Bool, self.receiveOk)

    def sendWaypoints(self, waypoints):
        self.start_publisher.publish(Bool(True))
        rospy.sleep(0.5)
        for waypoint in waypoints:
            self.pose_publisher.publish(waypoint)
            self.ok = False
        rospy.sleep(0.5)
        self.finish_publisher.publish(Bool(True))
        self.awaitResponse()
    
    def awaitResponse(self):
        while not self.ok:
            rospy.sleep(0.1)

    def sendAll(self):
        self.sendWaypoints(self.waypoints)

    def receiveOk(self, msg):
        self.ok = True

class Manager:
    def __init__(self):
        rospy.init_node("manager", anonymous=False)
        self.pose_publisher = rospy.Publisher("ur_pose/in", geometry_msgs.msg.Pose, queue_size=10, latch=True)
        self.current_pose = geometry_msgs.msg.Pose()
        self.pose_subscriber = rospy.Subscriber("ur_pose/out", geometry_msgs.msg.Pose, self.receivePose)
        self.pose_starting_publisher = rospy.Publisher("ur_pose/start", Bool, queue_size=1, latch=False)
        self.path_publisher = PathPublisher("ur_path")
        self.pose_response = None
        rospy.Subscriber("ur/ok", Bool, self.receiveOK)
        rospy.sleep(2)

    def sendWaypoints(self, waypoints):
        self.path_publisher.sendWaypoints(waypoints)
        self.awaitResponse()

    def receivePose(self, pose):
        #rospy.loginfo(rospy.get_caller_id() + " I heard: %s", pose)
        self.current_pose = copy.deepcopy(pose)

    def receiveOK(self, msg):
        self.pose_response = msg.data

    def sendPose(self, pose):
        self.pose_response = None
        self.pose_publisher.publish(copy.deepcopy(pose))
        self.awaitResponse()

    def awaitResponse(self):
        while self.pose_response == None:
            rospy.sleep(0.1)
        if not self.pose_response:
            raise PoseException("Pose could not be reached!")

    def getCurrentPose(self):
        return copy.deepcopy(self.current_pose)

    def startingPosition(self):
        self.pose_response = None
        self.pose_starting_publisher.publish(Bool(True))
        self.awaitResponse()

    def sendWaypoints(self, waypoints):
        self.path_publisher.sendWaypoints(waypoints)

    
        

def listen_to_sensor():
    rospy.Subscriber("artec_capture_in", String, callback)

def horizontal_ballhaus(manager, base_pose, distance, max_horizontal_angle, vertical_angle, resolution=1, z_offset=0):
    try:
        for a in range(resolution, max_horizontal_angle+1, resolution):
            manager.sendPose(create3dBallhausPose(base_pose, distance, a, vertical_angle, z_offset=z_offset))
    except PoseException:
        print("reached limit at {} Degrees".format(a))

    try:
        for a in range(-1 * resolution, -1 * max_horizontal_angle+1, -1 * resolution):
            manager.sendPose(create3dBallhausPose(base_pose, distance, a, vertical_angle, z_offset=z_offset))
    except PoseException:
        print("reached limit at {} Degrees".format(a))

def main():
    manager = Manager()
    manager.startingPosition()

    distance = 0.5
    angle = 90
    steps = 5
    z_offset = 0.17
    
    base_pose = manager.getCurrentPose()
    #horizontal_ballhaus(manager, base_pose, distance, angle, 0, resolution=steps, z_offset=z_offset)
    #horizontal_ballhaus(manager, base_pose, distance, angle, 45, resolution=steps, z_offset=z_offset)
    #horizontal_ballhaus(manager, base_pose, distance, angle, 90, resolution=steps, z_offset=z_offset)
    #wp = create3dBallhaus(base_pose, distance, 20, z_offset=z_offset)
    wp = WaypointList()
    for a in range(5, 61, 5):
        wp.addWaypoint(create3dBallhausPose(base_pose, distance, a, 0, z_offset=z_offset))
    for a in range(-5, -61, -5):
        wp.addWaypoint(create3dBallhausPose(base_pose, distance, a, 0, z_offset=z_offset))
    
    print(wp.waypoints)
    manager.sendWaypoints(wp.waypoints)

    manager.startingPosition()

if __name__ == "__main__":
    main()