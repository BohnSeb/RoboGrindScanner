#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import String, Bool, Float32
import geometry_msgs.msg
import copy
from robo_arm.positioning import create3dBallhausPose, create3dBallhaus, create3dBallhausRange
from robo_arm.util import Rotation, WaypointList
import sys

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

class ArtecCommunicator:
    def __init__(self):
        self.starter = rospy.Publisher("artec_capture/start", String, queue_size=10, latch=False)
        self.finisher = rospy.Publisher("artec_capture/stop", String, queue_size=10, latch=False)
        self.ok_receiver = rospy.Subscriber("artec_capture/objsaved", String, self.receiveOk)
        self.distance_publisher = rospy.Publisher("artec_distance/start",String,queue_size=10, latch=False)
        self.distance_subscriber = rospy.Subscriber("artec_distance/distance", Float32, self.receiveDistance)
        self.ok = False
        self.distance = None

    def receiveDistance(self, msg):
        self.distance = msg.data

    def getDistance(self):
        self.distance = None
        self.distance_publisher.publish("detect distance!")
        try:
            while self.distance == None:
                rospy.sleep(0.5)
            #convert from mm to meter
            return self.distance/1000
        except:
            raise Exception("Abort")

    def receiveOk(self, msg):
        self.ok = True

    def startScanning(self):
        self.starter.publish("Start scanning!")
    
    def finishScanning(self):
        self.ok = False
        self.finisher.publish("Stop scanning!")

    def waitTillFinished(self):
        while not self.ok:
            rospy.sleep(0.1)    



class Manager:
    def __init__(self):
        rospy.init_node("manager", anonymous=False, disable_signals=True)
        self.pose_publisher = rospy.Publisher("ur_pose/in", geometry_msgs.msg.Pose, queue_size=10, latch=True)
        self.current_pose = geometry_msgs.msg.Pose()
        self.pose_subscriber = rospy.Subscriber("ur_pose/out", geometry_msgs.msg.Pose, self.receivePose)
        self.pose_starting_publisher = rospy.Publisher("ur_pose/start", Bool, queue_size=1, latch=False)
        self.path_publisher = PathPublisher("ur_path")
        self.pose_response = None
        rospy.Subscriber("ur/ok", Bool, self.receiveOK)
        self.artec_communicator = ArtecCommunicator()
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

def horizontal_ballhaus(manager, base_pose, distance, max_horizontal_angle, vertical_angle, vertical_rotation_offset=0, resolution=1, z_offset=0):
    try:
        for a in range(resolution, max_horizontal_angle+1, resolution):
            manager.sendPose(create3dBallhausPose(base_pose, distance, a, vertical_angle, vertical_rotation_offset=vertical_rotation_offset, z_offset=z_offset))
    except PoseException:
        print("reached limit at {} Degrees".format(a))

    try:
        for a in range(-1 * resolution, -1 * max_horizontal_angle+1, -1 * resolution):
            manager.sendPose(create3dBallhausPose(base_pose, distance, a, vertical_angle, vertical_rotation_offset=vertical_rotation_offset, z_offset=z_offset))
    except PoseException:
        print("reached limit at {} Degrees".format(a))

def vertical_ballhaus(manager, base_pose, distance, max_vertical_angle, horizontal_angle, vertical_rotation_offset=0, resolution=1, z_offset=0):
    try:
        for a in range(resolution, max_vertical_angle, resolution):
            manager.sendPose(create3dBallhausPose(base_pose, distance, horizontal_angle, a, vertical_rotation_offset=vertical_rotation_offset, z_offset=z_offset))
    except PoseException:
        print("reached limit at {} vertical Degrees".format(a))

def askForEnter():
    result = raw_input("Press ENTER to start the next scanning process or press CTRL-C to finish: ")

def main():
    manager = Manager()
    rospy.loginfo("Moving to starting position")
    try:
        manager.startingPosition()
    except PoseException:
        pass # happens if current pose is already the same as the requested pose

    #starter = rospy.Publisher("artec_capture/start", String, queue_size=10, latch=True)
    #finisher = rospy.Publisher("artec_capture/stop", String, queue_size=10, latch=True)
    distance = 0.75
    angle = 90
    steps = 5
    z_offset = 0.17 + 0.1
    vertical_rotation_offset = 0
    
    base_pose = manager.getCurrentPose()
    #starter.publish("los geht's!")
    #horizontal_ballhaus(manager, base_pose, distance, angle, 0, resolution=steps, z_offset=z_offset, vertical_rotation_offset=vertical_rotation_offset)
    #vertical_ballhaus(manager, base_pose, distance, angle, -5, resolution=steps, z_offset=z_offset, vertical_rotation_offset=vertical_rotation_offset)
    #manager.startingPosition()
    try:
        rospy.loginfo("starting the process")
        rospy.loginfo("====================")
        repeat = True
        while repeat:
            rospy.loginfo("Fetching Distance from Artec Scanner...")
            distance = manager.artec_communicator.getDistance()
            rospy.loginfo("complete")

            rospy.loginfo("Starting scanning Job...")
            manager.artec_communicator.startScanning()
            rospy.sleep(4) #delay to give artec time to start scanning process
            rospy.loginfo("complete")

            rospy.loginfo("Moving...")
            horizontal_ballhaus(manager, base_pose, distance, angle, 0, resolution=steps, z_offset=z_offset, vertical_rotation_offset=vertical_rotation_offset)
            vertical_ballhaus(manager, base_pose, distance, angle, -5, resolution=steps, z_offset=z_offset, vertical_rotation_offset=vertical_rotation_offset)
            rospy.loginfo("complete")

            rospy.loginfo("Processing data...")
            manager.artec_communicator.finishScanning()
            #move into starting position
            manager.startingPosition()
            #wait till scan is post processed and aligned
            manager.artec_communicator.waitTillFinished()
            rospy.loginfo("complete")

            #print information
            rospy.loginfo("Finished Scanning Process successfully. Move the Robot-Platform to a new Position to scan the Object from a different perspective.")
            askForEnter()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
        rospy.loginfo("See the results in the scanningfolder on the Artec-Connection-Computer")
    except Exception:
        print("Exiting...")
    
if __name__ == "__main__":
    main()