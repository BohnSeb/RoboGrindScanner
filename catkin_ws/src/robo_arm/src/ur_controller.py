#!/usr/bin/env python
from robo_arm.ur import UR, Box
from robo_arm.util import Rotation
import rospy
import geometry_msgs.msg
from std_msgs.msg import Bool, String
from robo_arm.util import WaypointList

def initPublisher(topic, msg_type, queue_size=10, latch=True):
    return rospy.Publisher(topic, msg_type, queue_size=queue_size, latch=latch)

def initSubscriber(topic, msg_type, callback):
    return rospy.Subscriber(topic, msg_type, callback)

class PathSubscriber:
    def __init__(self, base_topic, ur, queue_size=10):
        #receiver
        self.waypoint_list = None
        self.is_listening = False
        self.ok_publisher = rospy.Publisher(base_topic + "/ok", Bool, queue_size=1, latch=False)
        self.ur = ur
        rospy.Subscriber(base_topic+"/start", Bool, self.receiveStart)
        rospy.Subscriber(base_topic+"/pose", geometry_msgs.msg.Pose, self.receiveWaypoint)
        rospy.Subscriber(base_topic+"/finish", Bool, self.receiveFinish)
    
    def receiveWaypoint(self, pose):
        if self.is_listening:
            self.waypoint_list.addWaypoint(pose)

    def receiveStart(self, msg):
        self.waypoint_list = WaypointList()
        self.is_listening = True

    def receiveFinish(self, msg):
        self.is_listening = False
        self.ur.travelPath(self.waypoint_list)
        self.ok_publisher.publish(Bool())
        self.waypoint_list = None
        

class UrController:
    def __init__(self):
        self.ur = UR("ur_controller", anonymous=True)
        rospy.sleep(2)
        self.pose_publisher = initPublisher("ur_pose/out", geometry_msgs.msg.Pose)
        self.start_subscriber = initSubscriber("ur_pose/start", Bool, self.moveToStartingPosition)
        self.pose_subscriber = initSubscriber("ur_pose/in", geometry_msgs.msg.Pose, self.receivePoseMessage)
        self.ok_publisher = initPublisher("ur/ok", Bool)
        self.path_subscriber = PathSubscriber("ur_path", self.ur)
        self.moveToStartingPosition()
        
        

    def moveToStartingPosition(self, msg=Bool(False)):
        #self.ur.setJointsWithAngle(0, -81, 102, -20, 90, -90)
        self.ur.setJointsWithAngle(-50, -143, 146, -90, 90, -40)
        #self.ur.setJointsWithAngle(-35, -42, 134, -95, 56, -90)
        #self.ur.setJointsWithAngle(-33, -68, 130, -62, 57, -90)
        #self.ur.setJointsWithAngle(-51, -120, 112, 9, 39, -90)
        rotation = Rotation(0, 90, 0)
        pose = self.ur.getPose()
        pose.orientation = rotation.asMoveitQuaternion()
        self.ur.planAndMove(pose)
        self.ok_publisher.publish(Bool(True))

    def addBox(self, name, header_frame, x, y, z):
        box = Box(name, x, y, z)
        box.setHeaderFrameId(header_frame)
        box.addToScene(self.ur.scene)
        return box

    def removeBox(self, name):
        self.ur.scene.remove_world_object(name)

    def addTool(self, name, x, y, z):
        tool = self.addBox(name, "tool0", x, y, z)
        self.ur.grabObject(tool)
        return tool

    def removeTool(self, tool):
        self.ur.releaseObject(tool)
        tool.removeFromScene(self.ur.scene)

    def receivePoseMessage(self, pose):
        rospy.loginfo("Moving to: {}".format(pose))
        wp = WaypointList()
        wp.addWaypoint(pose)
        succesfull = self.ur.travelPath(wp)
        response = Bool()
        response.data = succesfull
        self.ok_publisher.publish(response)

    def sendCurrentPose(self):
        self.pose_publisher.publish(self.ur.getPose())

    def loopCurrentPose(self, frequenzy):
        rate = rospy.Rate(frequenzy)
        while not rospy.is_shutdown():
            self.sendCurrentPose()
            rate.sleep()

def addArtec(controller):
        length = 0.158
        width = 0.063
        height = 0.262
        tool = Box("Artec", length, width, height)
        r = Rotation(0,0,0)
        tool.setRotation(r)
        tool.pose.pose.position.x -= length/2 - 0.03
        tool.pose.pose.position.z += 0.1
        tool.setHeaderFrameId("tool0")
        tool.addToScene(controller.ur.scene)
        controller.ur.grabObject(tool)
        return tool

def addFloor(controller):
    length = 10
    width = 10
    height = 0.1
    floor = Box("Floor", length, width, height)
    floor.setHeaderFrameId("base")
    floor.setRotation(Rotation(0, 0, 0))
    floor.pose.pose.position.z -= height + 0.095
    floor.addToScene(controller.ur.scene)
    return floor

def addPodest(controller):
    length = 0.16
    width = 0.16
    height = 0.2
    podest = Box("Podest", length, width, height)
    podest.setHeaderFrameId("base")
    podest.setRotation(Rotation(0, 0, 0))
    podest.pose.pose.position.z -= height
    podest.pose.pose.position.x += length + 0.02
    podest.addToScene(controller.ur.scene)
    return podest


def main():
    controller = UrController()
    #tool = controller.addTool("Artec", 0.158,0.063,0.262)
    #tool = controller.addArtec()
    tool = addArtec(controller)
    floor = addFloor(controller)
    podest = addPodest(controller)
    try:
        controller.loopCurrentPose(1)
    except rospy.ROSInterruptException:
        controller.removeTool(tool)
        controller.removeBox(floor.name)
        controller.removeBox(podest)

if __name__ == "__main__":
    main()