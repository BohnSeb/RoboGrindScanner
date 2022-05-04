#!/usr/bin/env python
from robo_arm.ur import UR, Box
import rospy
import geometry_msgs.msg

def initArtecHitbox(ur):
    #add box with artec measurements to scene
    artec = Box("artec",0.158,0.063,0.262)
    #set its reference frame
    artec.setHeaderFrameId("tool0")
    #add box to scene
    artec.addToScene(ur.scene)
    #attach box to robot
    ur.grabObject(artec)
    return artec

def init():
    ur = UR("ur_controller")
    ur.setJointsWithAngle(0,-81,102,-111,90,-90)
    rospy.sleep(2)
    artec = initArtecHitbox(ur)
    return ur, artec

def initPublisher(topic, msg_type, queue_size=10, latch=True):
    pub = rospy.Publisher(topic, msg_type, queue_size=queue_size, latch=latch)
    return pub

def sendCurrentPose(publisher, ur):
    try:
        publisher.publish(ur.getPose())
    except rospy.ROSInterruptException:
        pass

def loopSending(publisher, ur, hz):
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        sendCurrentPose(publisher, ur)
        rate.sleep()

def receivedPoseMessage(pose, ur):
    rospy.loginfo("Moving to: {}".format(pose))
    ur.planAndMove(pose)


def main():
    ur, artec = init()
    pub = initPublisher("ur_pose_out", geometry_msgs.msg.Pose)
    def callback(pose):
        print("Habe empfangen!")
        receivedPoseMessage(pose, ur)
    rospy.Subscriber("ur_pose_in", geometry_msgs.msg.Pose, callback)
    print("rdy")
    loopSending(publisher=pub, ur=ur, hz=1)
    ur.releaseObject(artec)
    artec.removeFromScene(ur.scene)
    #rospy.spin()

if __name__ == "__main__":
    main()