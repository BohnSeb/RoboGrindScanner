#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import geometry_msgs.msg

def send(pub):
    rate = rospy.Rate(1)
    i = 0
    #while not rospy.is_shutdown():
    test_str = "Hallo Jonas! {}".format(i)
    pub.publish(test_str)
    rate.sleep()
    i += 1
    test_str = "Hallo Jonas! {}".format(i)
    pub.publish(test_str)

def listen():
    rospy.Subscriber("artec_capture_in", String, callback)

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + " I heard: %s", msg.data)

def main():
    rospy.init_node('manager', anonymous=False)
    pub = rospy.Publisher('ur_pose_in', geometry_msgs.msg.Pose, queue_size=10, latch=True)
    rospy.sleep(2)
    pub.publish(geometry_msgs.msg.Pose())


if __name__ == "__main__":
    main()