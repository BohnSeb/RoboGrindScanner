#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("artec_comm_tester", anonymous=False)
    starter = rospy.Publisher("artec_capture/start", String, queue_size=10, latch=True)
    finisher = rospy.Publisher("artec_capture/stop", String, queue_size=10, latch=True)
    rospy.sleep(1)
    starter.publish("Moin,Servus, Moin!")
    rospy.sleep(1)
    starter.publish("Moin,Servus, Moin!")
    rospy.sleep(1)
    starter.publish("Moin,Servus, Moin!")
    rospy.sleep(30)
    finisher.publish("Stoppen Sie dies!!!")
    rospy.sleep(1)

if __name__ == "__main__":
    main()