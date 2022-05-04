#!/usr/bin/env python
from robo_arm.ur import UR, Box
import rospy
from robo_arm.positioning import create3dBallhaus

def main():
    distance = 0.6 #in meter
    degrees = 10 # in degrees (goes up and down)
    ur = UR()
    ur.setJointsWithAngle(0,-81,102,-111,90,-90)
    rospy.sleep(2) #sleep to let the planning execution be ready
    
    #add box with artec measurements to scene
    artec = Box("artec",0.158,0.063,0.262)
    #set its reference frame
    artec.setHeaderFrameId("tool0")
    #add box to scene
    artec.addToScene(ur.scene)
    #attach box to robot
    ur.grabObject(artec)

    #start moving
    bh = create3dBallhaus(ur.getPose(), distance, degrees, z_offset=artec.height/2)
    ur.travelPath(bh)

    #release box
    ur.releaseObject(artec)
    #delete Box
    artec.removeFromScene(ur.scene)

if __name__ == "__main__":
    main()