from robo_arm.util import WaypointList, Rotation
import math
import copy

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

def calcReverseCirclePosition(radius, angle):
    x = radius * math.cos(math.radians(angle)) * -1 + radius
    y = radius * math.sin(math.radians(angle))
    return x,y

def calcReverseSphere(radius, horizontal_angle, vertical_angle):
    x,z = calcReverseCirclePosition(radius, vertical_angle)
    x2,y = calcReverseCirclePosition(radius, horizontal_angle)
    x += x2
    return x,y,z

def create3dBallhausPose(starting_pose, distance, horizontal_angle, vertical_angle, vertical_rotation_offset=0, x_offset=0, y_offset=0, z_offset=0):
    x,y,z = calcReverseSphere(distance, horizontal_angle, vertical_angle)
    rotation = Rotation(0,vertical_angle  + vertical_rotation_offset, -1 *horizontal_angle)
    pose = copy.deepcopy(starting_pose)
    delta_x = z_offset * math.sin(math.radians(vertical_angle))
    pose.position.x += x - delta_x
    pose.position.y += y + delta_x * math.sin(math.radians(horizontal_angle))
    pose.position.z += z - z_offset * math.cos(math.radians(vertical_angle)) + z_offset
    pose.orientation = rotation.asMoveitQuaternion()
    return pose

def create3dBallhaus(starting_pose, distance, angle, vertical_rotation_offset = 0, x_offset=0, y_offset=0, z_offset=0):
    wp = WaypointList()
    pose = create3dBallhausPose(starting_pose, distance, angle, angle, vertical_rotation_offset, x_offset, y_offset, z_offset)
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

def create3dBallhausRange(starting_pose, distance, horizontal_angle, vertical_angle_min, vertical_angle_max, steps, x_offset=0, y_offset=0, z_offset=0):
    wp = WaypointList()
    for horizontal in range(steps, horizontal_angle, steps):
        wp.addWaypoint(create3dBallhausPose(starting_pose, distance, horizontal, vertical_angle_min, x_offset=x_offset, y_offset=y_offset, z_offset=z_offset))
    for horizonal in range(-1 * steps, -1 * horizontal_angle, steps * -1):
        wp.addWaypoint(create3dBallhausPose(starting_pose, distance, horizontal, vertical_angle_min, x_offset=x_offset, y_offset=y_offset, z_offset=z_offset))
    for horizonal in range(-1 * steps, -1 * horizontal_angle, steps * -1):
        wp.addWaypoint(create3dBallhausPose(starting_pose, distance, horizontal, vertical_angle_max, x_offset=x_offset, y_offset=y_offset, z_offset=z_offset))
    for horizontal in range(steps, horizontal_angle, steps):
        wp.addWaypoint(create3dBallhausPose(starting_pose, distance, horizontal, vertical_angle_max, x_offset=x_offset, y_offset=y_offset, z_offset=z_offset))
    return wp

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