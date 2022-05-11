import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import math
import geometry_msgs.msg
import copy

class UR():
    def __init__(self, name, anonymous=False, disable_signals=False):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node(name, anonymous=anonymous)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("manipulator")
        planning_frame = group.get_planning_frame()
        eef_link = group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        

    def printBasicInformation(self):
        print("============ Available Planning Groups:", self.robot.get_group_names())

        print("============ Planning frame: %s" % self.planning_frame)

        print("============ End effector link: %s" % self.eef_link)

        print "============ Printing robot state"
        print self.robot.get_current_state()
        print ""

    def getJointValues(self):
        return self.group.get_current_joint_values()

    def getPose(self):
        return copy.deepcopy(self.group.get_current_pose().pose)

    def printCurrentJointValues(self):
        joints = self.getJointValues()
        print(joints)

    def printCurrentPose(self):
        pose = self.getPose()
        print(pose)

    def setJointsWithAngle(self, base, shoulder, arm, wrist, hand, tool):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = math.radians(base)
        joint_goal[1] = math.radians(shoulder)
        joint_goal[2] = math.radians(arm)
        joint_goal[3] = math.radians(wrist)
        joint_goal[4] = math.radians(hand)
        joint_goal[5] = math.radians(tool)
        self.joints(joint_goal)

    def joints(self, joints):
        self.group.go(joints, wait=True)
        self.group.stop()

    def moveTo(self, x, y, z, a=None, b=None, c=None, w=None):
        pose_goal = self.getPose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        if a != None:
            pose_goal.orientation.x = a
        if b != None:
            pose_goal.orientation.y = b
        if c != None:
            pose_goal.orientation.z = c
        if w != None:
            pose_goal.orientation.w = w
        self.planAndMove(pose_goal)

    def planAndMove(self, pose):
        self.group.set_pose_target(pose)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def travelPath(self, waypointList):
        (plan, fraction) = self.group.compute_cartesian_path(
            waypointList.waypoints,
            0.01, #eef_steps
            0.0     #jump threshold
        )
        return self.group.execute(plan, wait=True)

    def grabObject(self, object):
        grasping_group = "manipulator"
        #touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, object.name, touch_links="tool0")

    def releaseObject(self, object):
        self.scene.remove_attached_object(self.eef_link, name=object.name)

class Box:
    def __init__(self, name,  length, width, height, pose=geometry_msgs.msg.PoseStamped()):
        self.name = name
        self.length = length
        self.width = width
        self.height = height
        self.pose = pose
        self.pose.header.frame_id = ""
        self.pose.pose.position.z = self.height / 2

    def setHeaderFrameId(self, frame_id):
        self.pose.header.frame_id = frame_id

    def setPose(self, pose):
        self.pose.pose=pose

    def setRotation(self, rotation):
        self.pose.pose.orientation = rotation.asMoveitQuaternion()
    
    def addToScene(self, scene):
        scene.add_box(self.name, self.pose, size=(self.length,self.width,self.height))
        self.ensureCollisionUpdates(scene, self.name)

    def ensureCollisionUpdates(self, scene,box_name, timeout=4, box_is_attached=False, box_is_known=False):
        #ensure collision updates
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            #test if box is in attached objects
            attached_object = scene.get_attached_objects([box_name])
            is_attached = len(attached_object.keys()) > 0
            #test if box is in scene
            is_known = box_name in scene.get_known_object_names()
            #test if box is in expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def removeFromScene(self, scene):
        scene.remove_world_object(self.name)

