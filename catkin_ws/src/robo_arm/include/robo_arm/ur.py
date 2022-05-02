import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import math

class UR():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur_python_interface", anonymous=True)
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
        return self.group.get_current_pose().pose

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
        self.group.execute(plan, wait=True)