#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Float64
from moveit_commander.conversions import pose_to_list
from chessbot_control.srv import RG2

# joint states
INITIAL_JOINT = [1.667920708656311, -1.9511421362506312, 2.4023525714874268, -
                 1.9895995298968714, -1.6048291365252894, 0]

# pose states
INITIAL_POSE = [-0.12655840598254664, 0.2699007394698777, 0.5456780779855925,
                0.7199482512298049, -0.013085499664068176, 0.6938641361462244, -0.00747300607127187]
POSE_1 = [-0.10831412374057263, 0.6179377787615377, 0.5954665412691271,
          0.7201567387141015, 0.03848213062389349, 0.6924433354702806, -0.020386870041952068]
POSE_2 = [-0.10724553486632607, 0.6185854205342528, 0.48334377480236634,
          0.7201567387141015, 0.03848213062389349, 0.6924433354702806, -0.020386870041952068]
POSE_3 = [-0.10768195441362702, 0.8185948485166824, 0.5964728327160349,
          0.7201567387141015, 0.03848213062389349, 0.6924433354702806, -0.020386870041952068]
POSE_4 = [-0.10778283476270295, 0.8183326190854067, 0.4878744291042064,
          0.7201567387141015, 0.03848213062389349, 0.6924433354702806, -0.020386870041952068]


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class ChessbotController(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(ChessbotController, self).__init__()

        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('chessbot_controller', anonymous=True)

        # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface to one group of joints.
        group = moveit_commander.MoveGroupCommander("arm_group")

        # We create a `DisplayTrajectory`_ publisher which is used later to publish trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # Setup gripper service
        rospy.wait_for_service("/rg2_gripper/control_width")
        self.set_gripper_width = rospy.ServiceProxy(
            "/rg2_gripper/control_width", RG2, persistent=True)

        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the robot
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        # Go to the initial state
        # raw_input("============ Press ENTER to go to the initial state")
        # rospy.sleep(5)
        # self.goto_joint_goal(INITIAL_JOINT)
        # print(self.group.get_current_joint_values())

    def goto_joint_goal(self, joint_goal):
        self.group.go(joint_goal, wait=True)
        self.group.stop()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def goto_pose_goal(self, pose_goal):
        pose_goal = self.pose_from_state(pose_goal)
        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, waypoints):
        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        return plan, fraction

    def display_trajectory(self, plan):

        # You can ask RViz to visualize a plan (aka trajectory) for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again):
        ##
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        self.group.execute(plan, wait=True)

    def grip(self):
        self.set_gripper_width(Float64(55))
        rospy.sleep(1)

    def release(self):
        self.set_gripper_width(Float64(80))
        rospy.sleep(1)

    @property
    def joint_state(self):
        return self.group.get_current_joint_values()

    @property
    def pose_state(self):
        pose = self.group.get_current_pose().pose
        state = []
        state.append(pose.position.x)
        state.append(pose.position.y)
        state.append(pose.position.z)
        state.append(pose.orientation.w)
        state.append(pose.orientation.x)
        state.append(pose.orientation.y)
        state.append(pose.orientation.z)
        return state

    @staticmethod
    def pose_from_state(state):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = state[0]
        pose.position.y = state[1]
        pose.position.z = state[2]
        pose.orientation.w = state[3]
        pose.orientation.x = state[4]
        pose.orientation.y = state[5]
        pose.orientation.z = state[6]
        return pose
