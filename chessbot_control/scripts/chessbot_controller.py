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

## joint states
INITIAL_JOINT = [1.59804368019104, -1.7011879126178187, 2.1022276878356934, -1.994666878377096, -1.6116936842547815, 0.11014064401388168]

## pose states
INITIAL_POSE = [-0.10892143140980294, 0.40291717033409835, 0.595788178594448, 0.03848213062389349, 0.6924433354702806, -0.020386870041952068, 0.7201567387141015]
POSE_1 = [-0.10831412374057263, 0.6179377787615377, 0.5954665412691271, 0.038885119981729194, 0.6920874345455502, -0.020793324251015887, 0.7204655217668348]
POSE_2 = [-0.10724553486632607, 0.6185854205342528, 0.48334377480236634, 0.03821593918314646, 0.6922769849688436, -0.020006288086220535, 0.7203414929821887]
POSE_3 = [-0.10768195441362702, 0.8185948485166824, 0.5964728327160349, 0.04053214653334825, 0.6920458447404095, -0.02220413658412674, 0.7203725912286543]
POSE_4 = [-0.10778283476270295, 0.8183326190854067, 0.4878744291042064, 0.03963797578901994, 0.6920207641014878, -0.021273235394878263, 0.7204745258394472]


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
  def __init__(self, group_name):
    super(ChessbotController, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('chessbot_controller', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface to one group of joints.  
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Setup gripper service
    rospy.wait_for_service("/rg2_gripper/control_width")
    self.set_gripper_width = rospy.ServiceProxy("/rg2_gripper/control_width", RG2, persistent=True)

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
    raw_input("============ Press ENTER to go to the initial state")
    # rospy.sleep(5)
    self.go_to_joint_state(INITIAL_JOINT)
    # print(self.group.get_current_joint_values())

  def go_to_joint_state(self, joint_goal):
    self.group.go(joint_goal, wait=True)
    self.group.stop()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, pose_goal_values):
    pose_goal = self.pose_from_state(pose_goal_values)
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

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


  def display_trajectory(self, plan):

    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):

    ## Use execute if you would like the robot to follow the plan that has already been computed:
    self.group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

  def toggle_gripper(self, grip=False):
    if grip:
      width = Float64(55)
    else:
      width = Float64(80)

    try:
      result = self.set_gripper_width(width)
      rospy.sleep(1)    # very very important, we wait for the robot to grip
    except rospy.ServiceException, e:
      print(e)

  @property
  def joint_state(self):
    return self.group.get_current_joint_values()

  @property
  def pose_state(self):
    pose = self.group.get_current_pose().pose
    pose_state = []
    pose_state.append(pose.position.x)
    pose_state.append(pose.position.y)
    pose_state.append(pose.position.z)
    pose_state.append(pose.orientation.x)
    pose_state.append(pose.orientation.y)
    pose_state.append(pose.orientation.z)
    pose_state.append(pose.orientation.w)
    return pose_state

  @staticmethod
  def pose_from_state(pose_state):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = pose_state[0]
    pose.position.y = pose_state[1]
    pose.position.z = pose_state[2]
    pose.orientation.x = pose_state[3]
    pose.orientation.y = pose_state[4]
    pose.orientation.z = pose_state[5]
    pose.orientation.w = pose_state[6]
    return pose


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'hand'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:
    cbc = ChessbotController("arm_group")

    raw_input("============ Press ENTER to state demo project")

############# go to CP 1
    ps0 = cbc.pose_from_state(INITIAL_POSE)
    ps1 = cbc.pose_from_state(POSE_1)
    ps2 = cbc.pose_from_state(POSE_2)

    waypoints = []
    waypoints.append(ps0)
    waypoints.append(ps1)
    waypoints.append(ps2)
    plan, fraction = cbc.plan_cartesian_path(waypoints)
    cbc.execute_plan(plan)

    # import ipdb 
    # ipdb.set_trace()

    cbc.toggle_gripper(True)

############# go to CP 2
    cbc.go_to_pose_goal(POSE_1)
    cbc.go_to_pose_goal(POSE_3)
    cbc.go_to_pose_goal(POSE_4)

    cbc.toggle_gripper()

############# go to Initial State
    cbc.go_to_pose_goal(POSE_3)
    cbc.go_to_pose_goal(INITIAL_POSE)
    
  
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()

