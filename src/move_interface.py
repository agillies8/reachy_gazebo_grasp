#!/usr/bin/env python3  
import math
import time
import tf2_ros
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if the values in two lists are within a tolerance of each other.
  For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle 
  between the identical orientations q and -q is calculated correctly).
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    # Euclidean distance
    d = dist((x1, y1, z1), (x0, y0, z0))
    # phi = angle between orientations
    cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
    return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

  return True


class MoveGroupPythonInterfaceTutorial(object):
  """MoveGroupPythonInterfaceTutorial"""
  def __init__(self):

    super(MoveGroupPythonInterfaceTutorial, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "left_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

  def go_to_pose_goal(self, msg):

    #initialize cube position and looping rate
    rate = rospy.Rate(10.0)
    rate2 = rospy.Rate(1)
    cube_x = 0
    cube_y = 0
    cube_z = 0

    rate2.sleep()

    #attempts to get the latest transform of the cube from world
    try:
      trans = self.tfBuffer.lookup_transform('pedestal', 'cube2',rospy.Time(0))
      cube_x = trans.transform.translation.x
      cube_y = trans.transform.translation.y
      cube_z = trans.transform.translation.z
      # cube_r_x = trans.transform.rotation.x
      # cube_r_y = trans.transform.rotation.y
      # cube_r_z = trans.transform.rotation.z
      # cube_r_w = trans.transform.rotation.w
      q = quaternion_from_euler(math.radians(0), math.radians(-90), math.radians(-45))
      
      cube_r_x =  q[0]
      cube_r_y = q[1]
      cube_r_z = q[2]
      cube_r_w = q[3]

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rate.sleep()

    #prints the detected coordinates of the cube
    # print("x")
    # print(cube_x,cube_y,cube_z)
    # print("***************************************************************************")

    # rightEulerAngles = euler_from_quaternion([
    #         cube_r_x,
    #         cube_r_y,
    #         cube_r_z,
    #         cube_r_w])
    # r_ex = math.degrees(rightEulerAngles[0])
    # r_ey = math.degrees(rightEulerAngles[1])
    # r_ez = math.degrees(rightEulerAngles[2])
    
    print("-------------------------------------------------------------")
    # print(f"{r_ex},{r_ey},{r_ez}")
    
    # print("-------------------------------------------------------------")
    
    # print(cube_y)
    # print(cube_z)
# rostopic pub -1 /move_to_goal_topic/euler reachy_ros_moveit/move_to_goal_euler 
# "{ side: 'left', pos_x: 0.4, pos_y: 0.25, pos_z: 0.75, euler_x: 0.0, euler_y: -90.0, euler_z: -45.0, order: 'xyz' }"

    #loops thru the detected tags, and filters based on our pre-specificed tag (tag 2 in this case)
    #this also fliters out cube poses that are roughly outside the robot arm workspace
    for a in msg.detections:
      # print(a)
      if a.id[0] == 5:
      # and \
      #  0.0 < cube_x < 0.75 and \
      # 0.0 < cube_y < 0.75 and \
      # 0.3 < cube_z < 1.0 :
        #builds a pose goal based on the cube pose
        # pose_Goal = geometry_msgs.msg.PoseStamped()
        # frac, secs = math.modf(time.time())
        # pose_Goal.header.stamp.secs = secs
        # pose_Goal.header.stamp.nsecs = frac * pow(10, 9)
        # pose_Goal.pose.position.x = x
        # pose_Goal.pose.position.y = y
        # pose_Goal.pose.position.z = z
        # pose_Goal.pose.orientation.x = qx
        # pose_Goal.pose.orientation.y = qy
        # pose_Goal.pose.orientation.z = qz
        # pose_Goal.pose.orientation.w = qw

        pose_goal = geometry_msgs.msg.PoseStamped()
        # # frac, secs = math.modf(time.time())
        # # pose_goal.header.stamp.secs = secs
        # # pose_goal.header.stamp.nsecs = frac * pow(10, 9)
        pose_goal.pose.position.x = cube_x 
        pose_goal.pose.position.y = cube_y 
        pose_goal.pose.position.z = cube_z   + 0.15

        # # quat = quaternion_from_euler (0.0, -1.3,0.1)

        # pose_goal.orientation.x = quat[0]
        # pose_goal.orientation.y = quat[1]
        # pose_goal.orientation.z = quat[2]
        # pose_goal.orientation.w = quat[3]
        pose_goal.pose.orientation.x = cube_r_x
        pose_goal.pose.orientation.y = cube_r_y
        pose_goal.pose.orientation.z = cube_r_z
        pose_goal.pose.orientation.w = cube_r_w

        self.move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        print("Executing motion")
        # print(pose_goal)
        plan = self.move_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

#we start the node by initializing our move group interface object, and creating a subscriber that listens to the tag detections topic
def main():
  try:
    tutorial = MoveGroupPythonInterfaceTutorial()
    rospy.Subscriber('/tag_detections',
                     AprilTagDetectionArray,
                     tutorial.go_to_pose_goal)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
