#!/usr/bin/env python3

import json
import rospkg
from rospy_message_converter import json_message_converter

import sys
import rospy


import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

def callback(data):

    rospy.loginfo("I heard %s", data)

    json_msg = json_message_converter.convert_ros_message_to_json(data)
    rospack = rospkg.RosPack()

    path = rospack.get_path('reachy_gazebo_grasp')

    with open( path + "/data/data_file.json", "w") as write_file:
        json.dump(json_msg, write_file, indent=4)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('traj_goal_listener', anonymous=True)

    rospy.Subscriber("/left_arm_position_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()