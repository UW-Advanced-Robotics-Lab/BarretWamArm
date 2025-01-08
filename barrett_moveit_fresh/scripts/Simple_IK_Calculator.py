#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
import sys

def move_to_pose(x, y, z, qx, qy, qz, qw):
    # Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize a node for ROS
    rospy.init_node('wam_moveit_node', anonymous=True)

    # Instantiate RobotCommander and PlanningSceneInterface
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Create a MoveGroupCommander for the "Manipulator" move group
    move_group = moveit_commander.MoveGroupCommander("Manipulator")

    # Set the reference frame for the target pose (usually the world frame)
    move_group.set_pose_reference_frame("world")

    # Create a Pose object to define the target pose
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z

    # Set the quaternion orientation
    target_pose.orientation.x = qx
    target_pose.orientation.y = qy
    target_pose.orientation.z = qz
    target_pose.orientation.w = qw

    # Set the target pose for the move group
    move_group.set_pose_target(target_pose)

    # Plan and execute the motion to the target pose
    success = move_group.go(wait=True)

    if success:
        rospy.loginfo("Successfully moved to target pose.")
    else:
        rospy.logwarn("Failed to move to target pose.")

    # Stop the robot after the motion is completed
    move_group.stop()

    # Clear the targets
    move_group.clear_pose_targets()

    # Shut down moveit_commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        # Example target pose (XYZ and Quaternion)
        x = 0.5
        y = 0.2
        z = 0.3
        qx, qy, qz, qw = 0.0, 0.0, 0.7071, 0.7071  # Example quaternion (no rotation)

        move_to_pose(x, y, z, qx, qy, qz, qw)
    except rospy.ROSInterruptException:
        pass
