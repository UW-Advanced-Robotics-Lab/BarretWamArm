#!/usr/bin/env python

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import sys

def move_cartesian_path(x, y, z, qx, qy, qz, qw):
    # Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize a node for ROS
    rospy.init_node('wam_cartesian_path', anonymous=True)

    # Instantiate RobotCommander and PlanningSceneInterface
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Create a MoveGroupCommander for the "Manipulator" move group
    move_group = moveit_commander.MoveGroupCommander("Manipulator")

    # Set the reference frame for the target pose
    move_group.set_pose_reference_frame("wam_link_base")

    end_effector_link = move_group.get_end_effector_link()
    rospy.loginfo(f"End effector link: {end_effector_link}")

    # Define waypoints for the Cartesian path
    waypoints = []
    
    pose_goal = Pose()
    
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    pose_goal.orientation.x = qx
    pose_goal.orientation.y = qy
    pose_goal.orientation.z = qz
    pose_goal.orientation.w = qw

    waypoints.append(pose_goal)

    # Compute Cartesian path
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,  # List of waypoints
        0.01,       # End-effector step size
        False       # Jump threshold
    )

    if fraction == 1.0:
        rospy.loginfo("Successfully planned the Cartesian path.")

        # Extract the final joint positions from the planned trajectory
        joint_trajectory = plan.joint_trajectory
        final_joint_positions = joint_trajectory.points[-1].positions

        rospy.loginfo("Calculated final joint positions (without execution):")
        rospy.loginfo(final_joint_positions)
    else:
        rospy.logwarn("Could not compute a complete Cartesian path. Fraction achieved: %f", fraction)

    # Shut down moveit_commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        # Example target pose (XYZ and Quaternion)
        x = 0.3
        y = 0.0
        z = 0.7
        qx, qy, qz, qw = 0.7071, 0.0, 0, 0.7071  # Example quaternion (no rotation)

        move_cartesian_path(x, y, z, qx, qy, qz, qw)
    except rospy.ROSInterruptException:
        pass
