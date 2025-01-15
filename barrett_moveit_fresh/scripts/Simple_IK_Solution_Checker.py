#!/usr/bin/env python

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from barrett_wam_msgs.srv import JointMove  # Correct service type import
import sys


def move_cartesian_path(x, y, z, qx, qy, qz, qw):
    """
    Move the WAM arm along a Cartesian path and check if the path is valid.
    Returns the final joint positions if valid, otherwise None.
    """
    # Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize a node for ROS
    rospy.init_node("wam_cartesian_path", anonymous=True)

    # Instantiate RobotCommander and PlanningSceneInterface
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Create a MoveGroupCommander for the "Manipulator" move group
    move_group = moveit_commander.MoveGroupCommander("Manipulator")

    # Set the reference frame for the target pose
    move_group.set_pose_reference_frame("wam_link_footprint")
    move_group.set_goal_tolerance(0.01)  # Position tolerance in meters
    move_group.set_goal_orientation_tolerance(0.05)  # Orientation tolerance in radians

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
        # Extract final joint positions from the trajectory
        joint_trajectory = plan.joint_trajectory
        final_joint_positions = joint_trajectory.points[-1].positions
        moveit_commander.roscpp_shutdown()
        return final_joint_positions
    else:
        moveit_commander.roscpp_shutdown()
        return None


def main():
    """
    Iterate through combinations of x and z while keeping y and orientation constant.
    Store and display all valid solutions.
    """
    rospy.loginfo("Starting Cartesian path search...")

    # Constant values for y and orientation
    y = 0.0
    qx, qy, qz, qw = 0.0, 0.0, 1, 0.0

    # Prompt user for step size
    while True:
        try:
            step_size = float(input("Enter the step size (0 < step_size <= 1): "))
            if 0 < step_size <= 1:
                break
            else:
                print("Step size must be greater than 0 and less than or equal to 1.")
        except ValueError:
            print("Invalid input. Please enter a numeric value.")

    # Define ranges for x and z dynamically based on the step size
    x_range = [round(i, 4) for i in frange(0, 1, step_size)]
    z_range = [round(i, 4) for i in frange(0, 1, step_size)]

    valid_solutions = []  # To store all valid x, z combinations and joint positions

    # Iterate through all combinations of x and z
    for x in x_range:
        for z in z_range:
            rospy.loginfo(f"Checking pose: x={x}, y={y}, z={z}, qx={qx}, qy={qy}, qz={qz}, qw={qw}")
            joint_positions = move_cartesian_path(x, y, z, qx, qy, qz, qw)
            if joint_positions:
                rospy.loginfo(f"Valid solution found at x={x}, z={z}")
                valid_solutions.append((x, z, joint_positions))

    # Display all valid solutions
    rospy.loginfo("All valid solutions:")
    for solution in valid_solutions:
        x, z, joint_positions = solution
        rospy.loginfo(f"x={x}, z={z}, joint_positions={joint_positions}")

    rospy.loginfo("Search complete.")


def frange(start, stop, step):
    """
    Generate a range of floating-point numbers.
    """
    while start <= stop:
        yield round(start, 8)
        start += step


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Script interrupted. Exiting...")
    except KeyboardInterrupt:
        rospy.loginfo("Script terminated by user.")
