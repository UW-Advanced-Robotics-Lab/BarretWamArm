#!/usr/bin/env python

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from barrett_wam_msgs.srv import JointMove  # Correct service type import
from std_srvs.srv import Empty  # For the go_home service
import sys


def move_cartesian_path(x, y, z, qx, qy, qz, qw):
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
    move_group.set_pose_reference_frame("wam_link_base")

    end_effector_link = move_group.get_end_effector_link()
    rospy.loginfo(f"End effector link: {end_effector_link}")
    
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
        rospy.loginfo("Successfully planned the Cartesian path.")

        # Extract the final joint positions from the planned trajectory
        joint_trajectory = plan.joint_trajectory
        final_joint_positions = joint_trajectory.points[-1].positions

        rospy.loginfo("Calculated final joint positions (without execution):")
        rospy.loginfo(final_joint_positions)

        # Call the /wam/joint_move service with final joint positions
        call_joint_move_service(final_joint_positions)

    else:
        rospy.logwarn(
            "Could not compute a complete Cartesian path. Fraction achieved: %f", fraction
        )

    # Shut down moveit_commander
    moveit_commander.roscpp_shutdown()


def call_joint_move_service(joint_positions):
    rospy.loginfo("Calling /wam/joint_move service...")

    # Wait for the service to become available
    rospy.wait_for_service("/wam/joint_move")
    try:
        # Create a service proxy for /wam/joint_move
        joint_move_service = rospy.ServiceProxy("/wam/joint_move", JointMove)

        # Prepare and send the request
        response = joint_move_service(joint_positions)
        rospy.loginfo("Service response: %s", response)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


def move_to_joint_positions(joint_positions):
    rospy.loginfo("Sending joint positions to /wam/joint_move service...")
    call_joint_move_service(joint_positions)


def go_home():
    rospy.loginfo("Calling /wam/go_home service...")
    rospy.wait_for_service("/wam/go_home")
    try:
        # Create a service proxy for /wam/go_home
        go_home_service = rospy.ServiceProxy("/wam/go_home", Empty)

        # Call the service
        go_home_service()
        rospy.loginfo("WAM arm moved to home position.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to /wam/go_home failed: %s", e)


if __name__ == "__main__":
    try:
        while not rospy.is_shutdown():
            # Input position, joint angles, or go home command
            user_input = input(
                "Enter 'p' for Cartesian pose, 'j' for joint positions, or 'h' to go home. Type 'exit' to quit: "
            )

            if user_input.lower() == "exit":
                rospy.loginfo("Exiting...")
                break

            try:
                # Handle Cartesian input (p)
                if user_input.startswith("p"):
                    _, x, y, z, qx, qy, qz, qw = user_input.split()
                    x, y, z, qx, qy, qz, qw = map(float, [x, y, z, qx, qy, qz, qw])
                    move_cartesian_path(x, y, z, qx, qy, qz, qw)

                # Handle joint position input (j)
                elif user_input.startswith("j"):
                    _, j1, j2, j3, j4, j5, j6, j7 = user_input.split()
                    joint_positions = list(map(float, [j1, j2, j3, j4, j5, j6, j7]))
                    move_to_joint_positions(joint_positions)

                # Handle go home command (h)
                elif user_input.strip() == "h":
                    go_home()

                else:
                    rospy.logwarn(
                        "Invalid input format. Please start with 'p', 'j', or 'h'."
                    )

            except ValueError:
                rospy.logwarn(
                    "Invalid input format. Please ensure numbers are provided in the correct format."
                )
            except rospy.ROSInterruptException:
                rospy.loginfo("Interrupted. Exiting...")
                break
    except KeyboardInterrupt:
        rospy.loginfo("Script terminated by user.")
