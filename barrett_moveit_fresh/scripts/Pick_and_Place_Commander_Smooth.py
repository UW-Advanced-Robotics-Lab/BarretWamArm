#!/usr/bin/env python

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from barrett_wam_msgs.srv import JointMove  # Correct service type import
from std_srvs.srv import Empty  # For the go_home service
import sys

def call_empty_service(service_name):
    rospy.loginfo(f"Calling {service_name} service...")

    # Wait for the service to become available
    rospy.wait_for_service(service_name)
    try:
        # Create a service proxy
        empty_service = rospy.ServiceProxy(service_name, Empty)

        # Call the service
        empty_service()
        rospy.loginfo(f"Service {service_name} called successfully.")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to {service_name} failed: {e}")

def move_cartesian_path(x, y, z, qx, qy, qz, qw):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("wam_cartesian_path", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("Manipulator")

    move_group.set_pose_reference_frame("wam_link_base")
    end_effector_link = move_group.get_end_effector_link()
    rospy.loginfo(f"End effector link: {end_effector_link}")
    move_group.set_goal_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.05)
    move_group.set_planning_time(20.0)  # Increase the planning time to 10 seconds
    move_group.set_num_planning_attempts(100)  # Increase the number of planning attempts



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

    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, False
    )

    if fraction == 1.0:
        rospy.loginfo("Successfully planned the Cartesian path.")
        joint_trajectory = plan.joint_trajectory
        final_joint_positions = joint_trajectory.points[-1].positions
        rospy.loginfo("Calculated final joint positions (without execution):")
        rospy.loginfo(final_joint_positions)
        call_joint_move_service(final_joint_positions)
    else:
        rospy.logwarn(
            "Could not compute a complete Cartesian path. Fraction achieved: %f", fraction
        )

    moveit_commander.roscpp_shutdown()

def call_joint_move_service(joint_positions):
    rospy.loginfo("Calling /wam/joint_move service...")
    rospy.wait_for_service("/wam/joint_move")
    try:
        joint_move_service = rospy.ServiceProxy("/wam/joint_move", JointMove)
        response = joint_move_service(joint_positions)
        rospy.loginfo("Service response: %s", response)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def move_to_joint_positions(joint_positions):
    rospy.loginfo("Sending joint positions to /wam/joint_move service...")
    call_joint_move_service(joint_positions)

def go_home():
    call_empty_service("/wam/go_home")

def move_to_pick():
    """Move the WAM arm to the 'pick' position."""
    rospy.loginfo("Moving to 'pick' position...")
    pick_positions = [0, 0.857, 0, 1.579, 0, 0.722, 0]
    move_to_joint_positions(pick_positions)

def move_to_place():
    """Move the WAM arm to the 'place' position."""
    rospy.loginfo("Moving to 'place' position...")
    place_positions = [0, -0.857, 0, -0.9, 0, -1.349, 0]
    move_to_joint_positions(place_positions)

if __name__ == "__main__":
    try:
        while not rospy.is_shutdown():
            user_input = input(
                "Enter 'p' for Cartesian pose, 'j' for joint positions, 'h' to go home, 'pick', 'place', or 'q' to quit: "
            )

            if user_input.lower() == "q":
                rospy.loginfo("Exiting...")
                break

            try:
                if user_input.strip() == "pick":
                    move_to_pick()

                elif user_input.strip() == "place":
                    move_to_place()

                elif user_input.startswith("p"):
                    _, x, y, z, qx, qy, qz, qw = user_input.split()
                    x, y, z, qx, qy, qz, qw = map(float, [x, y, z, qx, qy, qz, qw])
                    move_cartesian_path(x, y, z, qx, qy, qz, qw)

                elif user_input.startswith("j"):
                    _, j1, j2, j3, j4, j5, j6, j7 = user_input.split()
                    joint_positions = list(map(float, [j1, j2, j3, j4, j5, j6, j7]))
                    move_to_joint_positions(joint_positions)

                elif user_input.strip() == "h":
                    go_home()

                elif user_input.strip() == "og":
                    call_empty_service("/wam/bhand/open_grasp")

                elif user_input.strip() == "os":
                    call_empty_service("/wam/bhand/open_spread")

                elif user_input.strip() == "cg":
                    call_empty_service("/wam/bhand/close_grasp")

                elif user_input.strip() == "cs":
                    call_empty_service("/wam/bhand/close_spread")

                else:
                    rospy.logwarn(
                        "Invalid input format. Please start with 'p', 'j', 'h', 'og', 'os', 'cg', 'cs', 'pick', or 'place'."
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
