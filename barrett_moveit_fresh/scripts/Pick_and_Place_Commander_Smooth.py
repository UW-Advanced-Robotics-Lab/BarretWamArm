#!/usr/bin/env python

import rospy
import moveit_commander
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

def move_cartesian_path_with_joint_waypoints(joint_waypoints):
    rospy.init_node("wam_cartesian_path_with_joint_waypoints", anonymous=True)

    for joint_positions in joint_waypoints:
        rospy.loginfo(f"Moving to joint waypoint: {joint_positions}")
        call_joint_move_service(joint_positions)

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

def move_to_place_with_joint_waypoints():
    """Move the WAM arm to the 'place' position using joint waypoints."""
    rospy.loginfo("Moving to 'place' position using joint waypoints...")

    joint_waypoints = [
        [0, 0.857, 0, 1.579, 0, 0.722, 0],
        [0.2, 0.6, 0.1, 1.2, 0.1, 0.8, 0.1],
        [0.4, 0.3, 0.2, 0.9, 0.2, 0.6, 0.2],
        [0.6, 0.0, 0.3, 0.7, 0.3, 0.4, 0.3],
        [0, -0.857, 0, -0.9, 0, -1.349, 0]
    ]

    move_cartesian_path_with_joint_waypoints(joint_waypoints)

if __name__ == "__main__":
    try:
        while not rospy.is_shutdown():
            user_input = input(
                "Enter 'pick', 'place', or 'q' to quit: "
            )

            if user_input.lower() == "q":
                rospy.loginfo("Exiting...")
                break

            try:
                if user_input.strip() == "pick":
                    move_to_pick()

                elif user_input.strip() == "place":
                    move_to_place_with_joint_waypoints()

                else:
                    rospy.logwarn(
                        "Invalid input format. Please enter 'pick', 'place', or 'q'."
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
