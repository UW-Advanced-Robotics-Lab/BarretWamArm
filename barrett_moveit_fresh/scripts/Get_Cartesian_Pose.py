#!/usr/bin/env python
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from barrett_wam_msgs.srv import JointMove  # Correct service type import
from std_srvs.srv import Empty  # For the go_home service
import sys

def get_cartesian_pose(joint_positions):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("wam_fk_query", anonymous=True)

    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("Manipulator")

    # Set the joint target to the specified positions
    move_group.set_joint_value_target(joint_positions)

    # Plan to this joint configuration (no execution)
    plan = move_group.plan()

    if plan:
        # Extract the calculated pose from the final state of the plan
        end_effector_pose = move_group.get_current_pose().pose
        rospy.loginfo("Cartesian Pose at Joint Positions:")
        rospy.loginfo(end_effector_pose)
        return end_effector_pose
    else:
        rospy.logwarn("Failed to compute forward kinematics.")

    moveit_commander.roscpp_shutdown()

# Example usage for pick and place positions
if __name__ == "__main__":
    pick_positions = [0, 0.857, 0, 1.579, 0, 0.722, 0]
    place_positions = [0, -0.857, 0, -0.9, 0, -1.349, 0]

    rospy.loginfo("Pick position FK query:")
    get_cartesian_pose(pick_positions)

    rospy.loginfo("Place position FK query:")
    get_cartesian_pose(place_positions)
