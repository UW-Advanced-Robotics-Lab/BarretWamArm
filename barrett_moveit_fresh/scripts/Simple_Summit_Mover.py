#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import math

def publish_goal(x, y, angle):
    # Initialize the ROS node
    rospy.init_node('goal_publisher', anonymous=True)
    pub = rospy.Publisher('/uwarl/move_base_simple/goal', PoseStamped, queue_size=10)
    
    # Wait for publisher connection
    rospy.sleep(1)

    # Create PoseStamped message
    goal_msg = PoseStamped()
    goal_msg.header.frame_id = "uwarl_map"
    goal_msg.header.stamp = rospy.Time.now()
    
    # Set position
    goal_msg.pose.position.x = x
    goal_msg.pose.position.y = y
    goal_msg.pose.position.z = 0.0  # Assuming 2D movement
    
    # Convert angle to quaternion
    goal_msg.pose.orientation.x = 0.0
    goal_msg.pose.orientation.y = 0.0
    goal_msg.pose.orientation.z = math.sin(angle / 2.0)
    goal_msg.pose.orientation.w = math.cos(angle / 2.0)
    
    # Publish the message
    rospy.loginfo("Publishing goal: x={}, y={}, angle={} (radians)".format(x, y, angle))

    pub.publish(goal_msg)

if __name__ == "__main__":
    try:
        # Get inputs from the user
        x = float(input("Enter x position: "))
        y = float(input("Enter y position: "))
        angle = float(input("Enter angle (in radians): "))
        
        publish_goal(x, y, angle)
    except rospy.ROSInterruptException:
        pass
