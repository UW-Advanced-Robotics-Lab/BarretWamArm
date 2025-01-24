#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def publish_goal(x, y, quat_z, quat_w):
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
    
    # Set orientation (quaternion)
    goal_msg.pose.orientation.x = 0.0
    goal_msg.pose.orientation.y = 0.0
    goal_msg.pose.orientation.z = quat_z
    goal_msg.pose.orientation.w = quat_w
    
    # Publish the message
    rospy.loginfo("Publishing goal: x={}, y={}, quat_z={}, quat_w={}".format(x, y, quat_z, quat_w))

    pub.publish(goal_msg)

if __name__ == "__main__":
    try:
        # Get inputs from the user
        x = float(input("Enter x position: "))
        y = float(input("Enter y position: "))
        quat_z = float(input("Enter quaternion z: "))
        quat_w = float(input("Enter quaternion w: "))
        
        publish_goal(x, y, quat_z, quat_w)
    except rospy.ROSInterruptException:
        pass
