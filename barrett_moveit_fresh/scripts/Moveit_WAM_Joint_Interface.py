#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

class JointStateCombiner:
    def __init__(self):
        # Initialize the node
        rospy.init_node('joint_state_combiner', anonymous=True)

        # Subscribers to WAM and BHAND joint states
        self.wam_joint_states = JointState()
        self.bhand_joint_states = JointState()
        rospy.Subscriber('/wam/joint_states', JointState, self.wam_joint_states_callback)
        rospy.Subscriber('/wam/bhand/joint_states', JointState, self.bhand_joint_states_callback)

        # Publisher for combined joint states
        self.combined_joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def wam_joint_states_callback(self, msg):
        self.wam_joint_states = msg
        self.publish_combined_joint_states()

    def bhand_joint_states_callback(self, msg):
        self.bhand_joint_states = msg
        self.publish_combined_joint_states()

    def publish_combined_joint_states(self):
        # Only publish if we have both WAM and BHAND joint states
        if len(self.wam_joint_states.position) > 0 and len(self.bhand_joint_states.position) > 0:
            # Mapping the WAM and BHAND joint names to MoveIt names
            wam_name_map = {
                'wam_j1': 'wam_joint_1',
                'wam_j2': 'wam_joint_2',
                'wam_j3': 'wam_joint_3',
                'wam_j4': 'wam_joint_4',
                'wam_j5': 'wam_joint_5',
                'wam_j6': 'wam_joint_6',
                'wam_j7': 'wam_joint_7'
            }

            bhand_name_map = {
                'inner_f1': 'bhand_finger1',
                'inner_f2': 'bhand_finger2',
                'inner_f3': 'bhand_finger3',
                'spread': 'bhand_spread',
                'outer_f1': 'bhand_finger1_joint_3',
                'outer_f2': 'bhand_finger2_joint_3',
                'outer_f3': 'bhand_finger3_joint_3'
            }

            # Convert WAM joint names to MoveIt joint names
            wam_names_moved = [wam_name_map.get(name, name) for name in self.wam_joint_states.name]
            
            # Convert BHAND joint names to MoveIt joint names
            bhand_names_moved = [bhand_name_map.get(name, name) for name in self.bhand_joint_states.name]

            # Combine the WAM and BHAND names and positions
            combined_names = wam_names_moved + bhand_names_moved
            combined_positions = self.wam_joint_states.position + self.bhand_joint_states.position

            # Create the combined message
            combined_msg = JointState()
            combined_msg.header.stamp = rospy.Time.now()  # Ensure a unique and increasing timestamp
            combined_msg.name = combined_names
            combined_msg.position = combined_positions

            # Publish the combined joint states
            try:
                self.combined_joint_states_pub.publish(combined_msg)
                rospy.loginfo("Published combined joint states.")
            except rospy.ROSException as e:
                rospy.logerr(f"Failed to publish combined joint states: {e}")

if __name__ == '__main__':
    try:
        combiner = JointStateCombiner()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Joint State Combiner node terminated.")
