#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

# Joint name mappings
wam_joint_mapping = {
    "wam_j1": "wam_joint_1",
    "wam_j2": "wam_joint_2",
    "wam_j3": "wam_joint_3",
    "wam_j4": "wam_joint_4",
    "wam_j5": "wam_joint_5",
    "wam_j6": "wam_joint_6",
    "wam_j7": "wam_joint_7"
}

bhand_joint_mapping = {
    "spread": "bhand_spread",
    "inner_f1": "bhand_finger1",
    "outer_f1": "bhand_finger1_joint_3",
    "inner_f2": "bhand_finger2",
    "outer_f2": "bhand_finger2_joint_3",
    "inner_f3": "bhand_finger3",
    "outer_f3": "bhand_finger3_joint_3"
}

# Global storage for joint states
current_wam_state = JointState()
current_bhand_state = JointState()

# Publisher for remapped joint states
pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

# Remap and store WAM joint states
def wam_joint_state_callback(msg):
    global current_wam_state
    current_wam_state = remap_joint_states(msg, wam_joint_mapping)
    publish_combined_joint_states()

# Remap and store BHAND joint states
def bhand_joint_state_callback(msg):
    global current_bhand_state
    current_bhand_state = remap_joint_states(msg, bhand_joint_mapping)
    publish_combined_joint_states()

# Remap joint names based on provided mapping
def remap_joint_states(msg, mapping):
    remapped_msg = JointState()
    remapped_msg.header = msg.header
    
    for i, name in enumerate(msg.name):
        remapped_name = mapping.get(name, name)  # Use mapped name or pass through
        remapped_msg.name.append(remapped_name)
        remapped_msg.position.append(msg.position[i])
        if len(msg.velocity) > i:
            remapped_msg.velocity.append(msg.velocity[i])
        if len(msg.effort) > i:
            remapped_msg.effort.append(msg.effort[i])

    return remapped_msg

# Publish a single combined message with WAM and BHAND states
def publish_combined_joint_states():
    combined_msg = JointState()
    combined_msg.header.stamp = rospy.Time.now()  # Use current timestamp
    
    # Combine WAM and BHAND joint states
    for state in [current_wam_state, current_bhand_state]:
        combined_msg.name.extend(state.name)
        combined_msg.position.extend(state.position)
        combined_msg.velocity.extend(state.velocity)
        combined_msg.effort.extend(state.effort)

    pub.publish(combined_msg)

if __name__ == "__main__":
    rospy.init_node("joint_state_remapper")
    rospy.Subscriber("/wam/joint_states", JointState, wam_joint_state_callback)
    rospy.Subscriber("/wam/bhand/joint_states", JointState, bhand_joint_state_callback)
    rospy.spin()
