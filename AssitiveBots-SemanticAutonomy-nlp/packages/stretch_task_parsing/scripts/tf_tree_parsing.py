#!/usr/bin/env python
import rospy
import tf2_msgs.msg

def tf_callback(tf_message):
    for transform in tf_message.transforms:
        parent = transform.header.frame_id
        child = transform.child_frame_id

        if parent == target_frame or child == target_frame:
            connected_frames.add(parent if parent != target_frame else child)

if __name__ == '__main__':
    target_frame = 'map'  # Replace with the target frame
    connected_frames = set()

    rospy.init_node('direct_tf_connections')
    rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, tf_callback)

    rospy.spin()  # Keep the node running

    # At this point, you can process the connected_frames set as needed
    # Note: This node will continuously update connected_frames set as long as it's running
    rospy.loginfo("All frames connected to map frame: {}".format(connected_frames))
    