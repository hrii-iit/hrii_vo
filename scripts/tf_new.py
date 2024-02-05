#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('virtual_tf_publisher')

    # Create a TransformStamped message
    transform_stamped = TransformStamped()

    # Set the frame IDs
    transform_stamped.header.frame_id = '/world'
    transform_stamped.child_frame_id = 'virtual_frame'

    # Set the translation
    transform_stamped.transform.translation.x = 1.0
    transform_stamped.transform.translation.y = 0.0
    transform_stamped.transform.translation.z = 0.0

    # Set the rotation (in quaternion)
    transform_stamped.transform.rotation.x = 0.0
    transform_stamped.transform.rotation.y = 0.0
    transform_stamped.transform.rotation.z = 0.0
    transform_stamped.transform.rotation.w = 1.0

    # Create a TransformBroadcaster
    broadcaster = tf2_ros.TransformBroadcaster()

    # Set the rate at which to broadcast the transform (e.g., 10 Hz)
    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():
        # Set the timestamp
        transform_stamped.header.stamp = rospy.Time.now()

        # Broadcast the transform
        broadcaster.sendTransform(transform_stamped)

        # Sleep to maintain the desired publishing rate
        rate.sleep()

