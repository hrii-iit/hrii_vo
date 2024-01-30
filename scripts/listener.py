import rospy
import tf
from geometry_msgs.msg import TransformStamped

def connect_tf_trees():
    rospy.init_node('tf_tree_connection')

    # Create a TF listener for each tree
    # listener_tree1 = tf.TransformListener(fixed_frame='base_link')
    # listener_tree2 = tf.TransformListener(fixed_frame='franka_a_franka_base_link')

    # Set the frames to connect
    frame_tree1 = 'map'
    frame_tree2 = 'franka_a_franka_base_link'

    # Create a TF broadcaster to publish the connection identity transform
    broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)  # Rate at which to publish the transform

    while not rospy.is_shutdown():
        try:
            # Get the latest transform from tree1's frame to tree2's frame
            #(trans, rot) = listener_tree1.lookupTransform(frame_tree2, frame_tree1, rospy.Time(0))
            rot = [0, 0, 0, 1]
            # Create a connection identity transform with no translation
            identity_trans = (0, 0, 0)
            identity_rot = (rot[0], rot[1], rot[2], rot[3])

            # Publish the connection identity transform
            broadcaster.sendTransform(identity_trans, identity_rot, rospy.Time.now(), frame_tree2, frame_tree1)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Error occurred while connecting TF trees')

        rate.sleep()

if __name__ == '__main__':
    try:
        connect_tf_trees()
    except rospy.ROSInterruptException:
        pass