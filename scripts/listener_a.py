import rospy
import tf

rospy.init_node('tf_listener_node')  # Initialize the ROS node
listener = tf.TransformListener()  # Create a TransformListener object

rate = rospy.Rate(10)  # Set the loop rate (10 Hz in this example)

while not rospy.is_shutdown():
    try:
        # Wait for the TF transformation to become available
        listener.waitForTransform('/franka_a_franka_base_link', '/franka_a_franka_EE', rospy.Time(), rospy.Duration(1.0))
        
        # Get the latest transform between the two frames
        (trans, rot) = listener.lookupTransform('/franka_a_franka_base_link', '/franka_a_franka_EE', rospy.Time(0))
        
        # Print the transformation values
        print("Translation: ", trans)
        print("Rotation: ", rot)
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Failed to get the TF transform")
    
    rate.sleep()  # Sleep to maintain the loop rate

