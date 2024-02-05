import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
import tf

# Initialize global variables
first = True
tf_listener = None
trans_p = None
rot_p = None
An = None

global parent_frame
global child_frame

parent_frame = '/world'
child_frame = '/skeleton_right_hand'

# Function to visualize the coordinate system
def visualize_coordinate_system(ax, euler_angles):
    # Convert Euler angles to rotation matrix
    rotation_matrix = Rotation.from_euler('XYZ', euler_angles, degrees=True).as_matrix()

    # Extract unit vectors for each axis
    x_axis = rotation_matrix[:, 0]
    y_axis = rotation_matrix[:, 1]
    z_axis = rotation_matrix[:, 2]

    # Clear the plot
    ax.cla()

    # Plot the coordinate system
    ax.quiver(0, 0, 0, x_axis[0], x_axis[1], x_axis[2], color='red', label='X-axis')
    ax.quiver(0, 0, 0, y_axis[0], y_axis[1], y_axis[2], color='green', label='Y-axis')
    ax.quiver(0, 0, 0, z_axis[0], z_axis[1], z_axis[2], color='blue', label='Z-axis')

    # Set axis labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Set equal aspect ratio
    max_range = np.array([x_axis, y_axis, z_axis]).max()
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range, max_range])

    # Add a legend
    ax.legend()

# Function to convert quaternion to Euler angles
def euler_from_quaternion(q):
    return np.rad2deg(euler_from_quaternion([q.x, q.y, q.z, q.w]))

# Function to convert Euler angles to rotation matrix
def euler_to_rotation_matrix(euler_angles):
    return Rotation.from_euler('XYZ', euler_angles, degrees=True).as_matrix()

def main():
    global tf_listener
    global parent_frame
    global child_frame
    global first 
    global trans_p
    global rot_p   
    global An

    rospy.init_node('tf_listener', anonymous=True)

    parent_frame = '/world'
    child_frame = '/skeleton_right_hand'

    tf_listener = tf.TransformListener()

    if first:
        # (trans_p, rot_p) = tf_listener.lookupTransform('/world', '/skeleton_right_hand', rospy.Time(0))
        # An = np.array(trans_p)
        first = False

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = tf_listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))

            # Calculate the difference in position
            # PP = PoseStamped()
            # PP.pose.position.x = trans[0] - trans_p[0]
            # PP.pose.position.y = trans[1] - trans_p[1]
            # PP.pose.position.z = trans[2] - trans_p[2]

            # Print Euler angles
            eu = euler_from_quaternion(rot)
            print(f"The Euler angles are: {eu}")

            # Visualize the coordinate system
            visualize_coordinate_system(ax, eu)

            plt.pause(0.01)

            rate.sleep()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
