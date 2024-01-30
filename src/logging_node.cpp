#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#include <eigen_conversions/eigen_msg.h>


namespace vo {

class LoggingNode
{
  public:
    LoggingNode(ros::NodeHandle& nh) : nh_(nh)
    {}

    bool init()
    {
      // Get namespace
      std::string namespace_str;
      namespace_str = ros::this_node::getNamespace();
      namespace_str = namespace_str.substr(1, namespace_str.size());

      // Initialize matlogger
      std::string logging_path_str;
      // std::string logPath;
      // const char * homePath = ::getenv("HOME");
      logging_path_str.append(::getenv("HOME"));
      logging_path_str.append("/log/");
      logging_path_str.append(namespace_str);
      logging_path_str.append("_vo_comparison_log");

      logger_ = XBot::MatLogger2::MakeLogger(logging_path_str);
      appender_ = XBot::MatAppender::MakeInstance();
      appender_->add_logger(logger_);
      appender_->start_flush_thread();

      robot_state_sub_ = nh_.subscribe("cartesian_pose_example_controller/O_T_EE", 1000, &LoggingNode::robotStateCallback, this);
      rgbd_odom_sub_ = nh_.subscribe("/franka_a/cartesian_impedance_controller/equilibrium_pose", 1000, &LoggingNode::rgbdOdomCallback, this);
      zed_odom_sub_ = nh_.subscribe("/zed2i_odom", 1000, &LoggingNode::zed2iOdomCallback, this);

      return true;
    }

    bool logData()
    {
      logger_->add("EE_position", robot_position_);
      logger_->add("EE_orientation", robot_orientation_.coeffs());

      logger_->add("rgbd_odom_position", rgbd_odom_position_);
      logger_->add("rgbd_odom_orientation", rgbd_odom_orientation_.coeffs());

      logger_->add("zed_odom_position", zed_odom_position_);
      logger_->add("zed_odom_orientation", zed_odom_orientation_.coeffs());

      logger_->add("time", ros::Time::now().toSec());
      return true;
    }

  private:
    ros::NodeHandle nh_;
    Eigen::Vector3d robot_position_, rgbd_odom_position_, zed_odom_position_;
    Eigen::Quaterniond robot_orientation_, rgbd_odom_orientation_, zed_odom_orientation_;
    ros::Subscriber robot_state_sub_;
    ros::Subscriber rgbd_odom_sub_;
    ros::Subscriber zed_odom_sub_;

    // Create matlogger
    XBot::MatLogger2::Ptr logger_;
    XBot::MatAppender::Ptr appender_;

    void robotStateCallback(const geometry_msgs::Pose::ConstPtr& msg)
    {
      ROS_INFO_ONCE("Robot state received.");
      tf::pointMsgToEigen(msg->position, robot_position_);
      tf::quaternionMsgToEigen(msg->orientation, robot_orientation_);
    }

    void rgbdOdomCallback(const geometry_msgs::Pose::ConstPtr& msg)
    {
      ROS_INFO_ONCE("RGBD odom received.");
      tf::pointMsgToEigen(msg->position, rgbd_odom_position_);
      tf::quaternionMsgToEigen(msg->orientation, rgbd_odom_orientation_);
    }

    void zed2iOdomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
      ROS_INFO_ONCE("Zed odom received.");
      tf::pointMsgToEigen(msg->pose.position, zed_odom_position_);
      tf::quaternionMsgToEigen(msg->pose.orientation, zed_odom_orientation_);
    }

}; // class LoggingNode

} // namespace ov

int main(int argc, char **argv)
{
  ros::init(argc, argv, "logging_node");
  ros::NodeHandle nh;

  vo::LoggingNode logging_node(nh);

  if (!logging_node.init())
  {
    ROS_ERROR("Logging node initialization failed.");
    return -1;
  }

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    logging_node.logData();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
