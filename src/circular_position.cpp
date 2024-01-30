/*
 * Author: Hamidreza Raei
 * email: hamidreza.raei@iit.it
 *
 * Finite State Machine template
 *
*/

#include "ros/ros.h"
#include <eigen_conversions/eigen_msg.h>
#include <hrii_utils/RosHelper.h>
#include <actionlib/client/simple_action_client.h>
#include "hrii_robot_msgs/FifthOrderPolyAction.h"
#include "hrii_utils/MessageDisplay.h"

#include "hrii_gri_interface/SetGripperData.h"
#include "hrii_utils/ros/GeometryMsgOperations.h"
#include "cmath"
#include "math.h"


int num_points = 30;


// FSM states declaration
enum class fsm_states {HOMING, CENTER, CIRCULAR,MOVE_LEFT, MOVE_RIGHT, MOVE_UP, ERROR, EXIT};

typedef actionlib::SimpleActionClient<hrii_robot_msgs::FifthOrderPolyAction> FifthOrderPolyAction;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_fsm");
  ros::NodeHandle nh;

  HRII_Utils::MessageDisplay display;
  display.setName(ros::this_node::getName());

  // Initialize service clients
  std::shared_ptr<FifthOrderPolyAction> traj_ac = std::make_shared<FifthOrderPolyAction>("/franka_set_position_action", true);
  // ros::ServiceClient gripper_open_srv = nh.serviceClient<hrii_gri_interface::SetGripperData>("gripper_srv_node/open");
  // ros::ServiceClient gripper_close_srv = nh.serviceClient<hrii_gri_interface::SetGripperData>("gripper_srv_node/close");
  // ros::ServiceClient gripper_grasp_from_inside_srv = nh.serviceClient<hrii_gri_interface::SetGripperData>("gripper_srv_node/grasp_from_inside");
  // ros::ServiceClient gripper_grasp_from_outside_srv = nh.serviceClient<hrii_gri_interface::SetGripperData>("gripper_srv_node/grasp_from_outside");

  auto moveToTargetPose = [&traj_ac, &display](const geometry_msgs::Pose &target_pose, const double& execution_time, const std_msgs::Float32MultiArray init_vel,
                      const std_msgs::Float32MultiArray fin_vel, const std_msgs::Float32MultiArray init_acc,
                      const std_msgs::Float32MultiArray fin_acc) {
    // Fill in the service request to be sent to the trajectory planner
    // specifying either the desired speed OR the execution time, set the other one to 0.0

    hrii_robot_msgs::FifthOrderPolyActionGoal traj_goal;
    traj_goal.header.stamp = ros::Time::now();
    traj_goal.goal.target_pose = target_pose;
    traj_goal.goal.execution_time = execution_time;
    traj_goal.goal.initial_velocity = init_vel;
    traj_goal.goal.final_velocity = fin_vel;
    traj_goal.goal.initial_acceleration = init_acc;
    traj_goal.goal.final_acceleration = fin_acc;


    std::vector<geometry_msgs::Pose> waypoints;
    // Send a request to the trajectory planner service
    traj_ac->sendGoal(traj_goal.goal);
    display.info("Sending request!");
    display.info("Trajectory planning success!");
    return true;
  };

  auto waitForTrajectoryResult = [&traj_ac, &display](const double& timeout)
  {
    display.warn("Waiting for result...");
    bool finished_before_timout = traj_ac->waitForResult(ros::Duration(timeout));
    if (finished_before_timout)
    {
        actionlib::SimpleClientGoalState state = traj_ac->getState();
        display.success("Trajectory finished: ", state.toString().c_str());
        return true;
    }
    else
    {
        display.error("Action did not finish before the timeout.");
        return false;
    }
  };

  auto moveToTargetPoseAndWait = [&moveToTargetPose, &waitForTrajectoryResult](const geometry_msgs::Pose& target_pose, const double& execution_time,
                             const std_msgs::Float32MultiArray init_vel, const std_msgs::Float32MultiArray fin_vel,
                             const std_msgs::Float32MultiArray init_acc, const std_msgs::Float32MultiArray fin_acc) {
    if(!moveToTargetPose(target_pose, execution_time, init_vel, fin_vel, init_acc, fin_acc))
        return false;

    return waitForTrajectoryResult(execution_time+2.0);
  };

  hrii_gri_interface::SetGripperData srv_data;

  // Wait until the trajectory planner service exists
  display.warn("Waiting for services to start...");
  traj_ac->waitForServer();
  display.warn("traj_ac ok");
  // gripper_open_srv.waitForExistence();
  // display.warn("franka_gripper_open_srv ok");
  // gripper_close_srv.waitForExistence();
  // display.warn("franka_gripper_close_srv ok");
  // gripper_grasp_from_inside_srv.waitForExistence();
  // gripper_grasp_from_outside_srv.waitForExistence();
  display.success("Services started.");

  //Wait until the controller_started param is found
//  display.warn("Waiting for controller to start...");
//  do{
//    ros::Duration(0.5).sleep();
//  }while(!HRII_Utils::getParamSuccess("controller_started"));

//  display.success("Controller started.");
  double center_x = 0.5

  double center_y = 0.0
  double center_z = 0.4
  // msgs/srvs declaration
  geometry_msgs::Pose target_pose;
  std_msgs::Float32MultiArray init_fin_conditions;
  init_fin_conditions.data.resize(3);

  // FSM state declaration and initialization
  fsm_states fsm_state = fsm_states::HOMING;

  display.success("DUMMY_FSM STARTED!");
  int i=0;
	int n=2;

  double radius = 0.15
 
  geometry_msgs::Pose point
  for (int i = 0; i < (num_points - 1); i = i + 1) 
  {
        double angle = i * 2 * pi / (num_points - 1);
        
        point.position.x = center_x + radius * std::cos(angle);
        point.position.y = center_y + radius * std::sin(angle);
        point.position.z = center_z;
        point.orientation.w = 0.0;
        
        waypoints.push_back(point);
    }


  while (ros::ok() && fsm_state != fsm_states::EXIT)
  {
    switch (fsm_state)
    {
      case fsm_states::HOMING:
      {
        display.success("- - - HOMING - - -");

        // Define a target_pose w.r.t. the world frame
        target_pose.position.x = 0.413;
			  target_pose.position.y = 0.000;
			  target_pose.position.z = 0.230;
			  target_pose.orientation.x = 0.90;
			  target_pose.orientation.y = -1.025;
			  target_pose.orientation.z = 0.000;
			  target_pose.orientation.w = 0.000;


        // Fill in the service request to be sent to the trajectory planner
        // specifying either the desired speed OR the execution time, set the other one to 0.0
        if(!moveToTargetPoseAndWait(target_pose, 5.0, init_fin_conditions, init_fin_conditions, init_fin_conditions, init_fin_conditions))
          fsm_state = fsm_states::ERROR;
        else
          fsm_state = fsm_states::CENTER;

        break;
      }
      
          }


      case fsm_states::CENTER:
		  {
			display.success("- - - CENTER - - -");

			// If you would like to set a relative motion, only modify the desired value
			// E.g. here we just want to move 20cm negatively on the y-axis
			target_pose.position.y = 0.0;
			target_pose.position.x = 0.413;
			target_pose.position.z = 0.430;
			target_pose.orientation.x = 0.90;
			target_pose.orientation.y = -1.025;
			target_pose.orientation.z = 0.000;
			target_pose.orientation.w = 0.000;


        // Close gripper
        //  if (!gripper_close_srv.call(srv_data))
        // {
        //   display.error("Gripper closing error");
        //   fsm_state = fsm_states::ERROR;
        //   break;
        // }
        // if(!srv_data.response.success)
        // {
        //   display.error("Gripper server: fail closing gripper.");
        //   fsm_state = fsm_states::ERROR;
        //   break;
        // }

        if(!moveToTargetPoseAndWait(target_pose, 2.0, init_fin_conditions, init_fin_conditions, init_fin_conditions, init_fin_conditions))
          fsm_state = fsm_states::ERROR;
        else
          fsm_state = fsm_states::CIRCULAR;


        break;
      }
      case fsm_state::CIRCULAR:
          {
            display.success("- - - CIRCULAR - - -");

            // Define the circular trajectory parameters
            // double radius = 0.1;  // Radius of the circular path
            // double center_x = 0.413;  // X coordinate of the center of the circular path
            // double center_y = 0.0;  // Y coordinate of the center of the circular path
            // double angular_speed = 0.2;  // Angular speed of the circular motion

            // // Calculate the current position on the circular path based on time
            // double angle = angular_speed * ros::Time::now().toSec();
            // double target_x = center_x + radius * cos(angle);
            // double target_y = center_y + radius * sin(angle);

            // // Set the target pose for the circular motion
            // target_pose.position.x = target_x;
            // target_pose.position.y = target_y;
            // target_pose.position.z = 0.430;
            // target_pose.orientation.x = 0.90;
            // target_pose.orientation.y = -1.025;
            // target_pose.orientation.z = 0.000;
            // target_pose.orientation.w = 0.000;

            double radius = 0.15
 
            geometry_msgs::Pose point
            for (double i = 0; i < (num_points - 1); i=i+1) 
            {
                  double angle = i * 2 * pi / (num_points - 1);
                  
                  point.position.x = center_x + radius * std::cos(angle);
                  point.position.y = center_y + radius * std::sin(angle);
                  point.position.z = center_z;
                  point.orientation.w = 0.0;
                  
                  waypoints.push_back(point);
              }

            

            // Move to the target pose with a specified execution time
            if (!moveToTargetPoseAndWait(target_pose, 2.0, init_fin_conditions, init_fin_conditions, init_fin_conditions, init_fin_conditions))
                fsm_state = fsm_states::ERROR;
            else
                fsm_state = fsm_states::CENTER;

            break;
          }

      case fsm_states::MOVE_LEFT:
      {
        display.success("- - - MOVE_LEFT - - -");
        // If you would like to set a relative motion, only modify the desired value
        // E.g. here we just want to move 20cm negatively on the y-axis
        target_pose.position.y-= n*(0.05*1.732);
			  target_pose.position.x = 0.413-n*0.05;
        target_pose.orientation.x = 0.90;
			  target_pose.orientation.y = -1.025;
			  target_pose.orientation.z = 0.000;
			  target_pose.orientation.w = 0.000;


        // Close gripper
        //  if (!gripper_close_srv.call(srv_data))
        // {
        //   display.error("Gripper closing error");
        //   fsm_state = fsm_states::ERROR;
        //   break;
        // }
        // if(!srv_data.response.success)
        // {
        //   display.error("Gripper server: fail closing gripper.");
        //   fsm_state = fsm_states::ERROR;
        //   break;
        // }

        if(!moveToTargetPoseAndWait(target_pose, 2.0, init_fin_conditions, init_fin_conditions, init_fin_conditions, init_fin_conditions))
          fsm_state = fsm_states::ERROR;
        else
          fsm_state = fsm_states::CENTER;

        break;
      }
      case fsm_states::MOVE_UP:
      {
        display.success("- - - MOVE_UP - - -");
        // If you would like to set a relative motion, only modify the desired value
        // E.g. here we just want to move 20cm negatively on the y-axis
        target_pose.position.y = 0.0;
			  target_pose.position.x = 0.413+n*0.1;
			  target_pose.position.z = 0.430;
			  target_pose.orientation.x = 0.90;
			  target_pose.orientation.y = -1.025;
			  target_pose.orientation.z = 0.000;
			  target_pose.orientation.w = 0.000;



        // Close gripper
        //  if (!gripper_close_srv.call(srv_data))
        // {
        //   display.error("Gripper closing error");
        //   fsm_state = fsm_states::ERROR;
        //   break;
        // }
        // if(!srv_data.response.success)
        // {
        //   display.error("Gripper server: fail closing gripper.");
        //   fsm_state = fsm_states::ERROR;
        //   break;
        // }

        if(!moveToTargetPoseAndWait(target_pose, 2.0, init_fin_conditions, init_fin_conditions, init_fin_conditions, init_fin_conditions))
          fsm_state = fsm_states::ERROR;
        else
          fsm_state = fsm_states::CENTER;

        break;
      }



      case fsm_states::MOVE_RIGHT:
      {
        display.success("- - - MOVE_RIGHT - - -");
        target_pose.position.y+= n*(0.050*1.732);
			  target_pose.position.x = 0.413-n*0.05;
        target_pose.orientation.x = 0.90;
			  target_pose.orientation.y = -1.025;
			  target_pose.orientation.z = 0.000;
			  target_pose.orientation.w = 0.000;


        // Open gripper
        // if (!gripper_open_srv.call(srv_data))
        // {
        //   display.error("Gripper closure error");
        //   fsm_state = fsm_states::ERROR;
        //   break;
        // }
        // if(!srv_data.response.success)
        // {
        //   display.error("Gripper server: fail closing gripper.");
        //   fsm_state = fsm_states::ERROR;
        //   break;
        // }

        if(!moveToTargetPoseAndWait(target_pose, 2.0, init_fin_conditions, init_fin_conditions, init_fin_conditions, init_fin_conditions))
          fsm_state = fsm_states::ERROR;
        else
          fsm_state = fsm_states::CENTER;

        break;
      }

      case fsm_states::ERROR:
      {
        display.success("- - - ERROR - - -");
        // Choose a recovery mode or exit
        fsm_state = fsm_states::EXIT;
        break;
      }

      case fsm_states::EXIT:
      {
          display.success("- - - EXIT - - -");
          break;
      }
    }

    ros::spinOnce();

  }

  return 0;
}

// bool moveToTargetPose(const geometry_msgs::Pose& target_pose, const double& execution_time, const std_msgs::Float32MultiArray init_vel,
//                       const std_msgs::Float32MultiArray fin_vel, const std_msgs::Float32MultiArray init_acc,
//                       const std_msgs::Float32MultiArray fin_acc)
// {
//   // Fill in the service request to be sent to the trajectory planner
//   // specifying either the desired speed OR the execution time, set the other one to 0.0
//   hrii_robot_msgs::FifthOrderPolyActionGoal traj_goal;
//   traj_goal.header.stamp = ros::Time::now();
//   traj_goal.goal.target_pose = target_pose;
//   traj_goal.goal.execution_time = execution_time;
//   traj_goal.goal.initial_velocity = init_vel;
//   traj_goal.goal.final_velocity = fin_vel;
//   traj_goal.goal.initial_acceleration = init_acc;
//   traj_goal.goal.final_acceleration = fin_acc;

//   // Send a request to the trajectory planner service
//   traj_ac->sendGoal(traj_goal.goal);
//   display.info("Sending request!");
//   display.info("Trajectory planning success!");
//   return true;
// }

// bool moveToTargetPoseAndWait(const geometry_msgs::Pose& target_pose, const double& execution_time,
//                              const std_msgs::Float32MultiArray init_vel, const std_msgs::Float32MultiArray fin_vel,
//                              const std_msgs::Float32MultiArray init_acc, const std_msgs::Float32MultiArray fin_acc)
// {
//     if(!moveToTargetPose(target_pose, execution_time, init_vel, fin_vel, init_acc, fin_acc))
//         return false;

//     return waitForTrajectoryResult(execution_time+2.0);
// }

// bool waitForTrajectoryResult(const double& timeout)
// {
//     display.warn("Waiting for result...");
//     bool finished_before_timout = traj_ac->waitForResult(ros::Duration(timeout));
//     if (finished_before_timout)
//     {
//         actionlib::SimpleClientGoalState state = traj_ac->getState();
//         display.success("Trajectory finished: ", state.toString().c_str());
//         return true;
//     }
//     else
//     {
//         display.error("Action did not finish before the timeout.");
//         return false;
//     }
// }
