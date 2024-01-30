/*
 * Author: Pietro Balatti
 * email: pietro.balatti@iit.it
 *
 * Finite State Machine template
 *
*/

#include "ros/ros.h"
#include <eigen_conversions/eigen_msg.h>
#include <hrii_utils/RosHelper.h>
#include <hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h>
#include "hrii_utils/MessageDisplay.h"

#include "hrii_gri_interface/SetGripperData.h"
#include "hrii_utils/ros/GeometryMsgOperations.h"

// FSM states declaration
enum class fsm_states {HOMING, MOVE_LEFT, MOVE_RIGHT, ERROR, EXIT};

geometry_msgs::Pose previous_target_pose;

HRII_Utils::MessageDisplay display;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dummy_fsm");
	ros::NodeHandle nh;

	display.setName(ros::this_node::getName());

	// Initialize service clients
	// ros::ServiceClient gripper_open_srv = nh.serviceClient<hrii_gri_interface::SetGripperData>("gripper/open");
	// ros::ServiceClient gripper_close_srv = nh.serviceClient<hrii_gri_interface::SetGripperData>("gripper/close");
	// ros::ServiceClient gripper_grasp_from_inside_srv = nh.serviceClient<hrii_gri_interface::SetGripperData>("gripper/grasp_from_inside");
	// ros::ServiceClient gripper_grasp_from_outside_srv = nh.serviceClient<hrii_gri_interface::SetGripperData>("gripper/grasp_from_outside");

	// hrii_gri_interface::SetGripperData srv_data;

	// Trajectory helper declaration and initialization
	HRII::TrajectoryHelper traj_helper("trajectory_handler");
	traj_helper.init();
	display.success("Trajectory handler initialized");
	
	// Wait until the gripper services exists
	// display.warn("Waiting for services to start...");
	// gripper_open_srv.waitForExistence();
	// gripper_close_srv.waitForExistence();
	// gripper_grasp_from_inside_srv.waitForExistence();
	// gripper_grasp_from_outside_srv.waitForExistence();
	// display.success("Services started.");

	//Wait until the controller_started param is found
	display.warn("Waiting for controller to start...");
	do{
		ros::Duration(0.5).sleep();
	}while(!HRII_Utils::getParamSuccess("controller_started"));
	
	display.success("Controller started.");

	// msgs/srvs declaration
	geometry_msgs::Pose target_pose;
	std::vector<geometry_msgs::Pose> waypoints;
	double execution_time = 5.0;

	// FSM state declaration and initialization
	fsm_states fsm_state = fsm_states::HOMING;

	display.success("DUMMY_FSM STARTED!");

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
			target_pose.position.z = 0.430;
			target_pose.orientation.x = 1.000;
			target_pose.orientation.y = 0.000;
			target_pose.orientation.z = 0.000;
			target_pose.orientation.w = 0.000;
			waypoints.push_back(target_pose);

			// Fill in the service request to be sent to the trajectory planner
			// specifying either the desired speed OR the execution time, set the other one to 0.0
			// The last boolean value sets the use_actual_pose_as_starting_pose
			if(!traj_helper.moveToTargetPoseAndWait(waypoints, execution_time, true))
				fsm_state = fsm_states::ERROR;
			else
				fsm_state = fsm_states::MOVE_LEFT;

			break;
		}

		case fsm_states::MOVE_LEFT:
		{
			display.success("- - - MOVE_LEFT - - -");

			// If you would like to set a relative motion, only modify the desired value
			// E.g. here we just want to move 20cm negatively on the y-axis
			target_pose.position.y-= 0.20;
			waypoints.push_back(target_pose);

			// Close gripper
			// if (!gripper_close_srv.call(srv_data))
			// {
			// 	display.error("Gripper closing error");
			// 	fsm_state = fsm_states::ERROR;
			// 	break;
			// }
			// if(!srv_data.response.success)
			// {
			// 	display.error("Gripper server: fail closing gripper.");
			// 	fsm_state = fsm_states::ERROR;
			// 	break;
			// }

			// waypoints is composed by previous desired and new target_pose
			// the last parameter (use_actual_pose_as_starting_pose) is set to false to avoid jumps
			if(!traj_helper.moveToTargetPoseAndWait(waypoints, execution_time, false))
				fsm_state = fsm_states::ERROR;
			else
				fsm_state = fsm_states::MOVE_RIGHT;

			// Erase previous point
			waypoints.erase(waypoints.begin());

			break;
		}

		case fsm_states::MOVE_RIGHT:
		{
			display.success("- - - MOVE_RIGHT - - -");
			target_pose.position.y+= 0.20;
			waypoints.push_back(target_pose);

			// Open gripper
			// if (!gripper_open_srv.call(srv_data))
			// {
			// display.error("Gripper closure error");
			// fsm_state = fsm_states::ERROR;
			// break;
			// }
			// if(!srv_data.response.success)
			// {
			// display.error("Gripper server: fail closing gripper.");
			// fsm_state = fsm_states::ERROR;
			// break;
			// }

			// waypoints is composed by previous desired and new target_pose
			// the last parameter (use_actual_pose_as_starting_pose) is set to false to avoid jumps
			if(!traj_helper.moveToTargetPoseAndWait(waypoints, execution_time, false))
				fsm_state = fsm_states::ERROR;
			else
				fsm_state = fsm_states::MOVE_LEFT;

			// Erase previous point
			waypoints.erase(waypoints.begin());

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
