/*
 * Author: Hamidreza Raei, Pietro Balatti
 * email: hamidreza.raei@iit.it, pietro.balatti@iit.it
 *
 * Cisual odometry comparison experiment
 *
*/

#include "ros/ros.h"
#include <eigen_conversions/eigen_msg.h>
#include <hrii_utils/RosHelper.h>
#include <hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h>
#include "hrii_utils/MessageDisplay.h"


#include "hrii_utils/ros/GeometryMsgOperations.h"

// FSM states declaration
enum class fsm_states {HOMING, CENTER, MOVE_DOWN, MOVE_LEFT, MOVE_RIGHT, MOVE_UP, ERROR, EXIT};

geometry_msgs::Pose previous_target_pose;

HRII_Utils::MessageDisplay display;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dummy_fsm");
	ros::NodeHandle nh;

	display.setName(ros::this_node::getName());

	// Trajectory helper declaration and initialization
	HRII::TrajectoryHelper traj_helper("trajectory_handler/execute_trajectory");
	traj_helper.init();
	display.success("Trajectory handler initialized");


	display.warn("Waiting for services to start...");

	display.success("Services started.");

	//Wait until the controller_started param is found
	display.warn("Waiting for controller to start...");
	do{
		ros::Duration(0.5).sleep();
	}while(!HRII_Utils::getParamSuccess("controller_started"));

	display.success("Controller started.");

	// msgs/srvs declaration
	geometry_msgs::Pose target_pose;
	std::vector<geometry_msgs::Pose> waypoints;
	double execution_time = 3.0;

	// FSM state declaration and initialization
	fsm_states fsm_state = fsm_states::HOMING;

	display.success("DUMMY_FSM STARTED!");
	int i=0;
	int n=3;

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
				fsm_state = fsm_states::CENTER;

			break;
		}
		case fsm_states::CENTER:
		{
			display.success("- - - CENTER - - -");

			// If you would like to set a relative motion, only modify the desired value
			// E.g. here we just want to move 20cm negatively on the y-axis
			target_pose.position.y = 0.0;
			target_pose.position.x = 0.363;
			target_pose.position.z = 0.290;
			target_pose.orientation.x = 1.000;
			target_pose.orientation.y = 0.000;
			target_pose.orientation.z = 0.000;
			target_pose.orientation.w = 0.000;
			waypoints.push_back(target_pose);



			// waypoints is composed by previous desired and new target_pose
			// the last parameter (use_actual_pose_as_starting_pose) is set to false to avoid jumps
			if(!traj_helper.moveToTargetPoseAndWait(waypoints, execution_time, false))
				fsm_state = fsm_states::ERROR;
			else
				if(i==2)
				{
				    fsm_state = fsm_states::MOVE_UP;
					i=0;
				}
				else if (i==1)
				{
					fsm_state = fsm_states::MOVE_RIGHT;
					i+=1;
				}
				else if (i==0)
				{
					fsm_state = fsm_states::MOVE_LEFT;
					i+=1;
				}



			// Erase previous point
			waypoints.erase(waypoints.begin());

			break;

		}
		case fsm_states::MOVE_UP:
		{
			display.success("- - - move up - - -");

			// If you would like to set a relative motion, only modify the desired value
			// E.g. here we just want to move 20cm negatively on the y-axis
			// target_pose.position.y = 0.0;
			target_pose.position.x += n*0.1;
			target_pose.position.z = 0.290;
			target_pose.orientation.x = 1.000;
			target_pose.orientation.y = 0.000;
			target_pose.orientation.z = 0.000;
			target_pose.orientation.w = 0.500;
			waypoints.push_back(target_pose);



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

		case fsm_states::MOVE_LEFT:
		{
			display.success("- - - MOVE_LEFT - - -");

			// If you would like to set a relative motion, only modify the desired value
			// E.g. here we just want to move 20cm negatively on the y-axis
			target_pose.position.y= - n*(0.05*1.732);
			target_pose.position.x = 0.363;
			target_pose.orientation.y = 0.000;
			target_pose.orientation.z = 0.000;
			target_pose.orientation.w = 0.000;
			waypoints.push_back(target_pose);



			// waypoints is composed by previous desired and new target_pose
			// the last parameter (use_actual_pose_as_starting_pose) is set to false to avoid jumps
			if(!traj_helper.moveToTargetPoseAndWait(waypoints, execution_time, false))
				fsm_state = fsm_states::ERROR;
			else
				fsm_state = fsm_states::MOVE_UP;

			// Erase previous point
			waypoints.erase(waypoints.begin());

			break;
		}

		case fsm_states::MOVE_DOWN:
		{
			display.success("- - - MOVE_DOWN - - -");

			// If you would like to set a relative motion, only modify the desired value
			// E.g. here we just want to move 20cm negatively on the y-axis
			// target_pose.position.y-= n*(0.05*1.732);
			target_pose.position.x -= n*0.1;
			target_pose.orientation.y = 0.000;
			target_pose.orientation.z = 0.000;
			target_pose.orientation.w = 0.000;
			waypoints.push_back(target_pose);



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


		case fsm_states::MOVE_RIGHT:
		{
			display.success("- - - MOVE_RIGHT - - -");
			target_pose.position.y= +n*(0.050*1.732);
			// target_pose.position.x = 0.363-n*0.05;
			target_pose.orientation.y = 0.000;
			target_pose.orientation.z = 0.000;
			target_pose.orientation.w = 0.000;
			waypoints.push_back(target_pose);



			// waypoints is composed by previous desired and new target_pose
			// the last parameter (use_actual_pose_as_starting_pose) is set to false to avoid jumps
			if(!traj_helper.moveToTargetPoseAndWait(waypoints, execution_time, false))
				fsm_state = fsm_states::ERROR;
			else
				fsm_state = fsm_states::MOVE_DOWN;

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
