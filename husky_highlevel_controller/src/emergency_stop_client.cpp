#include <cstdlib>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "emergency_stop_client");
	ros::NodeHandle nodeHandle;

	ros::ServiceClient client = nodeHandle.serviceClient<std_srvs::SetBool>("emergency_stop");
	std_srvs::SetBool service;


	if(atoi(argv[1]) == 1){
			ROS_INFO("It's true");
			service.request.data = true;
	}else{
			service.request.data = false;
	}

	if(client.call(service)){
		if(service.response.success == true){
			ROS_DEBUG_STREAM(service.response.message);
		}
		else{
			ROS_DEBUG_STREAM(service.response.message);
		}
	}
	else{
		ROS_ERROR("Failed to call emergency stop service");
		return 1;
	}

	return 0;

}
