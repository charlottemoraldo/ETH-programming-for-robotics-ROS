#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

ros::Publisher start_stop_publ;
std_msgs::Bool start_stop;

bool stop_(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response){
	if(request.data == false){
		start_stop.data = false;
		start_stop_publ.publish(start_stop);
		response.success = false;
		response.message = "False! ";
	}
	if(request.data == true){
		start_stop.data = true;
		start_stop_publ.publish(start_stop);
		response.success = true;
		response.message = "True! ";

	}
	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "emergency_stop_server");
	ros::NodeHandle nh;
	start_stop_publ = nh.advertise<std_msgs::Bool>("/start_stop", 1);
	ros::ServiceServer service = nh.advertiseService("emergency_stop", stop_);
	ros::spin();
	return 0;
}
