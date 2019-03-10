#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
	if (!(nodeHandle.getParam("husky_controller/topic", topic)
			& nodeHandle.getParam("husky_controller/queue_size", queue_size)
			& nodeHandle.getParam("husky_controller/control_gain", control_gain))){
		ROS_ERROR("Could not find topic name and/or queue size!");
	}
	
	start_stop_sub = nodeHandle.subscribe("/start_stop", 5, &HuskyHighlevelController::checkStop, this);
	laser_subscriber = nodeHandle.subscribe(topic, queue_size, &HuskyHighlevelController::scanCallback, this);
	cmd_vel_publisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	vis_pub = nodeHandle.advertise<visualization_msgs::Marker>("visualization", 0);
	ROS_INFO("Scanning node working!");
}

HuskyHighlevelController::~HuskyHighlevelController()
{
}

void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan& msg)
{
	int j;

	min_laser_distance = msg.range_min;
	laser_ranges = msg.ranges;
	minDistance = laser_ranges[0];
	angle_min = msg.angle_min;
	angle_increment = msg.angle_increment;

	for(int i=0 ; i<laser_ranges.size() ; i++)
	{
		if((minDistance > laser_ranges[i]) & (laser_ranges[i] > min_laser_distance))
		{
			minDistance = laser_ranges[i];
			j = i;
		}
	}

	pillar_angle = angle_min + (angle_increment * j);

	if (status == true){
		if(!std::isinf(minDistance)){
			husky_pillar_control(2, pillar_angle);
		}
		else{
			husky_pillar_control(0, 0.7853);
		}
	}
	else{
		husky_pillar_control(0, 0);
	}

	cmd_vel_publisher.publish(cmd_vel_controller);
	pillar_position();

	ROS_INFO_STREAM("Minimum distance from laser scanner: " << minDistance);
}

void HuskyHighlevelController::husky_pillar_control(float speed, float angle)
{
	cmd_vel_controller.linear.x = speed;
	cmd_vel_controller.angular.z = control_gain * (0-angle);
}

void HuskyHighlevelController::checkStop(const std_msgs::Bool& trigger)
{
	if (trigger.data == true){
		status = true;
	}
	else{
		status = false;
	}
}

void HuskyHighlevelController::pillar_position()
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = minDistance*cos(pillar_angle);
	marker.pose.position.y = (-1)*minDistance*sin(pillar_angle);
	marker.pose.position.z = 1;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	vis_pub.publish( marker );
}

}
