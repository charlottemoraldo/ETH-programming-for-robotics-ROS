#pragma once

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#define pi  3.14159;

using std::string;
using std::vector;
using geometry_msgs::Twist;

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:
	int queue_size;
	bool status;
	float minDistance;
	float min_laser_distance;
	float angle_min;
	float angle_increment;
	float pillar_angle;
	float control_gain;

	vector<float> laser_ranges;
	Twist cmd_vel_controller;
	string topic;

	void scanCallback(const sensor_msgs::LaserScan& msg);
	void husky_pillar_control(float speed, float angle);
	void pillar_position();
	void checkStop(const std_msgs::Bool& trigger);

	ros::NodeHandle nodeHandle_;
	ros::Subscriber laser_subscriber;
	ros::Subscriber start_stop_sub;
	ros::Publisher cmd_vel_publisher;
	ros::Publisher vis_pub;
};

} /* namespace */
