#ifndef KINECT_V2_JOY_H
#define KINECT_V2_JOY_H

#include <sstream>

#include <ros/ros.h>
#include <ros/console.h>
#include <kinect_msgs/HandState.h>
#include <kinect_msgs/Lean.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>

class KinectV2Joy {
public:
	KinectV2Joy();

private:
	ros::Subscriber hand_sub;

	ros::Subscriber joy_sub;

	ros::Subscriber tracked_id_sub;

	ros::Publisher joy_pub;

	sensor_msgs::Joy joy_msg;

	int tracked_id;

	int lopen_button;
	int lclose_button;

	int ropen_button;
	int rclose_button;

	int rx_joy;
	int ry_joy;

	int n_buttons;
	int n_axes;

	void hand_callback(const kinect_msgs::HandState &hand);

	void lean_callback(const kinect_msgs::Lean &lean);	

	void tracked_id_callback(const std_msgs::Int32 &id);

	void hydra_callback(const sensor_msgs::Joy &joy);
};




#endif