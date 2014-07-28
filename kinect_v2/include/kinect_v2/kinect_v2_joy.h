#ifndef KINECT_V2_JOY_H
#define KINECT_V2_JOY_H

#include <sstream>

#include <ros/ros.h>
#include <ros/console.h>
#include <kinect_msgs/HandState.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>

#include "kinect_vals.h"

class KinectV2Joy {
public:
	KinectV2Joy();

private:
	ros::Subscriber hand_sub;

	ros::Publisher joy_pub;

	void hand_callback(const kinect_msgs::HandState &hand);
};




#endif