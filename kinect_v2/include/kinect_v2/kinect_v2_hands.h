#ifndef KINECT_V2_HANDS_H
#define KINECT_V2_HANDS_H

#include <sstream>

#include <ros/ros.h>
#include <ros/console.h>
#include <kinect_msgs/HandState.h>
#include <visualization_msgs/Marker.h>

class KinectV2Hands {
public:
	KinectV2Hands();

private:
	ros::Subscriber hand_sub;

	ros::Publisher marker_pub;

	void hand_callback(const kinect_msgs::HandState &hand);

	visualization_msgs::Marker get_marker(ros::Time stamp, std::string frame_id, std::string ns, int id, int state);
};




#endif
