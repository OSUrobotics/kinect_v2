#ifndef KINECT_V2_TF_H
#define KINECT_V2_TF_H

#include <sstream>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <kinect_msgs/SkeletonArray.h>
#include <kinect_msgs/Skeleton.h>

class KinectV2Tf {
public:
	KinectV2Tf();

private:
	tf::TransformBroadcaster broadcaster;

	ros::Subscriber bodies_sub;
	ros::Publisher test_pub;

	void bodies_callback(const kinect_msgs::SkeletonArray &bodies);

	void broadcast_body_tf(ros::Time stamp, const kinect_msgs::Skeleton &body);

	void broadcast_joint_tf(ros::Time stamp, geometry_msgs::Pose parent_pos, geometry_msgs::Pose child_pos, std::string parent, std::string child);
};

#endif
