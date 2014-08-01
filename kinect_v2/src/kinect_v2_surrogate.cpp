#include "../include/kinect_v2/kinect_v2_surrogate.h"
// included in kinect_v2_tf.h:
// #include <sstream>

// #include <ros/ros.h>
// #include <ros/console.h>
// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Point.h>
// #include <tf/transform_broadcaster.h>
// #include <geometry_msgs/Pose.h>

// #include "kinect_vals.h"


KinectV2Surrogate::KinectV2Surrogate(){
	ros::NodeHandle private_nh("~");

	bodies_sub = private_nh.subscribe("/kinect_v2/bodies", 1, &KinectV2Surrogate::bodies_callback, this);

	track_id = -1;

	ros::spin();
}

void KinectV2Surrogate::bodies_callback(const kinect_msgs::SkeletonArray &bodies){
	if(bodies.bodies.size() == 0){
		ROS_WARN("Received empty body bag...");
	}

	// look for tracked body
	for(int i = 0; i < bodies.bodies.size(); i++){
		if(track_id == bodies.bodies[i].id){
		  broadcast_body_tf(ros::Time::now(), bodies.bodies[i]);
		  return;
		}
	}

	// if it can't be found, track the first body
	track_id = bodies.bodies[0].id;
	broadcast_body_tf(ros::Time::now(), bodies.bodies[0]);
	ROS_INFO("New tracking id: %d", track_id);
}

void KinectV2Surrogate::broadcast_body_tf(ros::Time stamp, const kinect_msgs::Skeleton &body){
	// Torso
	geometry_msgs::Pose head = body.joints[kinect_msgs::Skeleton::HEAD];
	head.orientation.w = 1.0;
	broadcast_joint_tf(stamp, head,                                              body.joints[kinect_msgs::Skeleton::NECK],          "surrogate_head",          "surrogate_neck");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::NECK],          body.joints[kinect_msgs::Skeleton::SPINESHOULDER], "surrogate_neck",          "surrogate_spineshoulder");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SPINESHOULDER], body.joints[kinect_msgs::Skeleton::SPINEMID],      "surrogate_spineshoulder", "surrogate_spinemid");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SPINEMID],      body.joints[kinect_msgs::Skeleton::SPINEBASE],     "surrogate_spinemid",      "surrogate_spinebase");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SPINESHOULDER], body.joints[kinect_msgs::Skeleton::SHOULDERRIGHT], "surrogate_spineshoulder", "surrogate_shoulderright");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SPINESHOULDER], body.joints[kinect_msgs::Skeleton::SHOULDERLEFT],  "surrogate_spineshoulder", "surrogate_shoulderleft");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SPINEBASE],     body.joints[kinect_msgs::Skeleton::HIPRIGHT],      "surrogate_spinebase",     "surrogate_hipright");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SPINEBASE],     body.joints[kinect_msgs::Skeleton::HIPLEFT],       "surrogate_spinebase",     "surrogate_hipleft");
	
	// Right Arm    
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SHOULDERRIGHT], body.joints[kinect_msgs::Skeleton::ELBOWRIGHT],    "surrogate_shoulderright", "surrogate_elbowright");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::ELBOWRIGHT],    body.joints[kinect_msgs::Skeleton::WRISTRIGHT],    "surrogate_elbowright",    "surrogate_wristright");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::WRISTRIGHT],    body.joints[kinect_msgs::Skeleton::HANDRIGHT],     "surrogate_wristright",    "surrogate_handright");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::HANDRIGHT],     body.joints[kinect_msgs::Skeleton::HANDTIPRIGHT],  "surrogate_handright",     "surrogate_handtipright");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::WRISTRIGHT],    body.joints[kinect_msgs::Skeleton::THUMBRIGHT],    "surrogate_wristright",    "surrogate_thumbright");

	// Left Arm
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SHOULDERLEFT],  body.joints[kinect_msgs::Skeleton::ELBOWLEFT],     "surrogate_shoulderleft",  "surrogate_elbowleft");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::ELBOWLEFT],     body.joints[kinect_msgs::Skeleton::WRISTLEFT],     "surrogate_elbowleft",     "surrogate_wristleft");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::WRISTLEFT],     body.joints[kinect_msgs::Skeleton::HANDLEFT],      "surrogate_wristleft",     "surrogate_handleft");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::HANDLEFT],      body.joints[kinect_msgs::Skeleton::HANDTIPLEFT],   "surrogate_handleft",      "surrogate_handtipleft");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::WRISTLEFT],     body.joints[kinect_msgs::Skeleton::THUMBLEFT],     "surrogate_wristleft",     "surrogate_thumbleft");

	// Right Leg
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::HIPRIGHT],      body.joints[kinect_msgs::Skeleton::KNEERIGHT],     "surrogate_hipright",      "surrogate_kneeright");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::KNEERIGHT],     body.joints[kinect_msgs::Skeleton::ANKLERIGHT],    "surrogate_kneeright",     "surrogate_ankleright");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::ANKLERIGHT],    body.joints[kinect_msgs::Skeleton::FOOTRIGHT],     "surrogate_ankleright",    "surrogate_footright");

	// Left Leg
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::HIPLEFT],       body.joints[kinect_msgs::Skeleton::KNEELEFT],      "surrogate_hipleft",       "surrogate_kneeleft");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::KNEELEFT],      body.joints[kinect_msgs::Skeleton::ANKLELEFT],     "surrogate_kneeleft",      "surrogate_ankleleft");
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::ANKLELEFT],     body.joints[kinect_msgs::Skeleton::FOOTLEFT],      "surrogate_ankleleft",     "surrogate_footleft");
}

void KinectV2Surrogate::broadcast_joint_tf(ros::Time stamp, geometry_msgs::Pose parent_pos, geometry_msgs::Pose child_pos, std::string parent, std::string child){
	tf::Transform parent_tf;
	parent_tf.setOrigin( tf::Vector3(parent_pos.position.x, parent_pos.position.y, parent_pos.position.z));
	parent_tf.setRotation(tf::Quaternion(parent_pos.orientation.x, parent_pos.orientation.y, parent_pos.orientation.z, parent_pos.orientation.w));

	tf::Transform child_tf;
	child_tf.setOrigin(tf::Vector3(child_pos.position.x, child_pos.position.y, child_pos.position.z));
	child_tf.setRotation(tf::Quaternion(child_pos.orientation.x, child_pos.orientation.y, child_pos.orientation.z, child_pos.orientation.w));

	// parent_tf.inverseTimes(child_tf);

	broadcaster.sendTransform(tf::StampedTransform(parent_tf.inverseTimes(child_tf), stamp, parent, child));
}