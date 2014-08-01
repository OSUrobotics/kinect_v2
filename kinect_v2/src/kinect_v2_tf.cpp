#include "../include/kinect_v2/kinect_v2_tf.h"
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


KinectV2Tf::KinectV2Tf(){
	ros::NodeHandle private_nh("~");

	bodies_sub = private_nh.subscribe("/kinect_v2/bodies", 1, &KinectV2Tf::bodies_callback, this);
	test_pub   = private_nh.advertise<geometry_msgs::PoseArray>("test", 1);

	ros::spin();
}

void KinectV2Tf::bodies_callback(const kinect_msgs::SkeletonArray &bodies){
	for(int i = 0; i < bodies.bodies.size(); i++){
		broadcast_body_tf(bodies.header.stamp, bodies.bodies[i]);
	}
}

void KinectV2Tf::broadcast_body_tf(ros::Time stamp, const kinect_msgs::Skeleton &body){
	std::ostringstream stream;
	stream << body.id;
	std::string id = stream.str();

	geometry_msgs::Point pt;
	pt.x = 0.0;
	pt.y = 0.0;
	pt.z = 0.0;

	geometry_msgs::Quaternion q;
	q.x = 0.0;
	q.y = 0.0;
	q.z = 0.0;
	q.w = 1.0;

	geometry_msgs::Pose kinect;
	kinect.position    = pt;
	kinect.orientation = q;

	geometry_msgs::PoseArray pa;
	pa.header.frame_id = "kinect_v2";
	for(int i = 0; i < 25; i++)
		pa.poses.push_back(body.joints[i]);
	test_pub.publish(pa);

	// Head
	// the kinect gives an orientation of 0, 0, 0, 0 for the head so we need to change it to scare off nan's 
	geometry_msgs::Pose head = body.joints[kinect_msgs::Skeleton::HEAD];
	head.orientation.w = 1.0;
	broadcast_joint_tf(stamp, kinect,                                            head,                                              "/kinect_v2",          "head_" + id);

	// Torso
	broadcast_joint_tf(stamp, head,                                              body.joints[kinect_msgs::Skeleton::NECK],          "head_" + id,          "neck_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::NECK],          body.joints[kinect_msgs::Skeleton::SPINESHOULDER], "neck_" + id,          "spineshoulder_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SPINESHOULDER], body.joints[kinect_msgs::Skeleton::SPINEMID],      "spineshoulder_" + id, "spinemid_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SPINEMID],      body.joints[kinect_msgs::Skeleton::SPINEBASE],     "spinemid_" + id,      "spinebase_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SPINESHOULDER], body.joints[kinect_msgs::Skeleton::SHOULDERRIGHT], "spineshoulder_" + id, "shoulderright_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SPINESHOULDER], body.joints[kinect_msgs::Skeleton::SHOULDERLEFT],  "spineshoulder_" + id, "shoulderleft_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SPINEBASE],     body.joints[kinect_msgs::Skeleton::HIPRIGHT],      "spinebase_" + id,     "hipright_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SPINEBASE],     body.joints[kinect_msgs::Skeleton::HIPLEFT],       "spinebase_" + id,     "hipleft_" + id);
	
	// Right Arm    
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SHOULDERRIGHT], body.joints[kinect_msgs::Skeleton::ELBOWRIGHT],    "shoulderright_" + id, "elbowright_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::ELBOWRIGHT],    body.joints[kinect_msgs::Skeleton::WRISTRIGHT],    "elbowright_" + id,    "wristright_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::WRISTRIGHT],    body.joints[kinect_msgs::Skeleton::HANDRIGHT],     "wristright_" + id,    "handright_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::HANDRIGHT],     body.joints[kinect_msgs::Skeleton::HANDTIPRIGHT],  "handright_" + id,     "handtipright_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::WRISTRIGHT],    body.joints[kinect_msgs::Skeleton::THUMBRIGHT],    "wristright_" + id,    "thumbright_" + id);

	// Left Arm
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::SHOULDERLEFT],  body.joints[kinect_msgs::Skeleton::ELBOWLEFT],     "shoulderleft_" + id,  "elbowleft_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::ELBOWLEFT],     body.joints[kinect_msgs::Skeleton::WRISTLEFT],     "elbowleft_" + id,     "wristleft_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::WRISTLEFT],     body.joints[kinect_msgs::Skeleton::HANDLEFT],      "wristleft_" + id,     "handleft_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::HANDLEFT],      body.joints[kinect_msgs::Skeleton::HANDTIPLEFT],   "handleft_" + id,      "handtipleft_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::WRISTLEFT],     body.joints[kinect_msgs::Skeleton::THUMBLEFT],     "wristleft_" + id,     "thumbleft_" + id);

	// Right Leg
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::HIPRIGHT],      body.joints[kinect_msgs::Skeleton::KNEERIGHT],     "hipright_" + id,      "kneeright_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::KNEERIGHT],     body.joints[kinect_msgs::Skeleton::ANKLERIGHT],    "kneeright_" + id,     "ankleright_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::ANKLERIGHT],    body.joints[kinect_msgs::Skeleton::FOOTRIGHT],     "ankleright_" + id,    "footright_" + id);

	// Left Leg
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::HIPLEFT],       body.joints[kinect_msgs::Skeleton::KNEELEFT],      "hipleft_" + id,       "kneeleft_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::KNEELEFT],      body.joints[kinect_msgs::Skeleton::ANKLELEFT],     "kneeleft_" + id,      "ankleleft_" + id);
	broadcast_joint_tf(stamp, body.joints[kinect_msgs::Skeleton::ANKLELEFT],     body.joints[kinect_msgs::Skeleton::FOOTLEFT],      "ankleleft_" + id,     "footleft_" + id);
}

void KinectV2Tf::broadcast_joint_tf(ros::Time stamp, geometry_msgs::Pose parent_pos, geometry_msgs::Pose child_pos, std::string parent, std::string child){
	tf::Transform parent_tf;
	parent_tf.setOrigin( tf::Vector3(parent_pos.position.x, parent_pos.position.y, parent_pos.position.z));
	parent_tf.setRotation(tf::Quaternion(parent_pos.orientation.x, parent_pos.orientation.y, parent_pos.orientation.z, parent_pos.orientation.w));

	tf::Transform child_tf;
	child_tf.setOrigin(tf::Vector3(child_pos.position.x, child_pos.position.y, child_pos.position.z));
	child_tf.setRotation(tf::Quaternion(child_pos.orientation.x, child_pos.orientation.y, child_pos.orientation.z, child_pos.orientation.w));

	broadcaster.sendTransform(tf::StampedTransform(parent_tf.inverseTimes(child_tf), stamp, parent, child));
}
