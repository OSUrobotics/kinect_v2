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


KinectV2Tf::KinectV2Tf(){
	ros::NodeHandle private_nh("~");

	bodies_sub = private_nh.subscribe("/kinect_v2/bodies", 1, &KinectV2Tf::bodies_callback, this);

	track_id = -1;

	ros::spin();
}

void KinectV2Tf::bodies_callback(const kinect_msgs::SkeletonArray &bodies){
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
	ROS_INFO("New tracking id: %d", track_id)
}

void KinectV2Tf::broadcast_body_tf(ros::Time stamp, const kinect_msgs::Skeleton &body){
	// Torso
	geometry_msgs::Pose head = body.joints[JOINTTYPE_HEAD];
	head.orientation.w = 1.0;
	broadcast_joint_tf(stamp, head,                                 body.joints[JOINTTYPE_NECK],          "head_surrogate",          "neck_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_NECK],          body.joints[JOINTTYPE_SPINESHOULDER], "neck_surrogate",          "spineshoulder_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SPINESHOULDER], body.joints[JOINTTYPE_SPINEMID],      "spineshoulder_surrogate", "spinemid_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SPINEMID],      body.joints[JOINTTYPE_SPINEBASE],     "spinemid_surrogate",      "spinebase_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SPINESHOULDER], body.joints[JOINTTYPE_SHOULDERRIGHT], "spineshoulder_surrogate", "shoulderright_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SPINESHOULDER], body.joints[JOINTTYPE_SHOULDERLEFT],  "spineshoulder_surrogate", "shoulderleft_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SPINEBASE],     body.joints[JOINTTYPE_HIPRIGHT],      "spinebase_surrogate",     "hipright_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SPINEBASE],     body.joints[JOINTTYPE_HIPLEFT],       "spinebase_surrogate",     "hipleft_surrogate");
	
	// Right Arm    
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SHOULDERRIGHT], body.joints[JOINTTYPE_ELBOWRIGHT],    "shoulderright_surrogate", "elbowright_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_ELBOWRIGHT],    body.joints[JOINTTYPE_WRISTRIGHT],    "elbowright_surrogate",    "wristright_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_WRISTRIGHT],    body.joints[JOINTTYPE_HANDRIGHT],     "wristright_surrogate",    "handright_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_HANDRIGHT],     body.joints[JOINTTYPE_HANDTIPRIGHT],  "handright_surrogate",     "handtipright_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_WRISTRIGHT],    body.joints[JOINTTYPE_THUMBRIGHT],    "wristright_surrogate",    "thumbright_surrogate");

	// Left Arm
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SHOULDERLEFT],  body.joints[JOINTTYPE_ELBOWLEFT],     "shoulderleft_surrogate",  "elbowleft_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_ELBOWLEFT],     body.joints[JOINTTYPE_WRISTLEFT],     "elbowleft_surrogate",     "wristleft_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_WRISTLEFT],     body.joints[JOINTTYPE_HANDLEFT],      "wristleft_surrogate",     "handleft_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_HANDLEFT],      body.joints[JOINTTYPE_HANDTIPLEFT],   "handleft_surrogate",      "handtipleft_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_WRISTLEFT],     body.joints[JOINTTYPE_THUMBLEFT],     "wristleft_surrogate",     "thumbleft_surrogate");

	// Right Leg
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_HIPRIGHT],      body.joints[JOINTTYPE_KNEERIGHT],     "hipright_surrogate",      "kneeright_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_KNEERIGHT],     body.joints[JOINTTYPE_ANKLERIGHT],    "kneeright_surrogate",     "ankleright_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_ANKLERIGHT],    body.joints[JOINTTYPE_FOOTRIGHT],     "ankleright_surrogate",    "footright_surrogate");

	// Left Leg
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_HIPLEFT],       body.joints[JOINTTYPE_KNEELEFT],      "hipleft_surrogate",       "kneeleft_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_KNEELEFT],      body.joints[JOINTTYPE_ANKLELEFT],     "kneeleft_surrogate",      "ankleleft_surrogate");
	broadcast_joint_tf(stamp, body.joints[JOINTTYPE_ANKLELEFT],     body.joints[JOINTTYPE_FOOTLEFT],      "ankleleft_surrogate",     "footleft_surrogate");
}

void KinectV2Tf::broadcast_joint_tf(ros::Time stamp, geometry_msgs::Pose parent_pos, geometry_msgs::Pose child_pos, std::string parent, std::string child){
	tf::Transform parent_tf;
	parent_tf.setOrigin( tf::Vector3(parent_pos.position.x, parent_pos.position.y, parent_pos.position.z));
	parent_tf.setRotation(tf::Quaternion(parent_pos.orientation.x, parent_pos.orientation.y, parent_pos.orientation.z, parent_pos.orientation.w));

	tf::Transform child_tf;
	child_tf.setOrigin(tf::Vector3(child_pos.position.x, child_pos.position.y, child_pos.position.z));
	child_tf.setRotation(tf::Quaternion(child_pos.orientation.x, child_pos.orientation.y, child_pos.orientation.z, child_pos.orientation.w));

	// parent_tf.inverseTimes(child_tf);

	broadcaster.sendTransform(tf::StampedTransform(parent_tf.inverseTimes(child_tf), stamp, parent, child));
}