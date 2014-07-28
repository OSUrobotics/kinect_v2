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

	ros::spin();
}

void KinectV2Tf::bodies_callback(const kinect_msgs::SkeletonArray &bodies){
	if(bodies.bodies.size() > 0){
		broadcast_body_tf(ros::Time::now(), bodies.bodies[0]);
	}
}

void KinectV2Tf::broadcast_body_tf(ros::Time stamp, const kinect_msgs::Skeleton &body){
	std::ostringstream stream;
	stream << 1;
	std::string id = stream.str();

    // Torso
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_HEAD],          body.joints[JOINTTYPE_NECK],          "head_" + id,          "neck_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_NECK],          body.joints[JOINTTYPE_SPINESHOULDER], "neck_" + id,          "spineshoulder_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SPINESHOULDER], body.joints[JOINTTYPE_SPINEMID],      "spineshoulder_" + id, "spinemid_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SPINEMID],      body.joints[JOINTTYPE_SPINEBASE],     "spinemid_" + id,      "spinebase_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SPINESHOULDER], body.joints[JOINTTYPE_SHOULDERRIGHT], "spineshoulder_" + id, "shoulderright_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SPINESHOULDER], body.joints[JOINTTYPE_SHOULDERLEFT],  "spineshoulder_" + id, "shoulderleft_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SPINEBASE],     body.joints[JOINTTYPE_HIPRIGHT],      "spinebase_" + id,     "hipright_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SPINEBASE],     body.joints[JOINTTYPE_HIPLEFT],       "spinebase_" + id,     "hipleft_" + id);
    
    // Right Arm    
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SHOULDERRIGHT], body.joints[JOINTTYPE_ELBOWRIGHT],    "shoulderright_" + id, "elbowright_" + id);
    broadcast_hand_tf(stamp, body.joints[JOINTTYPE_ELBOWRIGHT],     body.joints[JOINTTYPE_WRISTRIGHT], body.joints[JOINTTYPE_HANDRIGHT],    "elbowright_" + id,     "wristright_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_WRISTRIGHT],    body.joints[JOINTTYPE_HANDRIGHT],     "wristright_" + id,    "handright_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_HANDRIGHT],     body.joints[JOINTTYPE_HANDTIPRIGHT],  "handright_" + id,     "handtipright_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_WRISTRIGHT],    body.joints[JOINTTYPE_THUMBRIGHT],    "wristright_" + id,    "thumbright_" + id);

    // Left Arm
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_SHOULDERLEFT],  body.joints[JOINTTYPE_ELBOWLEFT],     "shoulderleft_" + id,  "elbowleft_" + id);
    broadcast_hand_tf(stamp, body.joints[JOINTTYPE_ELBOWLEFT],     body.joints[JOINTTYPE_WRISTLEFT], body.joints[JOINTTYPE_HANDLEFT],    "elbowleft_" + id,     "wristleft_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_WRISTLEFT],     body.joints[JOINTTYPE_HANDLEFT],      "wristleft_" + id,     "handleft_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_HANDLEFT],      body.joints[JOINTTYPE_HANDTIPLEFT],   "handleft_" + id,      "handtipleft_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_WRISTLEFT],     body.joints[JOINTTYPE_THUMBLEFT],     "wristleft_" + id,     "thumbleft_" + id);

    // Right Leg
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_HIPRIGHT],      body.joints[JOINTTYPE_KNEERIGHT],     "hipright_" + id,      "kneeright_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_KNEERIGHT],     body.joints[JOINTTYPE_ANKLERIGHT],    "kneeright_" + id,     "ankleright_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_ANKLERIGHT],    body.joints[JOINTTYPE_FOOTRIGHT],     "ankleright_" + id,    "footright_" + id);

    // Left Leg
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_HIPLEFT],       body.joints[JOINTTYPE_KNEELEFT],      "hipleft_" + id,       "kneeleft_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_KNEELEFT],      body.joints[JOINTTYPE_ANKLELEFT],     "kneeleft_" + id,      "ankleleft_" + id);
    broadcast_joint_tf(stamp, body.joints[JOINTTYPE_ANKLELEFT],     body.joints[JOINTTYPE_FOOTLEFT],      "ankleleft_" + id,     "footleft_" + id);
}

void KinectV2Tf::broadcast_joint_tf(ros::Time stamp, geometry_msgs::Pose parent_pos, geometry_msgs::Pose child_pos, std::string parent, std::string child){
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(child_pos.position.z - parent_pos.position.z, child_pos.position.x - parent_pos.position.x, child_pos.position.y - parent_pos.position.y) );

	tf::Quaternion q(0, 0, 0, 1);
	transform.setRotation(q);

	broadcaster.sendTransform(tf::StampedTransform(transform, stamp, parent, child));
}

void KinectV2Tf::broadcast_hand_tf(ros::Time stamp, geometry_msgs::Pose parent_pos, geometry_msgs::Pose wrist_pos, geometry_msgs::Pose hand_pos, std::string parent, std::string child){
	tf::Transform transform;
	
	float dx = hand_pos.position.z - wrist_pos.position.z;
	float dy = hand_pos.position.x - wrist_pos.position.x;
	float dz = hand_pos.position.y - wrist_pos.position.y;

	float pitch = atan2(dz, dx);
	float roll  = atan2(dy, dz);
	float yaw   = atan2(dx, dy); 

	transform.setOrigin( tf::Vector3(wrist_pos.position.z - parent_pos.position.z, wrist_pos.position.x - parent_pos.position.x, wrist_pos.position.y - parent_pos.position.y) );

	tf::Quaternion q(yaw, pitch, roll);
	transform.setRotation(q);

	broadcaster.sendTransform(tf::StampedTransform(transform, stamp, parent, child));
}
