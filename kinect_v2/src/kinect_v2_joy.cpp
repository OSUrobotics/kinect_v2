#include "../include/kinect_v2/kinect_v2_hands.h"

KinectV2Hands::KinectV2Hands(){
	ros::NodeHandle private_nh("~");

	hand_sub = private_nh.subscribe("/kinect_v2/hand_state", 1, &KinectV2Hands::hand_callback, this);
	marker_pub = private_nh.advertise<visualization_msgs::Marker>("/hand_markers", 1);

	ros::spin();
}

void KinectV2Hands::hand_callback(const kinect_msgs::HandState &hand){
	std::ostringstream stream;
	stream << hand.id;
	std::string id = stream.str();	

	marker_pub.publish(get_marker(hand.header.stamp, "/handright_" + id, "righthand", hand.id, hand.right));
	marker_pub.publish(get_marker(hand.header.stamp, "/handleft_" + id, "lefthand", hand.id, hand.left));
}

visualization_msgs::Marker KinectV2Hands::get_marker(ros::Time stamp, std::string frame_id, std::string ns, int id, int state){
	visualization_msgs::Marker marker;

	marker.header.frame_id = frame_id;
	marker.ns              = ns;

	marker.header.stamp = stamp;
	marker.id           = id;

	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 0.25;
	marker.scale.y = 0.25;
	marker.scale.z = 0.25;

	switch(state){
		case HANDSTATE_UNKNOWN:
			marker.color.r = 0.75f;
			marker.color.g = 0.75f;
			marker.color.b = 0.75f;
			break;
		case HANDSTATE_NOTTRACKED:
			marker.color.r = 1.0f;
			marker.color.g = 1.0f;
			marker.color.b = 1.0f;
			break;
		case HANDSTATE_OPEN:
			marker.color.r = 0.0f;
			marker.color.g = 1.0f;
			marker.color.b = 0.0f;
			break;
		case HANDSTATE_CLOSED:
			marker.color.r = 1.0f;
			marker.color.g = 0.0f;
			marker.color.b = 0.0f;
			break;
		case HANDSTATE_LASSO:
			marker.color.r = 0.0f;
			marker.color.g = 0.0f;
			marker.color.b = 1.0f;
			break;
		default:
			ROS_WARN("Unexpected hand state value");
			marker.color.r = 0.0f;
			marker.color.g = 0.0f;
			marker.color.b = 0.0f;
	}
	
	marker.color.a = 0.5;

	marker.lifetime = ros::Duration(0.5);

	return marker;
}