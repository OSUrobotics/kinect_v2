#include "../include/kinect_v2/kinect_v2_hands.h"




int main(int argc, char** argv){
	ros::init(argc, argv, "kinect_v2_hands");

	KinectV2Hands k = KinectV2Hands();

	return 0;
}