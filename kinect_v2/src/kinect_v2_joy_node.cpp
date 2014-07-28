#include "../include/kinect_v2/kinect_v2_joy.h"




int main(int argc, char** argv){
	ros::init(argc, argv, "kinect_v2_joy");

	KinectV2Joy k = KinectV2Joy();

	return 0;
}