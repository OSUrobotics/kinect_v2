#include "../include/kinect_v2/kinect_v2_surrogate.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "kinect_v2_surrogate");

	KinectV2Surrogate k = KinectV2Surrogate();

	return 0;
}
