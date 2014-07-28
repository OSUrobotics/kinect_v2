#include "../include/kinect_v2/kinect_v2_tf.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "kinect_v2_tf");

	KinectV2Tf k = KinectV2Tf();

	return 0;
}