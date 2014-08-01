#include "../include/kinect_v2/kinect_v2_joy.h"

KinectV2Joy::KinectV2Joy(){
	ros::NodeHandle private_nh("~");

	hand_sub = private_nh.subscribe("/kinect_v2/hand_state", 1, &KinectV2Joy::hand_callback, this);
	joy_sub  = private_nh.subscribe("/hydra_joy", 1, &KinectV2Joy::hydra_callback, this);
	tracked_id_sub = private_nh.subscribe("/kinect_v2/surrogate_id", 1, &KinectV2Joy::tracked_id_callback, this);

	joy_pub  = private_nh.advertise<sensor_msgs::Joy>("/kinect_v2/joy", 1);

	private_nh.param("lopen_button", lopen_button, 5);
	private_nh.param("lclose_button", lclose_button, 7);

	private_nh.param("ropen_button", ropen_button, 13);
	private_nh.param("rclose_button", rclose_button, 15);

	private_nh.param("rx_joy", rx_joy, 2);
	private_nh.param("ry_joy", ry_joy, 3);

	private_nh.param("n_buttons", n_buttons, 16);
	private_nh.param("n_axes", n_axes, 16);

	if(    lopen_button  > n_buttons
		|| lclose_button > n_buttons
		|| ropen_button  > n_buttons
		|| rclose_button > n_buttons
		|| lopen_button  < 0
		|| lclose_button < 0
		|| ropen_button  < 0
		|| rclose_button < 0
		)
	{
		ROS_FATAL("buttons exceed array bounds");
		exit(1);
	}

	if(    rx_joy  > n_axes
		|| ry_joy  > n_axes
		|| rx_joy  < 0
		|| ry_joy  < 0
		)
	{
		ROS_FATAL("axes exceed array bounds");
		exit(1);
	}

	joy_msg.buttons.resize(n_buttons);
	joy_msg.axes.resize(n_axes);

	tracked_id = -1;

	ros::spin();
}

void KinectV2Joy::tracked_id_callback(const std_msgs::Int32 &id){
	tracked_id = id.data;
}

void KinectV2Joy::hand_callback(const kinect_msgs::HandState &hand){
	if(hand.id != tracked_id){
		return;
	}

	switch(hand.left){
		case kinect_msgs::HandState::OPEN:
			joy_msg.buttons[lopen_button] = true;
			joy_msg.buttons[lclose_button] = false;
			break;
		case kinect_msgs::HandState::CLOSED:
			joy_msg.buttons[lopen_button] = false;
			joy_msg.buttons[lclose_button] = true;
			break;
		default:
			joy_msg.buttons[lopen_button] = false;
			joy_msg.buttons[lclose_button] = false;
			break;
	}

	switch(hand.right){
		case kinect_msgs::HandState::OPEN:
			joy_msg.buttons[ropen_button] = true;
			joy_msg.buttons[rclose_button] = false;
			break;
		case kinect_msgs::HandState::CLOSED:
			joy_msg.buttons[ropen_button] = false;
			joy_msg.buttons[rclose_button] = true;
			break;
		default:
			joy_msg.buttons[ropen_button] = false;
			joy_msg.buttons[rclose_button] = false;
			break;
	}
}

void KinectV2Joy::lean_callback(const kinect_msgs::Lean &lean){
	if(lean.id != tracked_id){
		return;
	}

	joy_msg.axes[rx_joy] = lean.x;
	joy_msg.axes[ry_joy] = lean.y;
}


void KinectV2Joy::hydra_callback(const sensor_msgs::Joy &joy){
	if(joy.buttons.size() != n_buttons){
		ROS_ERROR("joy message has unexpected number of buttons");
		return;
	}

	if(joy.axes.size() != n_axes){
		ROS_ERROR("joy message has unexpected number of axes");
		return;
	}

	for(int i = 0; i < n_buttons; i++){
		if(    i != lopen_button
			&& i != lclose_button
			&& i != ropen_button
			&& i != rclose_button
			)
		{
			joy_msg.buttons[i] = joy.buttons[i];
		}
	}

	for(int i = 0; i < n_axes; i++){
		if(i != rx_joy
			&& i != ry_joy
			)
		{
			joy_msg.axes[i] = joy.axes[i];
		}
	}

	joy_msg.header.frame_id = joy.header.frame_id;
	joy_msg.header.stamp    = joy.header.stamp;

	joy_pub.publish(joy_msg);
}