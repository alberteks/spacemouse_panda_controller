#include <stdio.h>
#include <hidapi.h>
#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

static hid_device* g_handle = nullptr;
int res;

bool initSpacemouse() {
	// Open the device using the VID, PID,
	// and optionally the Serial number.
	// Initialize the hidapi library
	res = hid_init();
	g_handle = hid_open(0x256F , 0xc635, NULL);
	if (!g_handle) {
		printf("Unable to open device\n");
		hid_exit();
		return false;
	}

	hid_set_nonblocking(g_handle,1);
	return true;
}

void closeSpacemouse() {
    if (g_handle) {
        hid_close(g_handle);
    }
    hid_exit();
}

double inputToOutput(double input, double input_start, double input_end, double output_start, double output_end){
	double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
	double output = output_start + slope * (input - input_start);
	return output;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "spacemouse_node");
	ros::NodeHandle nh; // nh is object representing spacemouse node

	// define topic name that node publishes to. then, controller will subscribe to this exact topic--only thing linking the two given decoupling setup in ros
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("spacemouse/twist", 1); // args are name of topic, queue size
	
	if (!initSpacemouse()) { // initialize spacemouse for reading; if not accessible then exit process
        std::cerr << "Failed to lock HID descriptors. Exiting." << std::endl;
        return -1;
    }

	ros::Rate rate(100); // sets mouse polling rate (to 100Hz)

	// double time_counter = 0.0;
	constexpr int bufSize = 10;
	unsigned char buf[bufSize];

	double input_start = 100.0; // lowest raw mouse value we register
	double input_end = 360.0;  // highest raw mouse value it is possible to register
	double output_start_lin = 0.0; // beginning of linear range we convert mouse values to
	double output_end_lin = 0.02; // end of linear range we convert mouse values to (2 cm)
	double output_start_ang = 0.0; // beginning of angular range we convert mouse values to
	double output_end_ang = (3.1415926535 / 180); // end of angular range we convert mouse values to (1 deg--units in rad)

	// main loop to read spacemouse input
	while (ros::ok()){
		// time_counter+=1;
		geometry_msgs::Twist mouse_msg; // twist struct has linear and angular 3d vectors

		//need two reads to get both x, y, z and roll, pitch, yaw
		res = hid_read(g_handle, buf, bufSize);
		res = hid_read(g_handle, buf, bufSize);

		int lin_x, lin_y, lin_z, ang_x, ang_y, ang_z = 0;
		
		if (res > 0) {
			// update x, y, z, roll, pitch, yaw
			if (buf[0] != 1){
				ang_x = (short)(buf[2] << 8) | buf[1];
				ang_y = (short)(buf[4] << 8) | buf[3];
				ang_z = (short)(buf[6] << 8) | buf[5];
			} else {
				lin_x = (short)(buf[2] << 8) | buf[1];
				lin_y = (short)(buf[4] << 8) | buf[3];
				lin_z = (short)(buf[6] << 8) | buf[5];
			}

			lin_z *= -1; // flip z axis so up is + and down is -

			if (std::abs(lin_x) <= 100) {
				mouse_msg.linear.x = 0;
			}
			else{
				mouse_msg.linear.x = inputToOutput(lin_x, input_start, input_end, output_start_lin, output_end_lin);
			}
			if (std::abs(lin_y) <= 100) {
				mouse_msg.linear.y = 0;
			}
			else{
				mouse_msg.linear.y = inputToOutput(lin_y, input_start, input_end, output_start_lin, output_end_lin);
			}
			if (std::abs(lin_z) <= 100) {
				mouse_msg.linear.z = 0;
			}
			else{
				mouse_msg.linear.z = inputToOutput(lin_z, input_start, input_end, output_start_lin, output_end_lin);
			}
			if (std::abs(ang_x) <= 100) {
				mouse_msg.angular.x = 0;
			}
			else{
				mouse_msg.angular.x = inputToOutput(ang_x, input_start, input_end, output_start_ang, output_end_ang);
			}
			if (std::abs(ang_y) <= 100) {
				mouse_msg.angular.y = 0;
			}
			else{
				mouse_msg.angular.y = inputToOutput(ang_y, input_start, input_end, output_start_ang, output_end_ang);
			}
			if (std::abs(ang_z) <= 100) {
				mouse_msg.angular.z = 0;
			}
			else{
				mouse_msg.angular.z = inputToOutput(ang_z, input_start, input_end, output_start_ang, output_end_ang);
			}

			pub.publish(mouse_msg); // sends msg payload to topic
		}
		ros::spinOnce();
		rate.sleep(); // sleep so that we poll at defined rate (100 Hz)
	}
	// Finalize the hidapi library
	closeSpacemouse();
	return 0;
}