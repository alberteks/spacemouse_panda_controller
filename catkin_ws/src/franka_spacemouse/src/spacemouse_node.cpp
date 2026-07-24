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

int main (int argc, char** argv)
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
				ang_y = (short)(buf[2] << 8) | buf[1];
				ang_z = (short)(buf[4] << 8) | buf[3];
				ang_x = (short)(buf[6] << 8) | buf[5];
			} else {
				lin_x = (short)(buf[2] << 8) | buf[1];
				lin_y = (short)(buf[4] << 8) | buf[3];
				lin_z = (short)(buf[6] << 8) | buf[5];
			}

			if (std::abs(lin_x) <= 100) {
				lin_x = 0;
			}
			if (std::abs(lin_y) <= 100) {
				lin_y = 0;
			}
			if (std::abs(lin_z) <= 100) {
				lin_z = 0;
			}
			if (std::abs(ang_x) <= 100) {
				ang_x = 0;
			}
			if (std::abs(ang_y) <= 100) {
				ang_y = 0;
			}
			if (std::abs(ang_z) <= 100) {
				ang_z = 0;
			}
			
			double lin_magnitude = sqrt(pow(lin_x, 2) + pow(lin_y, 2) + pow(lin_z, 2));
			double ang_magnitude = sqrt(pow(ang_x, 2) + pow(ang_y, 2) + pow(ang_z, 2));
			
			if (lin_magnitude >= 0.0001) {
				mouse_msg.linear.x = lin_x / lin_magnitude;
				mouse_msg.linear.y = lin_y / lin_magnitude;
				mouse_msg.linear.z = lin_z / lin_magnitude;
			}
			else {
				mouse_msg.linear.x = 0.0;
				mouse_msg.linear.y = 0.0;
				mouse_msg.linear.z = 0.0;
			}

			if (ang_magnitude >= 0.0001) {
				mouse_msg.angular.x = ang_x / ang_magnitude;
				mouse_msg.angular.y = ang_y / ang_magnitude;
				mouse_msg.angular.z = ang_z / ang_magnitude;
			}
			else {
				mouse_msg.angular.x = 0.0;
				mouse_msg.angular.y = 0.0;
				mouse_msg.angular.z = 0.0;
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