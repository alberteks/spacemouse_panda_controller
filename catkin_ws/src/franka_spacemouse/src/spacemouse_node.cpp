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
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("franka_controller/target_cartesian_velocity_percent", 1); // args are name of topic, queue size
	
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
		
		if (res > 0) {
			// update x, y, z, roll, pitch, yaw
			if (buf[0] != 1){
				mouse_msg.angular.y = (short)(buf[2] << 8) | buf[1];
				mouse_msg.angular.z = (short)(buf[4] << 8) | buf[3];
				mouse_msg.angular.x = (short)(buf[6] << 8) | buf[5];
			} else {
				mouse_msg.linear.x = (short)(buf[2] << 8) | buf[1];
				mouse_msg.linear.y = (short)(buf[4] << 8) | buf[3];
				mouse_msg.linear.z = (short)(buf[6] << 8) | buf[5];
			}

			if (std::abs(mouse_msg.linear.x) <= 100) {
				mouse_msg.linear.x = 0;
			}
			if (std::abs(mouse_msg.linear.y) <= 100) {
				mouse_msg.linear.y = 0;
			}
			if (std::abs(mouse_msg.linear.z) <= 100) {
				mouse_msg.linear.z = 0;
			}
			if (std::abs(mouse_msg.angular.x) <= 100) {
				mouse_msg.angular.x = 0;
			}
			if (std::abs(mouse_msg.angular.y) <= 100) {
				mouse_msg.angular.y = 0;
			}
			if (std::abs(mouse_msg.angular.z) <= 100) {
				mouse_msg.linear.z = 0;
			}
			
			// if want to constantly output raw input to console, see below
			// if (time_counter >= 20.0) {
			// 	std::cout << "x: " << mouse_msg.linear.x << ", y: " << mouse_msg.linear.y << ", z: " << mouse_msg.linear.z << std::endl;
			// 	time_counter = 0.0;
			// }
		}
		pub.publish(mouse_msg); // sends msg payload to topic
		ros::spinOnce();
		rate.sleep(); // sleep so that we poll at defined rate (100 Hz)
	}
	// Finalize the hidapi library
	res = hid_exit();
	return 0;
}