#include <stdio.h>
#include <thread>
#include <chrono>
#include <hidapi.h>
#include "SpacemouseArmController/shared_state.h"
#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

static hid_device* g_handle = nullptr;

bool initSpacemouse() {
	// Open the device using the VID, PID,
	// and optionally the Serial number.
	g_handle = hid_open(0x256F , 0xc635, NULL);
	if (!g_handle) {
		printf("Unable to open device\n");
		hid_exit();
		return false;
	}

	hid_set_nonblocking(g_handle,1);
	return true;
}

void spacemouseThread()
{
	double time_counter = 0.0;
	constexpr int bufSize = 10;
	int res;
	unsigned char buf[bufSize];

	int i;

	// Initialize the hidapi library
	res = hid_init();

	int local_x = 0, local_y = 0, local_z = 0;
	int local_pitch = 0, local_yaw = 0, local_roll = 0;

	// main loop to read spacemouse input
	while (true){
		time_counter+=1;
		//need two readss

		res = hid_read(g_handle, buf, bufSize);
		res = hid_read(g_handle, buf, bufSize);
		
		if (res > 0) {
			// update x, y, z, roll, pitch, yaw
			if (buf[0] != 1){
				local_pitch = (short)(buf[2] << 8) | buf[1];
				local_yaw = (short)(buf[4] << 8) | buf[3];
				local_roll = (short)(buf[6] << 8) | buf[5];
			} else {
				local_x = (short)(buf[2] << 8) | buf[1];
				local_y = (short)(buf[4] << 8) | buf[3];
				local_z = (short)(buf[6] << 8) | buf[5];
			}

			if (std::abs(local_x) <= 100) {
				local_x = 0;
			}
			if (std::abs(local_y) <= 100) {
				local_y = 0;
			}
			if (std::abs(local_z) <= 100) {
				local_z = 0;
			}
			if (std::abs(local_roll) <= 100) {
				local_roll = 0;
			}
			if (std::abs(local_pitch) <= 100) {
				local_pitch = 0;
			}
			if (std::abs(local_yaw) <= 100) {
				local_yaw = 0;
			}
			
			// if want to constantly output raw input to console, see below
			// if (time_counter >= 20.0) {
			// 	std::cout << "x: " << local_x << ", y: " << local_y << ", z: " << local_z << std::endl;
			// 	time_counter = 0.0;
			// }

			// lock and update shared struct for use in arm ctrler
			{
				std::lock_guard<std::mutex> lock(g_mouseMutex);
				g_mouseData.x = local_x;
				g_mouseData.y = local_y;
				g_mouseData.z = local_z;
				g_mouseData.roll = local_roll;
				g_mouseData.pitch = local_pitch;
				g_mouseData.yaw = local_yaw;
			} // lock releases after shared struct finishes updating
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10)); // polling rate for spacemouse
	}

	// Finalize the hidapi library
	res = hid_exit();
}

void closeSpacemouse() {
    if (g_handle) {
        hid_close(g_handle);
    }
    hid_exit();
}