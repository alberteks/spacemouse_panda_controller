#include <stdio.h> // printf
#include <thread>
#include <chrono>
#include <hidapi.h>
#include "shared_state.h"

#define MAX_STR 255

namespace {
	int x, y, z, yaw, pitch, roll; 

	void parseBuf(unsigned char* buf){
		if (buf[0] == 1){
			pitch = (short)(buf[2] << 8) | buf[1];
			yaw = (short)(buf[4] << 8) | buf[3];
			roll = (short)(buf[6] << 8) | buf[5];
		} else {
			x = (short)(buf[2] << 8) | buf[1];
			y = (short)(buf[4] << 8) | buf[3];
			z = (short)(buf[6] << 8) | buf[5];
		}
	}
}

void spacemouseThread()
{
	constexpr int bufSize = 10;
	int res;
	unsigned char buf[bufSize];
	hid_device *handle;
	int i;

	// Initialize the hidapi library
	res = hid_init();

	// Open the device using the VID, PID,
	// and optionally the Serial number.
	handle = hid_open(0x256F , 0xc635, NULL);
	if (!handle) {
		printf("Unable to open device\n");
		hid_exit();
 		return 1;
	}

	hid_set_nonblocking(handle,1);

	// main loop to read spacemouse input
	while (true){
		res = hid_read(handle, buf, bufSize);
		res = hid_read(handle, buf, bufSize);
		parseBuf(buf); // sets x, y, z, pitch, yaw, roll

		// lock and update shared struct for use in arm ctrler
		{
			std::lock_guard<std::mutex> lock(g_mouseMutex);
			g_mouseData.x = x;
			g_mouseData.y = y;
			g_mouseData.z = z;
			g_mouseData.roll = roll;
			g_mouseData.pitch = pitch;
			g_mouseData.yaw = yaw;
		} // lock releases after shared struct finishes updating

		std::this_thread::sleep_for(std::chrono::milliseconds(10)); // polling rate for spacemouse
	}

	// Finalize the hidapi library
	res = hid_exit();
}