#include <hidapi.h>
#include <chrono>
#include <stdio.h>
#include <iostream>
#include <thread>
#include "SpacemouseArmController/shared_state.h"

double goalSpeed = 0.0000001;

SpacemouseInput currentInput{}; // stores latest spacemouse input
DoublePose goalPosition{};
std::chrono::duration<double> timeSincePrint;
bool shouldPrint = false; 
auto start_time = std::chrono::high_resolution_clock::now();

void spacemouse_test_only() {
	while(true) {
		auto current_time = std::chrono::high_resolution_clock::now();
		timeSincePrint = current_time - start_time;
		if (timeSincePrint.count() >= 0.1) { // then this is cycle to print out updates, and reset time since last print
	        shouldPrint = true;
	        start_time = std::chrono::high_resolution_clock::now();
	    }
		// read mouse input into currentInput
		{
	        std::unique_lock<std::mutex> lock(g_mouseMutex, std::try_to_lock);
	        if (lock.owns_lock()) { // only get the mouse input if lock is available rn. if not, then reuse currentInput from prev mouse state read
	        	currentInput = g_mouseData;
	        }
	    } // lock releases and control loop not blocked

	    // print raw mouse input every 2 sec
	    if (shouldPrint) {
	    	std::cout << "raw mouse input-- x: " << currentInput.x << ", y: " << currentInput.y << ", z: " << currentInput.z << "\n";
	    }

	    // update goal position by mouse input scaled by goal speed
	    goalPosition.x += (double)currentInput.x*goalSpeed;
	    goalPosition.y += (double)currentInput.y*goalSpeed;
	    goalPosition.z += (double)currentInput.z*goalSpeed;
	    // print new goal position every 2 sec
	    if (shouldPrint) {
	    	std::cout << "goal pos-- x: " << goalPosition.x << ", y: " << goalPosition.y << ", z: " << goalPosition.z << "\n";
			shouldPrint = false;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1)); // polling rate
	}
}