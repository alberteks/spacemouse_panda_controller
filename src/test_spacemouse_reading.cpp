#include <cmath>
#include <iostream>
#include <stdio.h> // printf

#include <hidapi.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include <math.h>

#include "SpacemouseArmController/shared_state.h"
#include "SpacemouseArmController/examples_common.h"

double goalSpeed = 0.0000001;
double armSpeed = 0.001;

void testSpacemouseReadingThread(const std::string& hostname) { 
  try {
    std::cout << "Connecting to robot at " << hostname << std::endl;

    franka::Robot robot(hostname); // access robot through libfranka via its hostname
    setDefaultBehavior(robot);

    std::array<double, 16> current_pose; 
    std::array<double, 16> init_pose; 
    
    SpacemouseInput currentInput{}; // stores latest spacemouse input
    DoublePose goalPosition;
    DoublePose difference;
    bool initialized = false; 
    double timeSincePrint = 0.0; // want to print out updates from loop every 2 sec, so use this to track
    bool shouldPrint = false;

    robot.control([&timeSincePrint, &shouldPrint, &init_pose, &initialized, &current_pose, &currentInput, &goalPosition, &difference](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
      timeSincePrint += period.toSec(); // update time since last print
        // set goal pose to initial end effector position for first cycle
      if (!initialized) {
        init_pose = robot_state.O_T_EE;
        current_pose = robot_state.O_T_EE;
        goalPosition.x = current_pose[12];
        goalPosition.y = current_pose[13];
        goalPosition.z = current_pose[14];
        std::cout << "initial goal pos-- x: " << goalPosition.x << ", y: " << goalPosition.y << ", z: " << goalPosition.z << "\n";
        initialized = true;
      }


      if (timeSincePrint >= 4.0) { // then this is cycle to print out updates, and reset time since last print
        shouldPrint = true;
        timeSincePrint = 0;
      }

      //get mouse input from spacemouse.cpp thread
      // lock and copy data out
      {
        std::unique_lock<std::mutex> lock(g_mouseMutex, std::try_to_lock);
        if (lock.owns_lock()) { // only get the mouse input if lock is available rn. if not, then reuse currentInput from prev mouse state read
          currentInput = g_mouseData;
        }
      } // lock releases and control loop not blocked

      // output raw mouse input
      // if (shouldPrint) {
      //   std::cout << "raw mouse input-- x: " << currentInput.x << ", y: " << currentInput.y << ", z: " << currentInput.z << "\n";
      // }

      // update goal position
      goalPosition.x += currentInput.x*goalSpeed;
      goalPosition.y += currentInput.y*goalSpeed;
      goalPosition.z += currentInput.z*goalSpeed;

      // output updated goal position
      if (shouldPrint) {
        std::cout << "goal pos-- x: " << goalPosition.x << ", y: " << goalPosition.y << ", z: " << goalPosition.z << "\n";
      }

      // find difference in goal and current
      difference.x = current_pose[12]-goalPosition.x;
      difference.y = current_pose[13]-goalPosition.y;
      difference.z = current_pose[14]-goalPosition.z;

      // find magnitude of difference vector
      float magnitude = sqrt(pow(difference.x,2)+pow(difference.y,2)+pow(difference.z,2));

      if (magnitude > 1e-6) {
        // normalize difference vector, multiply by speed
        difference.x *= armSpeed / magnitude;
        difference.y *= armSpeed / magnitude;
        difference.z *= armSpeed / magnitude;
      }

      //update current pose
      current_pose[12] += difference.x;
      current_pose[13] += difference.y;
      current_pose[14] += difference.z;

      // output how much we would (theoretically) update robot pos by
      if (shouldPrint) {
        std::cout << "update robot cartesian pos by-- x: " << difference.x << ", y: " << difference.y << ", z: " << difference.z << "\n"; 
        shouldPrint = false;
      }
      return franka::CartesianPose(init_pose); // return init pose so robot is stationary for test
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
  }
}