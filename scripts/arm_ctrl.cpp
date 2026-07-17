#include <cmath>
#include <iostream>
#include <stdio.h> // printf

#include <hidapi.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "shared_state.h"

void controlThread(const std::string& hostname) { 
  try {
    franka::Robot robot(hostname); // access robot through libfranka via its hostname
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}}; // default joint configuration
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 16> initial_pose;
    std::array<double, 16> current_pose; // stores current pose of robot for reference to move it
    double time = 0.0;
    
    SpacemouseInput currentInput; // stores latest spacemouse input

    // main motion loop
    robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();

      if (time == 0.0) {
        // at beginning, since we ensured robot moved to starting pose set that as the current pose
        initial_pose = robot_state.O_T_EE;
        current_pose = initial_pose;
      }

      // get mouse input from spacemouse.cpp thread
      //lock and copy data out
      {
        std::unique_lock<std::mutex> lock(g_mouseMutex, std::try_to_lock);
        if (lock.owns_lock()) { // only get the mouse input if lock is available rn. if not, then reuse currentInput from prev mouse state read
          currentInput = g_mouseData;
        }
      } // lock releases and control loop not blocked
      
      // adjust cartesian pose of robot arm based on mouse input
      current_pose[12] += currentInput.x;
      current_pose[13] += currentInput.y;
      current_pose[14] += currentInput.z;

      if (time >= 10.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(current_pose);
      }
      return current_pose;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
  }
}