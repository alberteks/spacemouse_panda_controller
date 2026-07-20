#include <cmath>
#include <iostream>
#include <stdio.h> // printf

#include <hidapi.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include <math.h>

#include "shared_state.h"

float goalSpeed = 0.001;
float armSpeed = 0.001;
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
    double tot_duration = 10.0;
    std::array<double, 3> filtered_input;
    
    SpacemouseInput currentInput; // stores latest spacemouse input
    SpacemouseInput goalPosition;
    SpacemouseInput difference;
    

    // main motion loop
    robot.control([&time, &initial_pose, &current_pose, &currentInput, &filteredInput](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();
      double dt = period.toSec(); // tracks current cycle's change in time to use for linear interpolation

      if (time == 0.0) {
        // at beginning, since we ensured robot moved to starting pose set that as the current pose
        initial_pose = robot_state.O_T_EE;
        current_pose = initial_pose;

        goalPosition.x = initial_pose[12];
        goalPosition.y = initial_pose[13];
        goalPosition.z = initial_pose[14];
        //!! code needs to be updated to include orientation later.
      }

      // get mouse input from spacemouse.cpp thread
      //lock and copy data out
      {
        std::unique_lock<std::mutex> lock(g_mouseMutex, std::try_to_lock);
        if (lock.owns_lock()) { // only get the mouse input if lock is available rn. if not, then reuse currentInput from prev mouse state read
          currentInput = g_mouseData;
        }
      } // lock releases and control loop not blocked

      //update goal position
      goalPosition.x += currentInput.x*goalSpeed;
      goalPosition.y += currentInput.y*goalSpeed;
      goalPosition.z += currentInput.z*goalSpeed;
      
      //find difference in goal and current
      difference.x = current_pose[12]-goalPosition.x;
      difference.y = current_pose[13]-goalPosition.y;
      difference.z = current_pose[14]-goalPosition.z;

      //find magnitude of difference vector
      float magXY = sqrt(pow(difference.x,2)+pow(difference.y,2));
      float magnitude = sqrt(pow(magXY,2)+pow(difference.z,2));

      //normalize difference vector, multiply by speed
      difference.x *= armSpeed / magnitude;
      difference.y *= armSpeed / magnitude;
      difference.z *= armSpeed / magnitude;

      //update current pose
      current_pose[12] += difference.x;
      current_pose[13] += difference.y;
      current_pose[14] += difference.z;

      if (time >= tot_duration) {
        std::cout << std::endl << "Motion duration over, shutting down teleop" << std::endl;
        return franka::MotionFinished(current_pose);
      }
      return current_pose;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
  }
}