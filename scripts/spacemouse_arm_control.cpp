#include <cmath>
#include <iostream>
#include <stdio.h> // printf
#include <wchar.h> // wchar_t

#include <hidapi.h>

#include <franka/exception.h>
#include <franka/robot.h>

#define MAX_STR 255

int x;
int y;
int z;
int pitch;
int yaw;
int roll;

int bufSize = 10;

void parseBuf(unsigned char* buf){ // bit manipulation function to parse raw input
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

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
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
    double time = 0.0;

    // setup for getting input from 3dspacemouse
    int res;
    unsigned char buf[bufSize];
    wchar_t wstr[MAX_STR];
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

    // main motion loop
    robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();

      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE;
      }

      // read mouse input
      res = hid_read(handle, buf, bufSize);
		  res = hid_read(handle, buf, bufSize);
		  parseBuf(buf); // sets x, y, z, roll, pitch, yaw
      
      std::array<double, 16> new_pose = initial_pose;
      new_pose[12] += x;
      new_pose[13] += y;
      new_pose[14] += z;

      if (time >= 10.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  // Finalize the hidapi library
	res = hid_exit();
  return 0;
}