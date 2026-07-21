#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>

void printState(const std::string& hostname) {
  try {
    franka::Robot robot(hostname);

    size_t count = 0;
    robot.read([&count](const franka::RobotState& robot_state) {
      std::cout << robot_state << std::endl;
      return count++ < 100;
    });

    std::cout << "Done." << std::endl;
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
  }
}