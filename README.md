# Franka Panda Emika Controller Using 3dConnexion Spacemouse Compact
## Description
A repository for controlling a 2020 Franka Panda Emika robotic arm with a [3dConnexion Spacemouse Compact](https://3dconnexion.com/us/product/spacemouse-compact/). Uses ROS1 Noetic and specifically the [franka_ros library](https://github.com/frankarobotics/franka_ros) with inverse kinematics for 6 DOF and gripper control. Primary motivation was to adapt franka_ros Spacemouse control to ROS1-only firmware. Can be adjusted to other robotic arms--contains three packages (a custom spacemouse input reading package, a franka_ros-derived joint IK controller, and a custom gripper control client). Developed in the [Interacting Robotics Systems Laboratory](https://sites.google.com/a/stonybrook.edu/robotics/) at Stony Brook University.

The primary packages for this repository are in the catkin_ws directory. Other directories are test code for Mujoco simulations or Python implementation and are a WIP; can be ignored. The dependencies listed are for building the catkin_ws workspace project.

## Dependencies
System requirements: Ubuntu 20.04 (Focal Fossa)
- [ROS1 Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
- [Franka ROS](https://github.com/frankarobotics/franka_ros)
- [HIDAPI Hidraw](https://github.com/libusb/hidapi)
- [Eigen3](https://github.com/PX4/eigen)

Project can be built using [catkin_make](https://github.com/ros/catkin), with the CMakeLists.txt included in the repo.

## Running the project
Source the ROS base environment containing the necessary packages. Then build the project in the catkin_ws directory using catkin_make and source the newly built workspace's devel/setup.bash. 

You can then use ROS commands to run the desired packages. To run the gripper node only, use the launch file in the panda_gripper_teleop package. To run the gripper node and control mode, use the launch file in the panda_arm_controller package (main implementation). 

To perform tasks using the Spacemouse and arm, control x, y, z, roll, pitch, and yaw using the mouse's 6 DOF from the joystick. Open the gripper using left button and close it using right button (note: uses GraspAction, so the gripper automatically stops closing whenit detects physical resistance from an object).

## License
This project is licensed under The MIT License -- see the [LICENSE](LICENSE) file for details.