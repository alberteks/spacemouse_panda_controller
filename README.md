# Franka Panda Emika Controller Using 3dConnexion Spacemouse Compact
## Description
A repository for controlling a 2020 Franka Panda Emika robotic arm with a [3dConnexion Spacemouse Compact](https://3dconnexion.com/us/product/spacemouse-compact/). Uses ROS1 Noetic and specifically the [franka_ros library](https://github.com/frankarobotics/franka_ros) with inverse kinematics for 6 DOF and gripper control. Primary motivation was to adapt franka_ros Spacemouse control to ROS1-only firmware. Can be adjusted to other robotic arms--contains three packages (a custom spacemouse input reading package, a franka_ros-derived joint IK controller, and a custom gripper control client). Developed in the [Interacting Robotics Systems Laboratory] (https://sites.google.com/a/stonybrook.edu/robotics/) at Stony Brook University.

The primary packages for this repository are in the catkin_ws directory. Other directories are test code for Mujoco simulations or Python implementation and are a WIP; can be ignored. The dependencies listed are for building the catkin_ws workspace project.

## Dependencies
System requirements: Ubuntu 20.04 (Focal Fossa)
- [ROS1 Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
- [Franka ROS](https://github.com/frankarobotics/franka_ros)
- [HIDAPI Hidraw](https://github.com/libusb/hidapi)
- [Eigen3](https://github.com/PX4/eigen)

Project can be built using [catkin_make](https://github.com/ros/catkin), with the CMakeLists.txt included in the repo.

## Running the project
To run the project, source an ROS directory containing the necessary packages. Then source the catkin_ws devel/setup.bash workspace, build the project using catkin_make, and use ROS commands to run the desired packages. To run the gripper node only, you can use the launch file in the panda_gripper_teleop package. To run the gripper node and control mode, you can use the launch file in the panda_arm_controller package (main implementation). To perform tasks using the Spacemouse and arm, control x, y, z, roll, pitch, and yaw using the mouse's 6 DOF from the joystick. Open the gripper using left button and close it using right button (it uses GraspAction, so that it will stop when it encounters resistance from an object and will be able to grasp it as it closes).

## License
This project is licensed under The MIT License -- see the [LICENSE](LICENSE) file for details.