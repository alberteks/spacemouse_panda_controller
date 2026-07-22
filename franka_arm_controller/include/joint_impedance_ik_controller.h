#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <Eigen/Dense>

namespace franka_arm_controllers {

class JointImpedanceIKController
    : public controller_interface::MultiInterfaceController // gives attributes of the arm 
          franka_hw::FrankaModelInterface, //sends commands to the 7 joints of the arm
          hardware_interface::EffortJointInterface, //gives current positions
          franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // helper methods — declared here, defined in the .cpp
  void updateJointStates();
  Eigen::Matrix<double, 7, 1> computeTorqueCommand(
      const Eigen::Matrix<double, 7, 1>& q_desired,
      const Eigen::Matrix<double, 7, 1>& q_current,
      const Eigen::Matrix<double, 7, 1>& dq_current);
  void solveIK(const Eigen::Vector3d& new_position, const Eigen::Quaterniond& new_orientation);
  void spacemouseCallback(const geometry_msgs::TwistConstPtr& msg);
  tf2::Vector3 transformVelocityToWorldFrame(const geometry_msgs::TwistConstPtr& msg) const;

  // member variables — the controller's persistent state
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  ros::Subscriber spacemouse_sub_;

  Eigen::Matrix<double, 7, 1> k_gains_;
  Eigen::Matrix<double, 7, 1> d_gains_;
  Eigen::Matrix<double, 7, 1> dq_filtered_;

  std::string arm_id_;
  std::vector<double> arm_mounting_orientation_;

  urdf::Model model_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  KDL::JntArray q_min_, q_max_, q_init_, q_result_;

  Eigen::Vector3d position_, desired_linear_position_update_;
  Eigen::Quaterniond orientation_, desired_angular_position_update_quaternion_;
  Eigen::Vector3d desired_angular_position_update_;

  std::vector<double> joint_positions_desired_;
  std::vector<double> joint_positions_current_, joint_velocities_current_, joint_efforts_current_;

  static constexpr int num_joints_ = 7;
};

}  // namespace franka_arm_controllers