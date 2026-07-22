#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <ros/subscriber.h>

#include <geometry_msgs/Twist.h>

#include <tf2/LinearMath/Vector3.h>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include <Eigen/Dense>

namespace franka_arm_controllers {

// Joint-space impedance controller that tracks a Cartesian teleop delta
// from mouse via per-cycle IK, using a PD + Coriolis-compensation
// torque law. 
class JointImpedanceIKController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface, //provides attributes
          hardware_interface::EffortJointInterface, //sends torque commands to the robot
          franka_hw::FrankaStateInterface> { //provides robot state info
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  static constexpr int kNumJoints = 7;
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  // --- internal helper methods ---

  // Refreshes joint_positions_current_/joint_velocities_current_/
  // joint_efforts_current_ from the claimed joint handles, and updates
  // the IK seed (q_init_) to the arm's current joint position.
  void updateJointStates();

  // PD-with-Coriolis-compensation torque law: stiffness * position error
  // - damping * filtered velocity + coriolis.
  Vector7d computeTorqueCommand(const Vector7d& q_desired,
                                 const Vector7d& q_current,
                                 const Vector7d& dq_current);

  // Solves joint angles (via KDL Newton-Raphson IK) that achieve the given
  // Cartesian pose, seeded from q_init_. Populates joint_positions_desired_.
  // On failure, logs and leaves joint_positions_desired_ at its last value
  // rather than throwing, so a transient IK failure holds position instead
  // of aborting the real-time control loop.
  void solveIK(const Eigen::Vector3d& new_position, const Eigen::Quaterniond& new_orientation);

  // Subscriber callback: converts an incoming Twist into a small per-cycle
  // Cartesian position/orientation delta (see max_linear_pos_update_ /
  // max_angular_pos_update_).
  void spacemouseCallback(const geometry_msgs::TwistConstPtr& msg);

  // Rotates a linear velocity from the robot's mounting frame into world
  // frame, using arm_mounting_orientation_.
  tf2::Vector3 transformVelocityToWorldFrame(const geometry_msgs::TwistConstPtr& msg) const;

  // Reads k_gains/d_gains/arm_id ROS params. Returns false (and logs) if
  // missing or mis-sized.
  bool assignParameters(ros::NodeHandle& node_handle);

  // --- hardware interface handles ---

  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

  // --- ROS interfaces ---

  ros::Subscriber spacemouse_sub_;

  // --- impedance gains / filtering ---

  Vector7d k_gains_;
  Vector7d d_gains_;
  Vector7d dq_filtered_;

  // --- identity / config ---

  std::string arm_id_;
  std::string namespace_prefix_;
  std::vector<double> arm_mounting_orientation_;  // roll, pitch, yaw of the base mount

  // Max per-cycle Cartesian deltas applied from a Twist message. Empirically
  // tuned: raise for a faster/less precise feel, lower for slower/more
  // precise control. See spacemouseCallback().
  double max_linear_pos_update_ = 0.007;
  double max_angular_pos_update_ = 0.03;

  // --- kinematics (URDF -> KDL chain) ---

  urdf::Model model_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  unsigned int nj_ = 0;
  KDL::JntArray q_min_;
  KDL::JntArray q_max_;
  KDL::JntArray q_init_;
  KDL::JntArray q_result_;

  // --- Cartesian state / teleop targets ---

  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;

  Eigen::Vector3d desired_linear_position_update_;
  Eigen::Vector3d desired_angular_position_update_;
  Eigen::Quaterniond desired_angular_position_update_quaternion_;

  // --- joint-space state ---

  std::vector<double> joint_positions_desired_;
  std::vector<double> joint_positions_current_;
  std::vector<double> joint_velocities_current_;
  std::vector<double> joint_efforts_current_;
};

}  // namespace franka_arm_controllers