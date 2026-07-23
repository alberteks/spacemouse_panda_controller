#include <panda_arm_controller/joint_impedance_ik_controller.h>

#include <pluginlib/class_list_macros.h>

#include <franka_msgs/SetFullCollisionBehavior.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <Eigen/Geometry>

namespace panda_arm_controller {

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------
bool JointImpedanceIKController::init(hardware_interface::RobotHW* robot_hw,
                                       ros::NodeHandle& node_handle) {
  if (!assignParameters(node_handle)) {
    return false;
  }

  // --- claim the effort (torque) interface, one handle per named joint ---
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) ||
      joint_names.size() != static_cast<size_t>(kNumJoints)) {
    ROS_ERROR("JointImpedanceIKController: joint_names not set or not of size %d", kNumJoints);
    return false;
  }

  auto* effort_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_interface == nullptr) {
    ROS_ERROR("JointImpedanceIKController: could not get EffortJointInterface from RobotHW");
    return false;
  }

  joint_handles_.clear();
  for (const auto& name : joint_names) {
    try {
      joint_handles_.push_back(effort_interface->getHandle(name));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR("JointImpedanceIKController: exception getting joint handle '%s': %s",
                name.c_str(), ex.what());
      return false;
    }
  }

  // --- claim the Franka model interface (Coriolis, mass matrix, etc.) ---
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR("JointImpedanceIKController: could not get FrankaModelInterface from RobotHW");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id_ + "_model"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR("JointImpedanceIKController: exception getting model handle: %s", ex.what());
    return false;
  }

  // --- claim the Franka state interface (O_T_EE, robot state) ---
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointImpedanceIKController: could not get FrankaStateInterface from RobotHW");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id_ + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR("JointImpedanceIKController: exception getting state handle: %s", ex.what());
    return false;
  }

  // // --- set conservative collision behavior before any teleop input can move the arm ---
  // ros::ServiceClient collision_client =
  //     node_handle.serviceClient<franka_msgs::SetFullCollisionBehavior>(
  //         "franka_control/set_full_collision_behavior");
  // if (!collision_client.waitForExistence(ros::Duration(5.0))) {
  //   ROS_ERROR("JointImpedanceIKController: set_full_collision_behavior service not available");
  //   return false;
  // }
  // franka_msgs::SetFullCollisionBehavior collision_srv;
  // // Conservative defaults; tune once basic operation is verified.
  // collision_srv.request.lower_torque_thresholds_acceleration = {20, 20, 18, 18, 16, 14, 12};
  // collision_srv.request.upper_torque_thresholds_acceleration = {20, 20, 18, 18, 16, 14, 12};
  // collision_srv.request.lower_torque_thresholds_nominal = {20, 20, 18, 18, 16, 14, 12};
  // collision_srv.request.upper_torque_thresholds_nominal = {20, 20, 18, 18, 16, 14, 12};
  // collision_srv.request.lower_force_thresholds_acceleration = {10, 10, 10, 10, 10, 10};
  // collision_srv.request.upper_force_thresholds_acceleration = {10, 10, 10, 10, 10, 10};
  // collision_srv.request.lower_force_thresholds_nominal = {10, 10, 10, 10, 10, 10};
  // collision_srv.request.upper_force_thresholds_nominal = {10, 10, 10, 10, 10, 10};
  // if (!collision_client.call(collision_srv) || !collision_srv.response.success) {
  //   ROS_ERROR("JointImpedanceIKController: failed to set collision behavior");
  //   return false;
  // }
  // ROS_INFO("JointImpedanceIKController: collision behavior set.");

  // --- fetch URDF and build the KDL chain ---
  std::string robot_description;
  if (!node_handle.getParam("/robot_description", robot_description)) {
    ROS_ERROR("JointImpedanceIKController: failed to get /robot_description param");
    return false;
  }
  if (!model_.initString(robot_description)) {
    ROS_ERROR("JointImpedanceIKController: failed to parse URDF");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(model_, tree_)) {
    ROS_ERROR("JointImpedanceIKController: failed to convert URDF to KDL tree");
    return false;
  }

  std::string base_name = arm_id_ + "_link0";
  std::string tcp_name = arm_id_ + "_hand_tcp";
  if (!tree_.getChain(base_name, tcp_name, chain_)) {
    ROS_ERROR("JointImpedanceIKController: failed to extract KDL chain to '%s'", tcp_name.c_str());
    return false;
  }

  nj_ = chain_.getNrOfJoints();
  q_min_ = KDL::JntArray(nj_);
  q_max_ = KDL::JntArray(nj_);
  q_init_ = KDL::JntArray(nj_);
  q_result_ = KDL::JntArray(nj_);

  unsigned int j = 0;
  for (const auto& segment : chain_.segments) {
    const KDL::Joint& kdl_joint = segment.getJoint();
    if (kdl_joint.getType() == KDL::Joint::None) {
      continue;
    }
    const std::string& joint_name = kdl_joint.getName();
    auto joint = model_.getJoint(joint_name);
    if (!joint || !joint->limits) {
      ROS_ERROR("JointImpedanceIKController: no limits found for joint '%s'", joint_name.c_str());
      return false;
    }
    q_min_(j) = joint->limits->lower;
    q_max_(j) = joint->limits->upper;
    q_init_(j) = (q_max_(j) + q_min_(j)) / 2.0;
    ++j;
  }

  // --- construct KDL solvers once, reused every solveIK() call ---
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
      chain_, q_min_, q_max_, *fk_solver_, *vel_solver_, 100, 1e-6);

  // --- SpaceMouse input ---
  spacemouse_sub_ = node_handle.subscribe(
      "spacemouse/twist", 1, &JointImpedanceIKController::spacemouseCallback, this);

  // Properly initialize fixed-size joint vectors
  joint_positions_desired_.assign(kNumJoints, 0.0);
  joint_positions_current_.assign(kNumJoints, 0.0);
  joint_velocities_current_.assign(kNumJoints, 0.0);
  joint_efforts_current_.assign(kNumJoints, 0.0);

  desired_linear_position_update_.setZero();
  desired_angular_position_update_.setZero();
  desired_angular_position_update_quaternion_.setIdentity();

  ROS_INFO("JointImpedanceIKController: initialized successfully.");
  return true;
}

// ---------------------------------------------------------------------------
// starting()
// ---------------------------------------------------------------------------
void JointImpedanceIKController::starting(const ros::Time& /*time*/) {
  dq_filtered_.setZero();
  desired_linear_position_update_.setZero();
  desired_angular_position_update_.setZero();
  desired_angular_position_update_quaternion_.setIdentity();

  // Prime current-state buffers and the IK seed from the arm's actual pose
  updateJointStates();

  // Set initial continuous target pose ONCE during starting()
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  target_position_ = transform.translation();
  target_orientation_ = Eigen::Quaterniond(transform.linear());

  ROS_INFO_STREAM_THROTTLE(5, "target_position_ = " << target_position_);
}

// ---------------------------------------------------------------------------
// update() — 1kHz Real-time Loop
// ---------------------------------------------------------------------------
void JointImpedanceIKController::update(const ros::Time& /*time*/,
                                         const ros::Duration& /*period*/) {
  updateJointStates();

  // Propose updated target Cartesian pose
  Eigen::Vector3d new_position = target_position_ + desired_linear_position_update_;
  Eigen::Quaterniond new_orientation = (target_orientation_ * desired_angular_position_update_quaternion_).normalized();

  // Solve IK and advance target position ONLY if IK succeeds
  if (solveIK(new_position, new_orientation)) {
    target_position_ = new_position;
    target_orientation_ = new_orientation;
  }

  if (joint_positions_desired_.empty()) {
    return;
  }

  Vector7d q_desired(joint_positions_desired_.data());
  Vector7d q_current(joint_positions_current_.data());
  Vector7d dq_current(joint_velocities_current_.data());

  ROS_INFO_STREAM_THROTTLE(5, "q_desired = " << q_desired);
  ROS_INFO_STREAM_THROTTLE(5, "q_current = " << q_current);

  Vector7d tau_d = computeTorqueCommand(q_desired, q_current, dq_current);

  for (int i = 0; i < kNumJoints; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
}

// ---------------------------------------------------------------------------
// updateJointStates()
// ---------------------------------------------------------------------------
void JointImpedanceIKController::updateJointStates() {
  for (int i = 0; i < kNumJoints; ++i) {
    joint_positions_current_[i] = joint_handles_[i].getPosition();
    joint_velocities_current_[i] = joint_handles_[i].getVelocity();
    joint_efforts_current_[i] = joint_handles_[i].getEffort();
    q_init_(i) = joint_positions_current_[i];
  }
}

// ---------------------------------------------------------------------------
// computeTorqueCommand()
// ---------------------------------------------------------------------------
JointImpedanceIKController::Vector7d JointImpedanceIKController::computeTorqueCommand(
    const Vector7d& q_desired, const Vector7d& q_current, const Vector7d& dq_current) {
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Vector7d coriolis(coriolis_array.data());

  const double kAlpha = 0.05;
  dq_filtered_ = (1.0 - kAlpha) * dq_filtered_ + kAlpha * dq_current;

  Vector7d q_error = q_desired - q_current;
  Vector7d tau_d = k_gains_.cwiseProduct(q_error) - d_gains_.cwiseProduct(dq_filtered_) + coriolis;

  ROS_INFO_STREAM_THROTTLE(5, "q_error = "<< q_error);

  return tau_d;
}

// ---------------------------------------------------------------------------
// solveIK()
// ---------------------------------------------------------------------------
bool JointImpedanceIKController::solveIK(const Eigen::Vector3d& new_position,
                                          const Eigen::Quaterniond& new_orientation) {
  KDL::Rotation kdl_rot = KDL::Rotation::Quaternion(
      new_orientation.x(), new_orientation.y(), new_orientation.z(), new_orientation.w());
  KDL::Vector kdl_pos(new_position.x(), new_position.y(), new_position.z());
  KDL::Frame desired_pose(kdl_rot, kdl_pos);

  int status = ik_solver_->CartToJnt(q_init_, desired_pose, q_result_);
  if (status < 0) {
    ROS_WARN_THROTTLE(1.0, "JointImpedanceIKController: IK failed (status %d), holding last target",
                       status);
    return false;
  }

  // Ensure vector size matches joint count before indexing
  if (joint_positions_desired_.size() != static_cast<size_t>(kNumJoints)) {
    joint_positions_desired_.resize(kNumJoints);
  }

  for (int i = 0; i < kNumJoints; ++i) {
    joint_positions_desired_[i] = q_result_(i);
  }
  return true;
}

// ---------------------------------------------------------------------------
// spacemouseCallback()
// ---------------------------------------------------------------------------
void JointImpedanceIKController::spacemouseCallback(const geometry_msgs::TwistConstPtr& msg) {
  tf2::Vector3 v_linear_world = transformVelocityToWorldFrame(msg);

  desired_angular_position_update_ =
      max_angular_pos_update_ * Eigen::Vector3d(msg->angular.x, msg->angular.y, msg->angular.z);
  desired_linear_position_update_ =
      max_linear_pos_update_ *
      Eigen::Vector3d(v_linear_world.x(), v_linear_world.y(), v_linear_world.z());

  Eigen::AngleAxisd roll_angle(desired_angular_position_update_.x(), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(desired_angular_position_update_.y(), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(desired_angular_position_update_.z(), Eigen::Vector3d::UnitZ());
  desired_angular_position_update_quaternion_ = yaw_angle * pitch_angle * roll_angle;
}

// ---------------------------------------------------------------------------
// transformVelocityToWorldFrame()
// ---------------------------------------------------------------------------
tf2::Vector3 JointImpedanceIKController::transformVelocityToWorldFrame(
    const geometry_msgs::TwistConstPtr& msg) const {
  tf2::Quaternion q;
  q.setRPY(arm_mounting_orientation_.at(0), arm_mounting_orientation_.at(1),
           arm_mounting_orientation_.at(2));

  tf2::Matrix3x3 rotation_matrix(q);
  rotation_matrix = rotation_matrix.transpose();

  tf2::Vector3 v_linear_robot(msg->linear.x, msg->linear.y, msg->linear.z);
  return rotation_matrix * v_linear_robot;
}

// ---------------------------------------------------------------------------
// assignParameters()
// ---------------------------------------------------------------------------
bool JointImpedanceIKController::assignParameters(ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("JointImpedanceIKController: arm_id parameter not set");
    return false;
  }

  if (!node_handle.getParam("arm_mounting_orientation", arm_mounting_orientation_) ||
      arm_mounting_orientation_.size() != 3) {
    ROS_WARN("JointImpedanceIKController: arm_mounting_orientation not set or wrong size, "
              "defaulting to [0, 0, 0]");
    arm_mounting_orientation_ = {0.0, 0.0, 0.0};
  }

  std::vector<double> k_gains, d_gains;
  if (!node_handle.getParam("k_gains", k_gains) ||
      k_gains.size() != static_cast<size_t>(kNumJoints)) {
    ROS_ERROR("JointImpedanceIKController: k_gains not set or not of size %d", kNumJoints);
    return false;
  }
  if (!node_handle.getParam("d_gains", d_gains) ||
      d_gains.size() != static_cast<size_t>(kNumJoints)) {
    ROS_ERROR("JointImpedanceIKController: d_gains not set or not of size %d", kNumJoints);
    return false;
  }
  for (int i = 0; i < kNumJoints; ++i) {
    k_gains_(i) = k_gains.at(i);
    d_gains_(i) = d_gains.at(i);
  }

  node_handle.param("max_linear_pos_update", max_linear_pos_update_, max_linear_pos_update_);
  node_handle.param("max_angular_pos_update", max_angular_pos_update_, max_angular_pos_update_);

  return true;
}

}  // namespace panda_arm_controller

PLUGINLIB_EXPORT_CLASS(panda_arm_controller::JointImpedanceIKController,
                        controller_interface::ControllerBase)