#include <panda_arm_controller/joint_impedance_ik_controller.h>

#include <pluginlib/class_list_macros.h>

#include <franka_msgs/SetFullCollisionBehavior.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <Eigen/Geometry>

namespace panda_arm_controller
{

    bool GripperController::init(hardware_interface::RobotHW *robot_hw,
                                 ros::NodeHandle &node_handle)
    {

        // gripper client definitions
        gripper_move_client_ = std::make_unique<GripperMoveClient>("franka_gripper/move", true);
        gripper_grasp_client_ = std::make_unique<GripperGraspClient>("franka_gripper/grasp", true);
        gripper_homing_client_ = std::make_unique<GripperHomingClient>("franka_gripper/homing", true);

        // if can't connect to gripper, throw error
        if (!gripper_move_client_->waitForServer(ros::Duration(5.0)) || !gripper_grasp_client_->waitForServer(ros::Duration(5.0)) || !gripper_homing_client_->waitForServer(ros::Duration(5.0)))
        {
            ROS_ERROR("JointImpedanceIKController: gripper action servers not available");
            return false;
        }

        // home gripper once at startup
        franka_gripper::HomingGoal homing_goal;
        gripper_homing_client_->sendGoalAndWait(homing_goal, ros::Duration(15.0));

        // --- Spacemouse input for gripper--hold close button to close, open button to open
        gripper_close_sub_ = node_handle.subscribe(
            "/spacemouse/gripper_close_held", 1, &JointImpedanceIKController::gripperCloseCallback, this);
        gripper_open_sub_ = node_handle.subscribe(
            "/spacemouse/gripper_open_held", 1, &JointImpedanceIKController::gripperOpenCallback, this);

        // ensures that gripper motion runs independently of the 1kHz update() loop (on 10hz)
        gripper_timer_ = node_handle.createTimer(
            ros::Duration(0.1), &JointImpedanceIKController::gripperTimerCallback, this);
    }

    // ---------------------------------------------------------------------------
    // gripper callback functions
    // ---------------------------------------------------------------------------
    // read close command data
    void JointImpedanceIKController::gripperCloseCallback(const std_msgs::BoolConstPtr &msg)
    {
        gripper_close_held_ = msg->data;
    }

    // read open command data
    void JointImpedanceIKController::gripperOpenCallback(const std_msgs::BoolConstPtr &msg)
    {
        gripper_open_held_ = msg->data;
    }

    // main callback for gripper. sendgoal is on 10hz (every 0.1 s)
    void JointImpedanceIKController::gripperTimerCallback(const ros::TimerEvent &)
    {
        if (!gripper_close_held_ && !gripper_open_held_)
        {
            return; // nothing held, so don't do anything
        }

        const double kStepPerTick = 0.001; // meters per 0.1s tick, tune to taste (~1cm/sec)

        if (gripper_close_held_)
        { // if close button held down, move gripper to close position
            current_gripper_width_ -= kStepPerTick;
        }
        else if (gripper_open_held_)
        {
            current_gripper_width_ += kStepPerTick;
        }
        
        if (current_gripper_width<0) current_gripper_width_ = 0;
        if (current_gripper_width>0.08) current_gripper_width_ = 0.08;
        

        franka_gripper::MoveGoal goal;
        goal.width = current_gripper_width_;
        goal.speed = 0.1;
        gripper_move_client_->sendGoal(goal);
    }

}