#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/MoveAction.h>
#include <sensor_msgs/JointState.h> // for reading current finger positions, and therefore current gripper width
// (to then adjust goal width)
#include <std_msgs/Bool.h> // we receive two bool msgs from spacemouse node

#include <algorithm>
#include <mutex> // for multithread blocking

using MoveClient = actionlib::SimpleActionClient<franka_gripper::MoveAction>;

class GripperTeleopNode {
public:
    explicit GripperTeleopNode(ros::NodeHandle &nh) : move_client_("franka_gripper/move", true) {
        nh.param("step_width", step_width_, 0.004); // in meters per tick (0.4 cm per tick)
        nh.param("jog_speed", jog_speed_, 0.05); // desired jog speed passed to MoveGoal (m/s)
        nh.param("timer_rate", timer_rate_, 10.0); // timer rate in Hz for gripper loop
        nh.param("min_width", min_width_, 0.00); // min width grippers should go to
        nh.param("max_width", max_width_, 0.08); // max width grippers should go to

        // --- Spacemouse input for gripper--hold close button to close, open button to open
        gripper_close_sub_ = nh.subscribe(
            "/spacemouse/gripper_close_held", 1, &GripperTeleopNode::gripperCloseCallback, this);
        gripper_open_sub_ = nh.subscribe(
            "/spacemouse/gripper_open_held", 1, &GripperTeleopNode::gripperOpenCallback, this);
        // subscribe to gripper state publisher, which we use to determine width
        gripper_state_sub_ = nh.subscribe(
            "/franka_gripper/joint_states", 1, &GripperTeleopNode::gripperStateCallback, this);
        ROS_INFO("gripper_teleop_node: waiting for franka_gripper/move action server...");
        if (!move_client_.waitForServer(ros::Duration(5.0))){
            ROS_ERROR("gripper_teleop_node: franka_gripper/move action server not available.");
        }
        timer_ = nh.createTimer(ros::Duration(1.0 / timer_rate_), &GripperTeleopNode::timerCallback, this); // create timer which ticks at timer_rate
        ROS_INFO("gripper_teleop_node: ready");
    }

private:
    // update open and close variables as needed
    void gripperOpenCallback(const std_msgs::BoolConstPtr &msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        open_held_ = msg->data;
    }

    void gripperCloseCallback(const std_msgs::BoolConstPtr &msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        close_held_ = msg->data;
    }
    
    //callback method for finding gripper state. 
    // accepts sensor msg data to determine current gripper width
    void gripperStateCallback(const sensor_msgs::JointState::ConstPtr &msg){
        // note: franka_gripper publishes each finger's pos as width * 0.5
        if (msg->position.size() >= 2){
            std::lock_guard<std::mutex> lock(state_mutex_); // mutex lock to read finger positions
            current_width_ = msg->position[0] + msg->position[1]; // calculate width between gripper fingers
            have_width_ = true; // update bool to track whether we have current width and can proceed
        }
    }

    // primary callback method, running on own ros timer, for performing gripper movement each tick
    void timerCallback(const ros::TimerEvent &) {
        bool open_held, close_held, have_width;
        double current_width; 
        {
            std::lock_guard<std::mutex> lock(state_mutex_); // mutex lock to update open hold state, close hold state, and width variables
            open_held = open_held_;
            close_held = close_held_;
            have_width = have_width_; 
            current_width = current_width_;
        }

        if (!have_width) {
            return; // don't do anything if we haven't received msg for gripper width rn
        }

        bool gripper_busy = false; 
        if (goal_sent_) {
            actionlib::SimpleClientGoalState state = move_client_.getState(); // fetches state of moveClient so we avoid sending too many commands for gripper to perform
            gripper_busy = (state == actionlib::SimpleClientGoalState::ACTIVE || state == actionlib::SimpleClientGoalState::PENDING);
        }
        if (gripper_busy) {
            return; // don't do anything!
        }
        
        // if we are good to proceed, then actually command gripper movement by sending updated goal width
        double target_width = current_width; 
        if (open_held) {
            target_width = std::min(max_width_, current_width + step_width_); // update target_width, as long as doesn't exceed gripper limit
        }
        else if (close_held) {
            target_width = std::max(min_width_, current_width - step_width_);
        }
        
        if (std::abs(target_width - current_width) < 1e-4){
            return; // don't do anything sicne the current_width is already target_width (reached limit)
        }

        franka_gripper::MoveGoal goal; // instantiate goal and then set target width and speed
        goal.width = target_width; 
        goal.speed = jog_speed_; 
        move_client_.sendGoal(goal); // send goal to action client to perform on robot
        goal_sent_ = true; // now a goal has been set, so update bool
    }

    MoveClient move_client_;
    ros::Subscriber gripper_close_sub_;
    ros::Subscriber gripper_open_sub_;
    ros::Subscriber gripper_state_sub_;
    ros::Timer timer_;

    std::mutex state_mutex_;
    bool open_held_{false};
    bool close_held_{false};
    bool have_width_{false};
    bool goal_sent_{false};
    double current_width_{0.08}; // gripper considered open by default

    double step_width_;
    double jog_speed_;
    double timer_rate_;
    double max_width_;
    double min_width_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_teleop_node");
    ros::NodeHandle nh("~");

    GripperTeleopNode node(nh);

    ros::spin();
    return 0;
}
