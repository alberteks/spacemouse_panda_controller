#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h> // gripper grasp; closes around object
#include <franka_gripper/MoveAction.h> // gripper move; to open it
#include <std_msgs/Bool.h> // we receive two bool msgs from spacemouse node

using GraspClient = actionlib::SimpleActionClient<franka_gripper::GraspAction>;
using MoveClient = actionlib::SimpleActionClient<franka_gripper::MoveAction>;

class GripperTeleopNode {
public:
    explicit GripperTeleopNode(ros::NodeHandle &nh) : grasp_client_("franka_gripper/grasp", true), move_client_("franka_gripper/move", true) {
        // --- Spacemouse input for gripper--hold close button to close, open button to open
        gripper_close_sub_ = nh.subscribe(
            "/spacemouse/gripper_close_held", 1, &GripperTeleopNode::gripperCloseCallback, this);
        gripper_open_sub_ = nh.subscribe(
            "/spacemouse/gripper_open_held", 1, &GripperTeleopNode::gripperOpenCallback, this);
        ROS_INFO("gripper_teleop_node: waiting for franka_gripper action servers...");
        if (!grasp_client_.waitForServer(ros::Duration(5.0))) {
            ROS_ERROR("gripper_teleop_node: franka_gripper/grasp action server not available.");
        }
        if (!move_client_.waitForServer(ros::Duration(5.0))) {
            ROS_ERROR("gripper_teleop_node: franka_gripper/move action server not available");
        }
        ROS_INFO("gripper_teleop_node: ready");
    }

private:
    void gripperOpenCallback(const std_msgs::BoolConstPtr &msg) {
        // only perform action once, when open button pressed rather than repeatedly as held. track w/ bool
        if (msg->data && !open_btn_pressed_) {
            open_btn_pressed_ = true;

            franka_gripper::MoveGoal goal;
            goal.width = 0.08; // max width
            goal.speed = 0.05; // speed in m/s for gripper motion
            move_client_.sendGoal(goal);
        }
        else if (!msg->data) {
            open_btn_pressed_ = false; // reset when btn released
        }
    }

    void gripperCloseCallback(const std_msgs::BoolConstPtr &msg) {
        // only perform action once, when close button pressed
        if (msg->data && !close_btn_pressed_) {
            close_btn_pressed_ = true;
            sendGraspGoal();
        }
        else if (!msg->data) {
            close_btn_pressed_ = false; // reset when btn released
        }
    }

    // method to set up necessary parameters and send grasp goal to action client
    void sendGraspGoal() {
        franka_gripper::GraspGoal goal;

        goal.width = 0.0; // setting target to 0m (fully closed) means it will automatically stop and grasp if it encounters resistance along way
        goal.speed = 0.05; // closing speed in m/s 
        goal.force = 10.0; // holding force in N

        // set up tolerances
        goal.epsilon.inner = 0.0; // allow it to fully close
        goal.epsilon.outer = 0.08; // max gripper width

        grasp_client_.sendGoal(goal); // send goal to action client
    }

    GraspClient grasp_client_;
    MoveClient move_client_;

    ros::Subscriber gripper_close_sub_;
    ros::Subscriber gripper_open_sub_;

    // track button states so the actions are only performed once on press
    bool open_btn_pressed_{false};
    bool close_btn_pressed_{false};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_teleop_node");
    ros::NodeHandle nh("~");

    GripperTeleopNode node(nh);

    ros::spin();
    return 0;
}
