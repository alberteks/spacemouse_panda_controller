from __future__ import print_function 
import rospy 
import actionlib 
import pyspacemouse
import sys
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal 
from trajectory_msgs.msg import JointTrajectoryPoint 

from spatialmath import SE3
import numpy as np
import roboticstoolbox as rtb

# Damping factor: higher = slower movement
TUNE = 1000 
#Default starting joint config for the Panda
START_JOINTS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
# creates a ROS action client for the robot arm controller
def create_client():
    client = actionlib.SimpleActionClient('panda_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for robot controller...")
    client.wait_for_server()
    return client
#moves robot to starting configuration before teleop begins
def move_to_start(client, q_start):
    """Command the robot to move to the specific start joints before teleop begins."""
    rospy.loginfo("Moving to initial set position...")
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 
                                   'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
    
    point = JointTrajectoryPoint()
    point.positions = q_start
    point.time_from_start = rospy.Duration(5.0) # Take 5 seconds to reach home
    goal.trajectory.points.append(point)
    
    # We use 'and_wait' because teleop shouldn't start until we are in position
    client.send_goal_and_wait(goal)
    rospy.loginfo("Robot is at start position. Start teleop.")
#creates a ROS action client for the robot arm controller
def main():
    rospy.init_node('spacemouse_teleop_panda')
    arm_client = create_client()
    
    # Load robot model for IK
    panda = rtb.models.URDF.Panda()
    
     # 1. MOVE PHYSICALLY: Go to the set start position
    move_to_start(arm_client, START_JOINTS)
    
    # 2. SYNC MATH: Initialize Cartesian variables from that set joint configuration
    current_q = np.array(START_JOINTS)
    T_start = panda.fkine(current_q)
    x, y, z = T_start.t
    
    print("SpaceMouse active. Move the device to control the robot arm.")
    print("Hold 'Shift' or your mapped exit button to stop, or Ctrl+C")

    try:
        with pyspacemouse.open() as device:
            while not rospy.is_shutdown():
                # 1. Read SpaceMouse
                state = device.read()
                
                # 2. Accumulate position based on mouse deflection
                x += state.x / TUNE
                y += state.y / TUNE
                z += state.z / TUNE
                
                # 3. Calculate IK for the new position
                # We use the previous joint values (current_q) as a starting seed for the solver
                target_pose = SE3(x, y, z) * SE3.OA([0, 1, 0], [0, 0, -1]) # Keeps gripper pointing down
                ik_solution = panda.ikine_LM(target_pose, q0=current_q)
                
                if ik_solution.success:
                    joint_goal = ik_solution.q
                    current_q = joint_goal # Update seed for next iteration
                    
                    # 4. Construct ROS Goal
                    arm_goal = FollowJointTrajectoryGoal()
                    arm_goal.trajectory.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 
                                                       'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
                    
                    point = JointTrajectoryPoint()
                    point.positions = joint_goal
                    
                    # VERY IMPORTANT: Short duration for smooth teleop
                    # If this is too long, the robot lags. If too short, it stutters.
                    point.time_from_start = rospy.Duration(0.1) 
                    
                    arm_goal.trajectory.points.append(point)
                    
                    # 5. Send goal (non-blocking)
                    # We don't use 'wait_for_result' because we need to send the next point ASAP
                    arm_client.send_goal(arm_goal)
                else:
                    rospy.logwarn("IK Solver failed to find a valid position at this coordinate")

                # Limit frequency to match ROS control capabilities (approx 10-30Hz for this method)
                rospy.sleep(0.05)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()