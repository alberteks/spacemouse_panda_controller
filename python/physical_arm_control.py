import argparse
import time
import pyspacemouse
import numpy as np
from pylibfranka import CartesianPose, ControllerMode, RealtimeConfig, Robot

dx=0
dy=0
dz=0
tune = 1000 # damping factor for mouse input

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="localhost", help="Robot IP address")
    args = parser.parse_args()

    # Connect to robot
    robot = Robot(args.ip, RealtimeConfig.kIgnore)

    try:
        # Set collision behavior
        lower_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        upper_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        lower_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        upper_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

        robot.set_collision_behavior(
            lower_torque_thresholds,
            upper_torque_thresholds,
            lower_force_thresholds,
            upper_force_thresholds,
        )

        # First move the robot to a suitable joint configuration
        print("Please make sure to have the user stop button at hand!")
        input("Press Enter to continue...")

        active_control = robot.start_cartesian_pose_control(ControllerMode.JointImpedance)
        time_elapsed = 0.0
        motion_finished = False
        
        robot_state, duration = active_control.readOnce()
        initial_cartesian_pose = robot_state.O_T_EE

        while not motion_finished:
            with pyspacemouse.open() as device:
                # get mouse input
                state = device.read()
                dx = state.x/tune
                dy = state.y/tune
                dz = state.z/tune
                # Read robot state and duration
                robot_state, duration = active_control.readOnce()

                # Update time
                time_elapsed += duration.to_sec()

                # Update joint positions
                new_cartesian_pose = initial_cartesian_pose.copy()
                new_cartesian_pose[12] += dx  # x position
                new_cartesian_pose[13] += dy # y position
                new_cartesian_pose[14] += dz  # z position

                # Set joint positions
                cartesian_pose = CartesianPose(new_cartesian_pose)

                # Set motion_finished flag to True on the last update
                if time_elapsed >= 15.0:
                    cartesian_pose.motion_finished = True
                    motion_finished = True
                    print("Finished motion, shutting down motion control.")

                # Send command to robot
                active_control.writeOnce(cartesian_pose)

    except Exception as e:
        print(f"Error occurred: {e}")
        if robot is not None:
            robot.stop()
        return -1