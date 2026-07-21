import mujoco
import numpy as np
import time
import mujoco.viewer
import pyspacemouse
import os

class DampedLeastSquares:

    def __init__(self, model, data, step_size, damping, joint_names, actuator_names, site_name):
        self.model = model
        self.data = data
        self.step_size = step_size
        self.damping = damping
        
        # 1. Resolve site ID (for precise end-effector tracking)
        try:
            self.site_id = self.model.site(site_name).id
        except ValueError:
            raise ValueError(f"Site '{site_name}' not found in the model.")

        # 2. Resolve joint and actuator addresses dynamically to prevent index shifting
        self.joint_ids = [self.model.joint(name).id for name in joint_names]
        self.qpos_adrs = [self.model.jnt_qposadr[jid] for jid in self.joint_ids]
        self.dof_adrs = [self.model.jnt_dofadr[jid] for jid in self.joint_ids]
        self.actuator_ids = [self.model.actuator(name).id for name in actuator_names]
        
        # Preallocate Jacobian matrices
        self.jac_pos = np.zeros((3, self.model.nv)) 
        self.jac_rot = np.zeros((3, self.model.nv)) 
        
        # Regularization term (6x6) for Damped Least Squares
        self.reg_term = (self.damping ** 2) * np.eye(6)         

    def calculate(self, dx, dy, dz, droll, dpitch, dyaw):
        # Update kinematics to obtain current global state
        mujoco.mj_forward(self.model, self.data) 
        
        # Get exact position and rotation matrix of the end-effector site
        current_pos = self.data.site_xpos[self.site_id]
        current_rot = self.data.site_xmat[self.site_id].reshape(3, 3)
        
        # 3. Position Error (assuming dx, dy, dz is the desired delta in world coordinates)
        error_pos = np.array([dx, dy, dz])

        # 4. Rotation Error
        # Calculate target rotation matrix (intrinsic rotation about local axes)
        gen_rot_matrix = self._generate_rot_matrix(droll, dpitch, dyaw)
        target_rot_matrix = current_rot @ gen_rot_matrix
        
        # Direct rotation-matrix cross-product error (avoids quaternion sign-flip bugs)
        error_rot = 0.5 * (
            np.cross(current_rot[:, 0], target_rot_matrix[:, 0]) +
            np.cross(current_rot[:, 1], target_rot_matrix[:, 1]) +
            np.cross(current_rot[:, 2], target_rot_matrix[:, 2])
        )
        
        # Combine into a 6D spatial error vector
        error = np.concatenate([error_pos, error_rot])

        # 5. Compute the Jacobian at the end-effector site
        mujoco.mj_jacSite(self.model, self.data, self.jac_pos, self.jac_rot, self.site_id)
        J_full = np.vstack([self.jac_pos, self.jac_rot]) # (6, nv)
        
        # Slice columns to only include the degrees of freedom of your 7 controlled joints
        J = J_full[:, self.dof_adrs] # (6, 7)

        # Damped Least Squares pseudo-inverse
        J_pinv = J.T @ np.linalg.inv(J @ J.T + self.reg_term)

        # Compute update step for the controlled joint velocities
        joint_step = J_pinv @ error # (7,)

        # 6. Safely integrate targets using a full-sized delta vector
        dq = np.zeros(self.model.nv)
        dq[self.dof_adrs] = joint_step
        
        q_target = self.data.qpos.copy() 
        mujoco.mj_integratePos(self.model, q_target, dq, self.step_size)
        
        # Clip specifically the controlled joints to their XML limits
        for jid, qpos_adr in zip(self.joint_ids, self.qpos_adrs):
            if self.model.jnt_limited[jid]:
                low, high = self.model.jnt_range[jid]
                q_target[qpos_adr] = np.clip(q_target[qpos_adr], low, high)

        # Send target positions to the specific position actuator slots
        for act_id, qpos_adr in zip(self.actuator_ids, self.qpos_adrs):
            self.data.ctrl[act_id] = q_target[qpos_adr]

        return np.linalg.norm(error)

    def _generate_rot_matrix(self, roll, pitch, yaw):
        cx, sx = np.cos(roll), np.sin(roll)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cz, sz = np.cos(yaw), np.sin(yaw)
        
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        
        return Rx @ Ry @ Rz
def run_spacemouse_control():
    # Load your model (replace with your file path)
    model = mujoco.MjModel.from_xml_path("C:\\Users\\imtot\\.gemini\\spacemouse_panda_controller\\assets\\franka_arm\\mjx_panda.xml")
    data = mujoco.MjData(model)

    # Define your specific arm details as named in the MJCF XML
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    actuator_names = ["actuator1", "actuator2", "actuator3", "actuator4", "actuator5", "actuator6", "actuator7"]
    site_name = "gripper" # Must exist in your XML

    # Instantiate the controller
    controller = DampedLeastSquares(
        model=model,
        data=data,
        step_size=0.1,    # Scale factor for closing tracking error
        damping=0.01,     # Stability factor near singularities
        joint_names=joint_names,
        actuator_names=actuator_names,
        site_name=site_name
    )

    # Scale factors translating SpaceMouse raw output [-1.0, 1.0] to physical increments
    translation_scale = 0.1  # Max 5 millimeters per command step
    rotation_scale = 0.1     # Max ~0.3 degrees per command step

    # Open the MuJoCo Passive Viewer and connect the SpaceMouse
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Default SpaceMouse connection (Option 2 - no AxisConvention argument)
        with pyspacemouse.open() as device:
            print("--------------------------------------------------")
            print("SpaceMouse Connected!")
            print("Press Ctrl+C or close the viewer window to exit.")
            print("--------------------------------------------------")
            
            # Run high-level controller at 100 Hz (dt = 0.01s)
            controller_dt = 0.01 
            physics_dt = model.opt.timestep  # usually 0.002s (500 Hz)
            steps_per_control_cycle = int(controller_dt / physics_dt)

            while viewer.is_running():
                cycle_start = time.time()

                # Read the latest device input
                state = device.read()

                # --------------------------------------------------------------
                # Manual Coordinate Frame Alignment
                # --------------------------------------------------------------
                # Since we are not using the AxisConvention utility, the raw values 
                # map directly to how your OS sees the controller hardware. 
                # If pushing forward on the controller cap moves the robot in an 
                # unexpected direction, modify the equations below:
                # E.g., to swap X and Y or invert Z:
                #   dx = -state.y * translation_scale
                #   dy = state.x * translation_scale
                #   dz = -state.z * translation_scale
                # --------------------------------------------------------------
                dx = state.y * translation_scale
                dy = state.x * translation_scale
                dz = state.z * translation_scale
                
                droll = state.roll * rotation_scale
                dpitch = state.pitch * rotation_scale
                dyaw = state.yaw * rotation_scale

                # Calculate the joint angle updates
                controller.calculate(dx, dy, dz, droll, dpitch, dyaw)

                # Step the physics engine to let actuators drive the arm to targets
                for _ in range(steps_per_control_cycle):
                    mujoco.mj_step(model, data)

                # Sync visual state with simulation state
                viewer.sync()

                # Maintain control loop timing (100 Hz)
                elapsed = time.time() - cycle_start
                sleep_time = controller_dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

if __name__ == "__main__":
    run_spacemouse_control()