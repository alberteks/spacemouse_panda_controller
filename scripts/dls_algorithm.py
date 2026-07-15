import mujoco
import numpy as np

class DampedLeastSquares:

    def __init__(self, model, data, step_size, damping):
        self.model = model
        self.data = data
        self.step_size = step_size
        self.damping = damping
        # matrices that will be used later for calculation
        self.jac_pos = np.zeros((3, self.model.nv)) # translational jacobian
        self.jac_rot = np.zeros((3, self.model.nv)) # rotational jacobian
        self.reg_term = (self.damping ** 2) * np.eye(6)  # creates modified version of identity matrix as regularization term; prevents issues in computation when two joints align linearly         
        # note above: @ is python's operator for matrix multiplication

    """
    function we might need in case we calculate joint limits ourselves
    def check_joint_limit(self, joint_pos): # checks joint limits
        for i in range(len(joint_pos)): # for each direction of motion for the joint, checks if it is at limit
            joint_pos[i] = max(self.model.jnt_range[i][0], min(joint_pos[i], self.model.jnt_range[i][1]))
    """
            
    def calculate(self, body_id, dx, dy, dz, droll, dpitch, dyaw):
        # initializing robot state
        mujoco.mj_forward(self.model, self.data) #  update kinematics to get current state
        current_pos = self.data.xpos[body_id] # get current global cartesian coords (x,y,z)
        current_rot = self.data.xmat[body_id].reshape(3, 3) # get rotational matrix for other 3 DOF
        
        # positional transformation target setting
        target_pos = current_pos + np.array([dx, dy, dz]) # sets target position based on given change from controller input
        # compute position error vector
        error_pos = target_pos - current_pos # calculate error in cartesian position

        # rotational transformation target setting
        gen_rot_matrix = self._generate_rot_matrix(droll, dpitch, dyaw)
        target_rot_matrix = current_rot @ gen_rot_matrix # sets target rotation based on given change from controller input
        # now use quaternions--4d vectors in linear space--for easier calculations with rotation
        current_quat = np.zeros(4)
        mujoco.mju_mat2Quat(current_quat, current_rot.flatten())
        target_quat = np.zeros(4)
        mujoco.mju_mat2Quat(target_quat, target_rot_matrix.flatten()) # set target rotation quaternion
        # compute rotation error vector using current and target quaternions
        error_rot = np.zeros(3)
        mujoco.mju_subQuat(error_rot, target_quat, current_quat) # vector containing rotational error (difference in orientation)

        # combine position and rotation error vectors into one error matrix
        error = np.concatenate([error_pos, error_rot])

        # calculate jacobian matrix for DLS
        mujoco.mj_jacBody(self.model, self.data, self.jac_pos, self.jac_rot, body_id)
        J = np.vstack([self.jac_pos, self.jac_rot]) # full 6 by nv matrix for our 6 DOF

        # calculate regularized gradient descent step using damped least squares solution to normal equation
        J_pinv = J.T @ np.linalg.inv(J @ J.T + self.reg_term)

        # compute update step for joint velocity
        joint_step = J_pinv @ error

        q_target = self.data.qpos.copy() # integrate changes into copy of qpos
        mujoco.mj_integratePos(self.model, q_target, joint_step * self.step_size, 1.0)
        
        # clip to joint limits and then push to ctrl to actually move arm
        for jnt_id in range(self.model.njnt):
            if self.model.jnt_limited[jnt_id]:  # check if joint limits are enabled in XML
                qpos_adr = self.model.jnt_qposadr[jnt_id]
                # single degree-of-freedom joints use exactly one qpos slot
                if self.model.jnt_type[jnt_id] in [mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE]:
                    low, high = self.model.jnt_range[jnt_id]
                    q_target[qpos_adr] = np.clip(q_target[qpos_adr], low, high)

        # move each of the joints based on qpos targets. 
        # mj_step() will nudge qpos closer to ctrl each time it is called, 
        # moving towards the end effector target over time
        self.data.ctrl[:7] = q_target[:7] 

        # return magnitude of error to determine when we reach target
        return np.linalg.norm(error)

    def _generate_rot_matrix(self, roll, pitch, yaw): # creates rotation matrix to help find target rotational change
        cx, sx = np.cos(roll), np.sin(roll)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cz, sz = np.cos(yaw), np.sin(yaw)
        
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        
        return Rx @ Ry @ Rz