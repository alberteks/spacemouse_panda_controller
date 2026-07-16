import pyspacemouse
import mujoco
import mujoco.viewer
import os
import time
from dls_algorithm import DampedLeastSquares

dx=0
dy=0
dz=0
dpitch=0
dyaw=0
droll=0

a=0

tune = 1000

script_dir = os.path.dirname(__file__)
xml_path = os.path.join(script_dir, "..", "assets", "franka_arm", "mjx_single_cube.xml")
mymodel = mujoco.MjModel.from_xml_path(xml_path)
mydata = mujoco.MjData(mymodel)

ik_solver = DampedLeastSquares(model=mymodel, data=mydata, step_size=0.1, damping=0.01) # instantiate inverse kinematics algo

end_effector_id = mymodel.body('hand').id # we want to move the hand end effector to the target position

error_magnitude = float('inf') # set the error magnitude to a really high number first. then we will update it in the algo

timestep = mymodel.opt.timestep # setup time tracking

with pyspacemouse.open() as device:
    with mujoco.viewer.launch_passive(mymodel,mydata) as viewer:
        while viewer.is_running():
            step_start = time.time()
            state = device.read() # read spacemouse info once per step
            # previous code for moving each joint independently based on input below
            """
            x += state.x/tune
            y += state.y/tune
            z += state.z/tune
            p += state.pitch/tune
            w += state.yaw/tune
            r += state.roll/tune

            #mujoco.mjs_setToPosition()
            mydata.ctrl[0] = w
            mydata.ctrl[2] = r
            mydata.ctrl[5] = x

            mydata.ctrl[3] = p
            mydata.ctrl[1] = y
            mydata.ctrl[6] = z
            """
            # change in position and rotation as indicated by mouse
            dx = state.x/tune
            dy = state.y/tune
            dz = state.z/tune
            dpitch = state.pitch/tune
            dyaw = state.yaw/tune
            droll = state.roll/tune

            # update buttons to open/close hand
            a += state.buttons[0]/tune
            a -= state.buttons[1]/tune
            mydata.ctrl[7] = a

            print(dx)
            ik_solver.calculate(body_id=end_effector_id, dx=dx, dy=dy, dz=dz, droll=droll, dpitch=dpitch, dyaw=dyaw)
            mujoco.mj_step(mymodel, mydata)
            viewer.sync()

            time_spent = time.time() - step_start
            if time_spent < timestep:
                time.sleep(timestep - time_spent)