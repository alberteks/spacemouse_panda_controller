import mujoco
import mujoco.viewer
import os
import time

script_dir = os.path.dirname(__file__)
xml_path = os.path.join(script_dir, "..", "assets", "franka_sim", "franka_panda.xml")

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

def main():
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(model, data)
            viewer.sync()

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()