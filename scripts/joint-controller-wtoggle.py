import pyspacemouse
import mujoco
import mujoco.viewer
import os

def numformat(f):
    num = f
    sx = '{:.2f}'.format(f)
    if len(sx)<=4:
        sx = '+'+sx
    return sx
x=0
y=0

# Split the 'z' accumulator into two variables to prevent joint jumping on toggle
z_joint4=0  # Target position for Joint 5
z_joint6=0  # Target position for Joint 7

p=0
w=0
r=0

a=0

# Toggle state variables
toggle_mode = 0          # 0 = Control Joint 5 & Add gripper; 1 = Control Joint 7 & Subtract gripper
last_button_1_state = 0  # For left button edge detection

tune = 1000
script_dir = os.path.dirname(__file__)
xml_path = os.path.join(script_dir, "..", "assets", "franka_arm", "mjx_single_cube.xml")
mymodel = mujoco.MjModel.from_xml_path(xml_path)

mydata = mujoco.MjData(mymodel)
with pyspacemouse.open() as device:
    with mujoco.viewer.launch_passive(mymodel,mydata) as viewer:
        while viewer.is_running():
            state = device.read()    

            x += state.x/tune
            y += state.y/tune
            
            # --- TOGGLE LOGIC ON LEFT BUTTON (buttons[1]) ---
            if state.buttons[1] == 1 and last_button_1_state == 0:
                toggle_mode = 1 - toggle_mode
            last_button_1_state = state.buttons[1]

            if toggle_mode == 0:
                z_joint4 += state.z/tune
            else:
                z_joint6 += state.z/tune

            p += state.pitch/tune
            w += state.yaw/tune
            r += state.roll/tune
            
            
            if toggle_mode == 0:
                a += state.buttons[0]/tune
            else:
                a -= state.buttons[0]/tune

            #mujoco.mjs_setToPosition()
            mydata.ctrl[0] = w
            mydata.ctrl[2] = r
            mydata.ctrl[5] = x

            mydata.ctrl[3] = p
            mydata.ctrl[1] = y
            
            mydata.ctrl[4] = z_joint4  # Joint 5 is frozen
            mydata.ctrl[6] = z_joint6  # Joint 7 is frozen

            mydata.ctrl[7] = a
            
            mujoco.mj_step(mymodel, mydata)
            viewer.sync()