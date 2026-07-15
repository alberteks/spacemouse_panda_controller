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
z=0
p=0
w=0
r=0

a=0

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
            z += state.z/tune
            p += state.pitch/tune
            w += state.yaw/tune
            r += state.roll/tune
            a += state.buttons[0]/tune
            a -= state.buttons[1]/tune

            #mujoco.mjs_setToPosition()
            mydata.ctrl[0] = w
            mydata.ctrl[2] = r
            mydata.ctrl[5] = x

            mydata.ctrl[3] = p
            mydata.ctrl[1] = y
            mydata.ctrl[6] = z

            mydata.ctrl[7] = a
            
            mujoco.mj_step(mymodel, mydata)
            viewer.sync()

