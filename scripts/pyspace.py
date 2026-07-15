import pyspacemouse
import mujoco
import mujoco.viewer

def numformat(f):
    num = f
    sx = '{:.2f}'.format(f)
    if len(sx)<=4:
        sx = '+'+sx
    return sx


tune = 1000
mymodel = mujoco.MjModel.from_xml_path("C:\\Users\\ali-28\\schmallan\\spacemouse_panda_controller\\assets\\franka_arm\\mjx_single_cube.xml")
mydata = mujoco.MjData(mymodel)
with pyspacemouse.open() as device:
    with mujoco.viewer.launch_passive(mymodel,mydata) as viewer:
        while viewer.is_running():

            x = state.x/tune
            y = state.y/tune
            z = state.z/tune
            p = state.pitch/tune
            w = 

            state = device.read()    
            #mujoco.mjs_setToPosition()
            mydata.ctrl[0] += state.yaw/tune
            mydata.ctrl[2] += state.roll/tune
            mydata.ctrl[4] += state.x/tune

            mydata.ctrl[3] += state.pitch/tune
            mydata.ctrl[1] += state.y/tune
            mydata.ctrl[5] += state.z/tune
            
            mujoco.mj_step(mymodel, mydata)
            viewer.sync()

