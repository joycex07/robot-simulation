import mujoco
import mujoco.viewer
import time

# Load the Franka FR3 model
model = mujoco.MjModel.from_xml_path(
    "mujoco_menagerie/franka_fr3/scene.xml"
)

data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(model.opt.timestep)