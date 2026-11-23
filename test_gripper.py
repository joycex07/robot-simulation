import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path(
    "mujoco_menagerie/robotiq_2f85/scene.xml"
)

data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        viewer.sync()
        time.sleep(0.03)