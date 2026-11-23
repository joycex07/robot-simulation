import mujoco
import mujoco.viewer
import time

import h5py
import numpy as np

# Load both robot specifications
fr3 = mujoco.MjSpec.from_file(
    "mujoco_menagerie/franka_fr3/fr3.xml"
)

gripper = mujoco.MjSpec.from_file(
    "mujoco_menagerie/robotiq_2f85/2f85.xml"
)

# Find the attachment site on the FR3
attachment_site = fr3.site("attachment_site")

# Attach the gripper to the FR3
fr3.attach(
    gripper,
    site=attachment_site,
    prefix="robotiq_",
    suffix=""
)
model = fr3.compile()

print("\nActuators:")

for i in range(model.nu):
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        i
    )

    print(i, name, model.actuator_ctrlrange[i])

data = mujoco.MjData(model)

# Set the gripper actuator command
data.ctrl[7] = 255



# Open the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:

    # Position camera so you can see the robot
    viewer.cam.lookat[:] = [0, 0, 0.5]
    viewer.cam.distance = 1.5
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -20

    while viewer.is_running():

        # Open the gripper
        data.ctrl[7] = 0

        for _ in range(500):
            if not viewer.is_running():
                break

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

        # Close the gripper
        data.ctrl[7] = 255

        for _ in range(500):
            if not viewer.is_running():
                break

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)