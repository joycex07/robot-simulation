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

with h5py.File("trajectory.h5", "r") as f:
    joint_positions = f["observation/robot_state/joint_positions"][:]

with h5py.File("trajectory.h5", "r") as f:
    timestamps = f["observation/timestamp/robot_state/read_start"][:]

total_time = timestamps[-1] - timestamps[0]
total_time /= 1000

delta_t = delta_t = np.diff(timestamps) / 1000


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
data.ctrl[7] = 255 #close gripper

start_positions = joint_positions[0].copy()
target_position = joint_positions[-1].copy()



# Open the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:

    # Position camera so you can see the robot
    viewer.cam.lookat[:] = [0, 0, 0.5]
    viewer.cam.distance = 1.5
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -20

    start_time = time.perf_counter()

    while viewer.is_running():
        for i in range(len(joint_positions)):
            data.qpos[:7] = joint_positions[i]
            mujoco.mj_forward(model, data)
            viewer.sync()

            if i < len(delta_t):
                time.sleep(delta_t[i])
