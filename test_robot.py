import mujoco
import mujoco.viewer
import time
import numpy as np
import h5py

# joint_posiitons: records joint posiitons at each stamp
with h5py.File("trajectory.h5", "r") as f:
    joint_positions = f["observation/robot_state/joint_positions"][:]

# Load the robot model
model = mujoco.MjModel.from_xml_path(
    "mujoco_menagerie/franka_emika_panda/scene.xml"
)

# Initialize the simulation
data = mujoco.MjData(model)

for i in range(model.njnt):
    print(i, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i))

# Initial robot configuration
start_pose = joint_positions[0].copy()

# Target robot configuration
target_pose = joint_positions[-1].copy()

with mujoco.viewer.launch_passive(model, data) as viewer:

    for i in range(len(joint_positions) - 1):
        if not viewer.is_running():
            break

        start_pose = joint_positions[i]
        end_pose = joint_positions[i + 1]

        # Generate 5 intermediate positions
        for alpha in np.linspace(0, 1, 5, endpoint=False):
            if not viewer.is_running():
                break

            data.qpos[:7] = (1 - alpha) * start_pose + alpha * end_pose

            mujoco.mj_forward(model, data)
            viewer.sync()

            time.sleep(0.014)

    # Show the final recorded position
    if viewer.is_running():
        data.qpos[:7] = joint_positions[-1]
        mujoco.mj_forward(model, data)
        viewer.sync()


for name in ["finger_joint1", "finger_joint2"]:
    joint_id = model.joint(name).id
    print(name, model.jnt_range[joint_id])

