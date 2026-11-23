import mujoco
import mujoco.viewer
import time

import h5py
import numpy as np

## define hexagonal prism
def hex_prism(radius=0.02, height=0.04):
    angles = np.linspace(0, 2 * np.pi, 6, endpoint=False)
    xy = np.column_stack((radius * np.cos(angles),
                          radius * np.sin(angles)))

    bottom = np.column_stack((xy, np.full(6, -height / 2)))
    top = np.column_stack((xy, np.full(6, height / 2)))
    vertices = np.vstack((bottom, top))

    faces = []

    for i in range(1, 5):
        faces.extend([[0, i + 1, i], [6, 6 + i, 6 + i + 1]])

    for i in range(6):
        j = (i + 1) % 6
        faces.extend([[i, j, j + 6], [i, j + 6, i + 6]])

    return vertices, np.array(faces)

## load robot
fr3 = mujoco.MjSpec.from_file(
    "mujoco_menagerie/franka_fr3/fr3.xml"
)

gripper = mujoco.MjSpec.from_file(
    "mujoco_menagerie/robotiq_2f85/2f85.xml"
)

## load files
with h5py.File("trajectory.h5", "r") as f:
    joint_positions = f["observation/robot_state/joint_positions"][:]
    gripper_positions = f["observation/robot_state/gripper_position"][:]

with h5py.File("trajectory.h5", "r") as f:
    timestamps = f["observation/timestamp/robot_state/read_start"][:]


print("Gripper min:", np.min(gripper_positions))
print("Gripper max:", np.max(gripper_positions))
print("First 10:", gripper_positions[:10])
print("Last 10:", gripper_positions[-10:])

## calculate timestamps the robot follows
total_time = timestamps[-1] - timestamps[0]
total_time /= 1000

delta_t = delta_t = np.diff(timestamps) / 1000

## find scaling factor for grip
max_grip = 0.5682819408359043

## attach robot and grip, and make sure orientation matches the one in the video
attachment_site = fr3.site("attachment_site")

attachment_site.quat = [0.7071, 0, 0, 0.7071]

fr3.attach(
    gripper,
    site=attachment_site,
    prefix="robotiq_",
    suffix=""
)


## create hexagonal prism mesh
vertices, faces = hex_prism(radius=0.03, height=0.06)

hex_mesh = fr3.add_mesh(
    name="hex_mesh",
    uservert=vertices.flatten().tolist(),
    userface=faces.flatten().tolist()
)

## Create the object
cube = fr3.worldbody.add_body(
    name="hex_object",
    pos=[0.555, 0.143, 0.031]
)

cube.add_freejoint()

cube.add_geom(
    name="hex_geom",
    type=mujoco.mjtGeom.mjGEOM_MESH,
    meshname="hex_mesh",
    mass=0.05,
    rgba=[1, 0.8, 0, 1]
)


## add ground to prevent cube from falling under table
fr3.worldbody.add_geom(
    name="ground",
    type=mujoco.mjtGeom.mjGEOM_PLANE,
    pos=[0, 0, 0],
    size=[2, 2, 0.1],
    rgba=[0.7, 0.7, 0.7, 1]
)

## Create blue cup
cup = fr3.worldbody.add_body(
    name="blue_cup",
    pos=[0.65, 0.15, 0]
)

cup_radius = 0.05
cup_height = 0.10
wall_thickness = 0.005

# Cup bottom
cup.add_geom(
    name="cup_bottom",
    type=mujoco.mjtGeom.mjGEOM_CYLINDER,
    size=[cup_radius, wall_thickness / 2, 0],
    pos=[0, 0, wall_thickness / 2],
    rgba=[0.1, 0.3, 0.9, 1]
)

# Cup walls
for i in range(12):
    angle = 2 * np.pi * i / 12

    cup.add_geom(
        name=f"cup_wall_{i}",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[
            np.pi * cup_radius / 12,
            wall_thickness / 2,
            cup_height / 2
        ],
        pos=[
            cup_radius * np.cos(angle),
            cup_radius * np.sin(angle),
            cup_height / 2
        ],
        euler=[0, 0, np.degrees(angle) + 90],
        rgba=[0.1, 0.3, 0.9, 1]
    )
    
model = fr3.compile()

data = mujoco.MjData(model)

data.qpos[:7] = joint_positions[0]
data.ctrl[:7] = joint_positions[0]
data.ctrl[7] = 0

mujoco.mj_forward(model, data)

# Test hexagon physics
hex_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "hex_object"
)


for _ in range(500):
    mujoco.mj_step(model, data)


## data.ctrl[7] = 255 #close gripper

## determine starting and ending joint posiitons based on loaded info
start_positions = joint_positions[0].copy()
target_position = joint_positions[-1].copy()

## launch and open viewer
with mujoco.viewer.launch_passive(model, data) as viewer:

    viewer.cam.lookat[:] = [0, 0, 0.5]
    viewer.cam.distance = 1.5
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -20

    if viewer.is_running():

        for i in range(len(joint_positions) - 1):

            start_joints = joint_positions[i]
            end_joints = joint_positions[i + 1]

            start_grip = gripper_positions[i]
            end_grip = gripper_positions[i + 1]

            dt = delta_t[i]

            # Interpolate arm and gripper together
            for alpha in np.linspace(0, 1, 5, endpoint=False):

                if not viewer.is_running():
                    break

                joints = (1 - alpha) * start_joints + alpha * end_joints
                grip = (1 - alpha) * start_grip + alpha * end_grip

                data.ctrl[:7] = joints
                data.ctrl[7] = (grip / max_grip) * 255

                # Advance simulation
                n_steps = max(1, round((dt / 5) / model.opt.timestep))

                for _ in range(n_steps):
                    mujoco.mj_step(model, data)

                
                viewer.sync()
                time.sleep(dt / 5)