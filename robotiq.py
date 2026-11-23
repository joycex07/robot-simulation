import mujoco

model = mujoco.MjModel.from_xml_path(
    "mujoco_menagerie/robotiq_2f85/2f85.xml"
)

for i in range(model.nbody):
    print(
        i,
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    )