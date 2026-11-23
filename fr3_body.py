import mujoco

model = mujoco.MjModel.from_xml_path(
    "mujoco_menagerie/franka_fr3/fr3.xml"
)

for i in range(model.nbody):
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        i
    )
    print(i, name)

print("\nFR3 sites:")

for i in range(model.nsite):
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        i
    )
    print(i, name)

site = model.site("attachment_site")

print("Position:", site.pos)
print("Orientation:", site.quat)

