import mujoco
import mujoco.viewer
import time

xml = """
<mujoco model="first_simulation">
    <option gravity="0 0 -9.81"/>

    <worldbody>
        <geom name="floor"
              type="plane"
              size="2 2 0.1"
              rgba="0.8 0.8 0.8 1"/>

        <body name="cube" pos="0 0 1">
            <freejoint/>
            <geom type="box"
                  size="0.05 0.05 0.05"
                  mass="0.1"
                  rgba="1 0.8 0 1"/>
        </body>
    </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)


with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(model.opt.timestep)
    