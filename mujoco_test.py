import mujoco
import mujoco.viewer

xml = """
<mujoco>
    <worldbody>
        <body pos="0 0 0 ">
            <joint name="j1" type="hinge" axis="0 1 0"/>

            <geom type="cylinder" size="0.1 0.1" mass="1"/>
        </body>
    </worldbody>

    <actuator>
        <motor joint="j1" ctrlrange="-10 10"/>
    </actuator>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

data.ctrl[0] = 2.0

print("actuators:", model.nu)
print("ctrl array:", data.ctrl)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        data.ctrl[0] = 2.0
        mujoco.mj_step(model, data)
        viewer.sync()