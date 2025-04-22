import mujoco
import mujoco.viewer
import numpy as np

# Load a built-in MuJoCo model (a simple ball)
model = mujoco.MjModel.from_xml_string("""
<mujoco>
  <worldbody>
    <body name="ball" pos="0 0 1">
      <joint type="free"/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
""")

data = mujoco.MjData(model)

# Launch interactive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
