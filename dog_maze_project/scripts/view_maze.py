import mujoco
import mujoco.viewer

# Load your XML maze file
model = mujoco.MjModel.from_xml_path("assets/maze_env.xml")
data = mujoco.MjData(model)

# Launch the built-in MuJoCo viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
