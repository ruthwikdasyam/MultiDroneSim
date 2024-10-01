
import mujoco
import mujoco_viewer

# Path to your XML file
xml_path = "scene.xml"

# Load the MuJoCo model from the XML file
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Create the viewer
viewer = mujoco_viewer.MujocoViewer(model, data)

# Run the simulation until the viewer is closed
while viewer.is_alive:
    mujoco.mj_step(model, data)
    viewer.render()

# Close the viewer
viewer.close()
