import mujoco
import mujoco.viewer
import numpy as np

# Load the model
model = mujoco.MjModel.from_xml_path('scene_v1.xml')
data = mujoco.MjData(model)

# Viewer setup
viewer = mujoco.viewer.launch_passive(model, data)
# Enable coordinate frame visualization
viewer.frame = mujoco.mjtFrame.mjFRAME_BODY  # This enables the axes frame
while True:
    # Apply control to each model
    # data.ctrl[model.actuator('joint1').id] = some_control_value_for_model1
    # data.ctrl[model.actuator('joint2').id] = some_control_value_for_model2
    # action = np.array([0.0000])  # just hover values

    # Apply the action (modify data.qfrc_applied or actuator controls)
    # data.ctrl[:] = action
    # Step simulation
    mujoco.mj_step(model, data)

    # Render
    viewer.sync()
