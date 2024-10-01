import mujoco
import mujoco.viewer
import numpy as np
import time

# Load the model
model = mujoco.MjModel.from_xml_path('scene_v1.xml')
data = mujoco.MjData(model)

# Viewer setup
viewer = mujoco.viewer.launch_passive(model, data)
time_step = model.opt.timestep 
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
    if viewer is not None:
        viewer.sync()
    time.sleep(time_step)
    

if viewer is not None:
    viewer.close()