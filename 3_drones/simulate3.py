import mujoco
import numpy as np
import time
import mujoco.viewer


# Load the MuJoCo model and create data object
model = mujoco.MjModel.from_xml_path("scene_3.xml")
data = mujoco.MjData(model)

# Viewer (optional for visualization)
viewer = mujoco.viewer.launch_passive(model, data)


# Simulation parameters
time_step = model.opt.timestep  # Get the simulation time step from the model
print(time_step)

simulation_duration = 10  # Total duration of simulation in seconds
n_steps = 1000
step=0
print()
# Main simulation loop
while True:
    # if step==(n_steps):
    #     mujoco.mj_resetData(model, data)
    #     step=0
    step += 1
    # Example action: control rotor thrusts
    action = np.array([3.47, 3.47, 3.47])  # just hover values
    # action = np.array([0.00, 0.00, 0.00, 0.00, 6.09, 5.09])  # just hover values

    # Apply the action (modify data.qfrc_applied or actuator controls)
    data.ctrl[:] = action

    # Step the simulation forward
    mujoco.mj_step(model, data)
    print("Body 1",data.xpos[1])

    # mujoco.mj_step(model2, data2)

    # Optional: Render the scene if using a viewer
    if viewer is not None:
        viewer.sync()

    # Optional: Read sensor data, positions, etc., for analysis or feedback
    drone_position = data.qpos[:6]  # Example: get drone's position
    # drone_position2 = data2.qpos[:3]  # Example: get drone's
    print(f"Step {step}: Position = {drone_position}")
    # print(f"Step {step}: Position2 = {drone_position2}")

    # Optional: Pause to match real-time if needed
    time.sleep(time_step)
    
    # Close viewer if used
    
if viewer is not None:
    viewer.close()