import mujoco
import numpy as np
import time
import mujoco.viewer
import gymnasium as gym


# Load the MuJoCo model and create data object
model = mujoco.MjModel.from_xml_path("scene_2.xml")
data = mujoco.MjData(model)

viewer = mujoco.viewer.launch_passive(model, data)

# Simulation parameters
time_step = model.opt.timestep  # Get the simulation time step from the model
# time_step2 = model2.opt.timestep  # Get the simulation time step from the model

simulation_duration = 10  # Total duration of simulation in seconds
# n_steps = int(simulation_duration / time_step)
n_steps = 100000

print()
# Main simulation loop
for step in range(n_steps):
    # Example action: control rotor thrusts
    action = np.array([0.00, 0.00, 0.00, 0.00, 4.39, 4.39])  # just hover values

    # Apply the action (modify data.qfrc_applied or actuator controls)
    data.ctrl[:] = action

    # Step the simulation forward
    mujoco.mj_step(model, data)

    viewer.sync()

    # Optional: Read sensor data, positions, etc., for analysis or feedback
    drone_position = data.qpos[:6]  # Example: get drone's position

    print(f"Step {step}: Position = {drone_position}")
    # print(f"Step {step}: Position2 = {drone_position2}")

    # Optional: Pause to match real-time if needed
    time.sleep(time_step)
    
# Close viewer if used
if viewer is not None:
    viewer.close()