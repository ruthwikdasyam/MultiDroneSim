import mujoco
import numpy as np
import time
import mujoco.viewer


# Load the MuJoCo model and create data object
model = mujoco.MjModel.from_xml_path("scene_4.xml")
data = mujoco.MjData(model)


# Viewer (optional for visualization)
# viewer = MujocoViewer(model, data)
viewer = mujoco.viewer.launch_passive(model, data)

# Simulation parameters
time_step = model.opt.timestep  # Get the simulation time step from the model

simulation_duration = 10  # Total duration of simulation in seconds
# n_steps = int(simulation_duration / time_step)
n_steps = 100000

print()
# Main simulation loop
for step in range(n_steps):
    # Example action: control rotor thrusts
    # Replace this with your control logic or RL action
    # action = np.array([-0.05, 0.53, 0.3])  # roll, pitch, vertical acceleration
    fly = 4.37
    fly2 = -4.37
    action = np.array([fly, fly, fly, fly])  # just hover values

    # Apply the action (modify data.qfrc_applied or actuator controls)
    data.ctrl[:] = action

    # Step the simulation forward
    mujoco.mj_step(model, data)

    # Optional: Render the scene if using a viewer
    if viewer is not None:
        viewer.sync()

    # Optional: Read sensor data, positions, etc., for analysis or feedback
    drone_position = data.qpos[:6]  # Example: get drone's position
    print(f"Step {step}: Position = {drone_position}")

    # Optional: Pause to match real-time if needed
    time.sleep(time_step)
    
# Close viewer if used
if viewer is not None:
    viewer.close()