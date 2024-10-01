import mujoco
import numpy as np
import time
import mujoco.viewer
import gymnasium as gym


# Load the MuJoCo model and create data object
model = mujoco.MjModel.from_xml_path("scene_1.xml")
data = mujoco.MjData(model)
viewer = mujoco.viewer.launch_passive(model, data)

# Simulation parameters
time_step = model.opt.timestep  # Get the simulation time step from the model
simulation_duration = 10  # Total duration of simulation in seconds
# n_steps = int(simulation_duration / time_step)
n_steps = 100000

# calculations
desired_position = np.array([0.2, 0, 1])

# PID controller
P = np.array([0.05,0.05,1])
D = np.array([0.005,0.005 ,0.5])
I = np.array([0.0000,0.0000,0.0])
g = 9.81


# Main simulation loop
for step in range(n_steps):
    # Example action: control rotor thrusts
    # action = np.array([0.00, 0.00, 4.39])  # just hover values

    R = data.xmat[1].reshape(3,3)  # Rotation matrix
    p = data.xpos[1]  # Position

    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = p

    ω_body = data.cvel[1][3:]
    ω_world = R @ ω_body

    print("body",ω_body)
    print("world",ω_world)

    error = desired_position - data.xpos[1]
    ades = (np.multiply(P, error) + 
            np.multiply(D, -ω_body)
            )

    action = np.array([(1/g)*(-ades[1]), (1/g)*(ades[0]), ades[2]])

    print(action)
    data.ctrl[:] = action

    mujoco.mj_step(model, data)
    viewer.sync()

    # Optional: Read sensor data, positions, etc., for analysis or feedback
    drone_position = data.qpos[:6]  # Example: get drone's position
    print(f"Step {step}: Position = {drone_position}")

    # Optional: Pause to match real-time if needed
    time.sleep(time_step)
    
# Close viewer if used
if viewer is not None:
    viewer.close()