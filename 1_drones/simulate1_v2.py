import mujoco
import numpy as np
import time
import mujoco.viewer

# Load the MuJoCo model and create data object
model = mujoco.MjModel.from_xml_path("scene_1.xml")
data = mujoco.MjData(model)
viewer = mujoco.viewer.launch_passive(model, data)

# Simulation parameters
time_step = model.opt.timestep  # Get the simulation time step from the model
simulation_duration = 10  # Total duration of simulation in seconds
n_steps = 100000

# PID controller parameters
P = np.array([0.000001, 0.000001, 1.0])
D = np.array([0.00, 0.00, 0.5])
I = np.array([0.00, 0.00, 0.1])
integral_error = np.zeros(3)
g = 9.81

# Desired position for the drone
desired_position = np.array([0.2, 0.0, 1.0])

# Main simulation loop
for step in range(n_steps):
    # Extract rotation and position
    R = data.xmat[1].reshape(3, 3)  # Rotation matrix
    p = data.xpos[1]  # Position
    ω_body = data.cvel[1][3:]  # Angular velocity in the body frame

    # Calculate error in position
    error = desired_position - p

    # Update integral term
    integral_error += error * time_step

    # PID controller for desired acceleration
    ades = (np.multiply(P, error) +
            np.multiply(D, -ω_body) +  # Apply damping in the body frame
            np.multiply(I, integral_error))

    # Convert desired acceleration to control action (thrust and pitch/roll control)
    thrust = ades[2]  # Altitude control (z-axis)
    roll_pitch_control = np.array([ades[0], ades[1]])  # Roll and pitch control

    # Convert control to actuator inputs
    action = np.array([(1/g) * (-roll_pitch_control[1]),  # Roll
                       (1/g) * (roll_pitch_control[0]),   # Pitch
                       thrust])                           # Thrust (z)

    data.ctrl[:] = action

    # Step the simulation
    mujoco.mj_step(model, data)
    viewer.sync()

    # Optional: Read sensor data, positions, etc., for analysis or feedback
    drone_position = data.qpos[:6]  # Get drone's position
    print(f"Step {step}: Position = {drone_position}")

    # Optional: Pause to match real-time if needed
    time.sleep(time_step)

# Close viewer if used
if viewer is not None:
    viewer.close()
