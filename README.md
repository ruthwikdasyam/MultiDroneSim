# Multi-Drone Simulation - MuJoCo

Requires **MuJoCo 2.2.2** or later.

## Overview

The files consist of simulations of drones in MuJoCo environments.  
This package includes a simplified robot description (MJCF) of the Crazyflie 2 model from [Bitcraze](https://www.bitcraze.io/). It is derived from the publicly available [ROS description](https://github.com/whoenig/crazyflie_ros).

<p float="left">
  <img src="images/1_drones.png" width="200">
  <img src="images/2_drones.png" width="200">
  <img src="images/3_drones.png" width="200">
  <img src="images/4_drones.png" width="200">
</p>

## Installation

The required packages are included in the Conda environment file. Use this command to create a Conda environment in your system:

```bash
conda env create -f environment.yml


## Package Description

- **1_drones folder**: 
  - Contains a MuJoCo environment with a closed-loop PID controller that maintains the drone at a certain height.
  - The **PID values** and **goal positions** can be modified in the `simulate1_v1.py` file.
  - The `simulate1_v1.py` file defines the goal position in 3D space, enabling the drone to move in X, Y, and Z directions.
  - The `simulate1_v2.py` file defines the goal position only in the Z-axis, causing the drone to lift and hold its position at the goal height without changing its X and Y positions.

- **2_drones folder**: 
  - Contains the `simulate2.py` file, which simulates two drones attached to either end of a slab with a spherical object in the middle.
  - This setup resembles a cooperative multi-drone transport system.
  - A minimal fly-off thrust is applied to both drones equally, lifting the slab and balancing the ball in the middle.

- **3_drones folder**: 
  - Contains the `simulate3.py` file, demonstrating the same cooperative transport simulation using three drones instead of two.

- **4_drones folder**: 
  - Contains the `simulate4.py` file, demonstrating the cooperative transport simulation with four drones.
