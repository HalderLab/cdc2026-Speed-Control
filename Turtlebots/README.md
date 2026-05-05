This repository contains the official experiment code for the paper: **"On Feedback Speed Control for a Planar Tracking"** by Xincheng Li, Tengyue Liu, and Udit Halder.

## Overview

This repository contains a ROS 2 and Python implementation of a two-agent leader-follower control using motion capture feedback. The follower agent is modeled as a nonholonomic unicycle and attempts to maintain a relative formation with the leader.

The script includes:
1. **Follower Tracking Control:** A feedback control law based on relative distance and bearing angles.
2. **Motion Capture:** Real-time pose updates from rigid body measurements (OptiTrack and Motive).
3. **Data Logging:** Automatic recording of experiment data to a CSV file for future analysis in MATLAB.

## Prerequisites

The code is written in Python for ROS 2 and requires the following:

* **Python 3**
* **ROS 2**
* `rclpy` (ROS 2 Python client library)
* `geometry_msgs` (robot command messages)
* `mocap4r2_msgs` (motion-capture messages)

## Directory Structure

* `2_agents.py` – ROS 2 node for two-agent leader-follower control

## Usage

To run the controller:
1. Make sure the ROS 2 environment is sourced.
2. Ensure motion capture data is being published on the expected topic.
3. Run the Python node.

