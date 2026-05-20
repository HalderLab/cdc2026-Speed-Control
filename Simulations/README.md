This repository contains the official simulation code for the paper: **"On Feedback Speed Control for a Planar Tracking"** by Xincheng Li, Tengyue Liu, and Udit Halder.

## Overview
While steering control in multi-agent formations is extensively studied, the role of active speed regulation is often overlooked. This repository provides the numerical simulations for a novel feedback speed control law paired with a constant bearing (CB) steering strategy. 

The control framework allows a follower agent (modeled as a nonholonomic unicycle) to maintain a rigid "abreast" formation (Bertrand mate) with a leader agent. The codebase includes simulations demonstrating:
1. **Ideal Tracking:** Asymptotic stability when the leader's steering effort is known.
2. **Leader-Independent Tracking:** Input-to-State Stability (ISS) and periodic orbit convergence when the leader's steering is unknown to the follower.
3. **N-Agent Chain Network:** Propagation of steering impulses through a multi-agent flock using cascaded pairwise tracking.

## Prerequisites
The numerical simulations are written in MATLAB and rely on standard built-in ODE solvers. No additional toolboxes are strictly required.
* **MATLAB** (Tested on R2023a or newer)


## Directory Structure
* `bertrand_simulation_2agent.m`: The primary script for simulating the two-agent leader-follower formation.
* `bertrand_simulation_Nagent.m`: The script for simulating the N-agent chain network.
* `animate2p.m` and `animateN.m`: The script for visualizing the results of the simulations for the two-agent and N-agent case respectively.
* `Nparticles.slx`: The Simulink file for simulating the N-agent chain network.

## Usage
To run the two-agent simulation:
1. Open `bertrand_simulation_2agent.m` in MATLAB.
2. Toggle the control mode variable (e.g., `p.know = 1/0`) to test Prop 3.1 vs. Prop 3.2.
3. Run the script. It utilizes `ode45` with tolerances set to `1e-5` (relative) and `1e-6` (absolute).
4. The script will automatically output the 2D spatial trajectories and the polar coordinate phase portraits as well as a video animating the simulation results.
