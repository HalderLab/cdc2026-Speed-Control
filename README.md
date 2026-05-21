# On Feedback Speed Control for a Planar Tracking

[![MATLAB](https://img.shields.io/badge/MATLAB-Simulation-blue.svg)](https://www.mathworks.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This is the official code repository for the paper: **"On Feedback Speed Control for a Planar Tracking"** by Xincheng Li, Tengyue Liu, and Udit Halder.

## Overview
While steering control in multi-agent formations has been extensively studied, the role of active speed regulation is often overlooked. This repository provides the codebase to validate a novel **feedback speed control law** paired with a **constant bearing (CB) steering strategy**.

The framework allows a follower agent (modeled as a nonholonomic unicycle) to maintain a rigid "abreast" formation (Bertrand mate) with a moving leader. The repository is divided into two main components: pure numerical simulations validating the mathematical proofs, and a hardware deployment package for physical mobile robots.
The control framework allows a follower agent (modeled as a nonholonomic unicycle) to maintain a rigid "abreast" formation (Bertrand mate) with a leader agent. The codebase includes simulations demonstrating:
1. **Ideal Tracking:** Asymptotic stability when the leader's steering effort is known.
2. **Leader-Independent Tracking:** Input-to-State Stability (ISS) and periodic orbit convergence when the leader's steering is unknown to the follower.
3. **N-Agent Chain Network:** Propagation of steering impulses through a multi-agent flock using cascaded pairwise tracking.

## Demo Video

[![Robotic Demonstration of Feedback Speed Control for Planar Leader-Follower Tracking](https://img.youtube.com/vi/0pK7FSkUUMg/0.jpg)](https://youtu.be/0pK7FSkUUMg)

This video demonstrates leader-follower formation control using two TurtleBot3 Burger robots.

## Contact

For questions regarding the code, hardware setup, or the mathematical proofs, please open an issue or contact the corresponding author:

[Udit Halder](https://sites.google.com/view/udithalder/)
