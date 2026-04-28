# ME 656: Autonomous Navigation for Mobile Robots

[![MATLAB](https://img.shields.io/badge/language-MATLAB-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![Course](https://img.shields.io/badge/course-ME%20656-blue)](https://github.com/crossfirev/ME656-AutonomousRobots)
[![Simulation Labs](https://img.shields.io/badge/simulation%20labs-2-5319e7)](https://github.com/crossfirev/ME656-AutonomousRobots)
![GitHub last commit](https://img.shields.io/github/last-commit/crossfirev/ME656-AutonomousRobots)

*Stevens Institute of Technology - Schaefer School of Engineering and Science*

This repository contains MATLAB simulation labs, reports, and generated results
for **ME 656: Autonomous Navigation for Mobile Robots**, a graduate robotics
course in the Department of Mechanical Engineering at Stevens Institute of
Technology.

---

## Course Description

This course covers mobile robot geometry, kinematics, and dynamics; control
and estimation for autonomous vehicles; motion planning with discrete,
continuous, and sampling-based methods; simultaneous localization and mapping;
active SLAM and autonomous exploration with occupancy grids; Markov decision
processes and partially observable Markov decision processes for planning under
uncertainty; and machine learning applications in autonomous navigation.

The simulation labs in this repository focus especially on how robot motion,
sensor measurements, map features, randomized planners, and uncertainty models
affect navigation performance.

Topics represented in the repository include:

- Batch least-squares localization and SLAM
- Kalman filtering for recursive state estimation
- Landmark sensing and loop-closure-style constraints
- Rapidly-exploring Random Tree (RRT) path planning
- Collision checking in a 2D obstacle workspace
- Localization uncertainty propagation along planned paths
- Particle-filter visualization for selected trajectories

---

## Learning Outcomes

The official student learning outcomes for ME 656 emphasize the ability to:

1. Apply control-system and state-estimation design techniques to mobile robots.
2. Model disturbances that introduce uncertainty into mobile robotic systems.
3. Use simulation to predict mobile robot performance and tune navigation solutions.
4. Analyze robotics research literature on contemporary mobile navigation topics.
5. Evaluate modeling assumptions and algorithms for localization, mapping, and planning.

The labs in this repository support those outcomes through MATLAB simulation,
technical reporting, and comparison of estimation and planning algorithms.

---

## Repository Structure

```text
.
|-- simlab1/
|   |-- sim.m                         # Driver for all SimLab 1 deliverables
|   |-- sim1.m ... sim5.m             # Least-squares and Kalman-filter studies
|   |-- problem_setup_orig.m          # Provided starter parameters
|   |-- Deliverable*.png              # Generated result figures
|   |-- report.tex                    # SimLab 1 report source
|   `-- Simlab1_Report_Matthew_Lepis.pdf
|-- simlab2/
|   |-- simlab2.m                     # Main RRT/Kalman/particle-filter driver
|   |-- simulation_configuation.m     # Workspace and algorithm configuration
|   |-- RRT.m, Tree.m, TreeNode.m     # RRT implementation
|   |-- propagate_KF_path.m           # Kalman covariance propagation
|   |-- collect_sensor_data.m         # Range-beam observability logic
|   |-- find_shortest_path.m
|   |-- find_best_path.m
|   |-- plot_*.m                      # Result visualization utilities
|   |-- original_files/               # Provided helper files
|   `-- report/
|       |-- report.tex                # SimLab 2 report source
|       |-- report.pdf
|       `-- Figure_*.png
|-- .gitignore
`-- README.md
```

---

## Getting Started

Clone the repository:

```bash
git clone https://github.com/crossfirev/ME656-AutonomousRobots.git
cd ME656-AutonomousRobots
```

Run SimLab 1 from MATLAB:

```matlab
cd simlab1
sim
```

Run SimLab 2 from MATLAB:

```matlab
cd simlab2
simlab2
```

SimLab 2 is currently configured for a large RRT run. For quicker iteration,
reduce the trial count in the call to `simulation_configuation` inside
`simlab2.m`.

---

## Simulation Labs

### SimLab 1: Localization and SLAM in a 1D Hallway

SimLab 1 studies a robot moving through a 10 m hallway with landmarks at
2 m, 5 m, and 8 m. The simulations compare odometry-only estimation against
landmark-aided estimation using both batch least squares and Kalman filtering.
The final deliverable sends the robot out and back through the hallway to
show how repeated landmark observations help reduce drift.

### SimLab 2: RRT Planning with Localization Uncertainty

SimLab 2 plans paths through a 2D obstacle workspace using RRT. Candidate paths
are evaluated by distance and by terminal localization uncertainty, computed
through Kalman-filter covariance propagation. The lab compares the shortest
path, minimum-uncertainty path, maximum-uncertainty path, and a weighted
best-tradeoff path. Extra-credit particle-filter plots visualize belief
evolution along selected paths.

---

## Course Logistics

- **Instructor:** Prof. Brendan Englot
- **Semester:** Spring 2026
- **Prerequisite:** ME 598, Introduction to Robotics
---

## Course Topics

- Challenges of robust autonomy
- Control systems for mobile robots
- State estimation, observation, and Kalman filters
- Simultaneous localization and mapping (SLAM)
- Classic papers in mobile robotics
- Motion planning
- Planning under uncertainty
- Robotics and machine learning

---

## Materials

Suggested textbooks from the syllabus:

- *Autonomous Mobile Robots*, 2nd Ed., Siegwart, Nourbakhsh, and Scaramuzza
  (ISBN 0262015358)
- *Principles of Robot Motion*, Choset, Lynch, Hutchinson, Kantor, Burgard,
  Kavraki, and Thrun (ISBN 0262033275)
- *Probabilistic Robotics*, Thrun, Burgard, and Fox (ISBN 0262201623)
- *Planning Algorithms*, Steven M. LaValle (ISBN 0521862051), available at
  <http://lavalle.pl/planning>

---

## Tools

- MATLAB for simulation, plotting, and analysis
- LaTeX for report generation
- Zoom, Canvas, Microsoft Word, and PowerPoint for course participation and deliverables
- Git/GitHub for version control and coursework organization

---

## Notes

- This repo reflects coursework for Stevens Institute of Technology
  (**Spring 2026**).
- All code is original unless otherwise noted in the assignment files or
  reports.
- Academic integrity is expected. Please do not copy these solutions directly
  if you are currently enrolled in ME 656.

---

## Author

Maintained by **Matthew Lepis**  
Master of Engineering - Robotics, Stevens Institute of Technology  
<https://www.linkedin.com/in/matthewlepis/>
