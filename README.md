# Autonomous-Rendezvous-and-Proximity-Operations-Simulation

A MATLAB-based simulation framework for studying autonomous rendezvous, proximity operations, and docking decision logic for a chaser spacecraft approaching a non-cooperative satellite in geostationary orbit (GEO).
Project status: Simulation prototype / ongoing development. The current baseline run completed 20 Monte Carlo trials with 0 successful docking events (0%) and 0 recorded abort events. The framework is useful for studying relative-motion modelling and controller behaviour, but successful and safe docking has not yet been demonstrated.

Table of Contents
- 1. Overview
- 2. Problem Statement
- 3. Objectives
- 4. System Concept
- 5. Coordinate Frame
- 6. Mathematical Model
- 7. Guidance and Control
- 8. Safety and Docking Logic
- 9. Monte Carlo Simulation
- 10. Software Workflow
- 11. Running the Simulation
- 12. Current Results
- 13. Current Limitations
- 14. Planned Improvements
- 15. Applications
- 16. References and Further Reading
1. Overview
Geostationary satellites provide services such as telecommunications, broadcasting, and weather monitoring. A satellite that has reached the end of its operational life, has lost control, or is otherwise non-responsive can become difficult to inspect, repair, reposition, or remove.

This project explores a simulation approach for an autonomous Chaser Spacecraft to approach an unprepared Target Satellite. The target is treated as non-cooperative: it is not assumed to provide docking interfaces, visual markers, or navigation data for the chaser.

The MATLAB prototype models three-dimensional relative motion, applies a Proportional-Derivative (PD) control law, represents a keep-out region, and evaluates docking and abort conditions over multiple randomized initial states.

The project is a simulation study. It does not currently implement a flight-qualified navigation system, a validated collision-avoidance system, or physical docking hardware.
2. Problem Statement
Conventional docking systems often rely on a cooperative target with known docking interfaces, fiducial markers, or communication and navigation support. An unprepared GEO satellite may not provide these features.

An autonomous servicing spacecraft therefore needs to:
1. Estimate its relative position and velocity with respect to the target.
2. Approach the target in a controlled manner.
3. Maintain safe separation during proximity operations.
4. Detect conditions in which continuing the approach may be unsafe.
5. Decide whether the docking conditions have been satisfied or an abort response is required.
This project focuses on the simulation of relative motion and preliminary control and decision logic for this scenario.
3. Objectives
- Model the relative motion of a chaser spacecraft with respect to a target satellite.
- Represent the motion in a three-dimensional local orbital reference frame.
- Implement a PD controller to reduce position and velocity errors.
- Include preliminary keep-out-zone (KOZ) representation and abort-threshold logic.
- Define position and relative-velocity conditions for a docking event.
- Run Monte Carlo trials with randomized initial position and velocity.
- Visualize the simulated trajectories and summarize trial outcomes.
4. System Concept
The simulation contains two conceptual spacecraft:
Element	Description
Target satellite	The reference object treated as non-cooperative
Chaser spacecraft	The servicing spacecraft whose relative motion is simulated
Relative state	Chaser position and velocity with respect to the target
Controller	PD control law that generates commanded acceleration
Safety logic	Preliminary abort-threshold and docking-condition checks
Simulation environment	MATLAB numerical integration and 3D visualization


High-level workflow
Initialize simulation parameters
            |
            v
Generate randomized chaser initial state
            |
            v
Calculate relative position and velocity
            |
            v
Compute PD control acceleration
            |
            v
Evaluate preliminary safety / abort condition
            |
            v
Propagate relative orbital dynamics
            |
            v
Update and display 3D trajectory
            |
            v
Check docking condition
       /          \
   Satisfied     Not satisfied
      |               |
      v               v
 Record docking    Continue until
    success        end of simulation
                      |
                      v
                 Record outcome
                      |
                      v
              Repeat Monte Carlo trial
5. Coordinate Frame
The trajectory is visualized using a local orbital reference frame:
- R-bar (radial): Direction along the local radial line from the central body through the target.
- V-bar (along-track): Direction approximately aligned with the target's orbital velocity.
- Cross-track: Direction normal to the target's orbital plane.
The simulation uses the state variables:
\[
\mathbf{r} = [x,\ y,\ z]^T
\]
\[
\mathbf{v} = [\dot{x},\ \dot{y},\ \dot{z}]^T
\]
where the coordinates represent relative position and velocity in the selected local orbital frame.

The plotted origin represents the target reference point. The desired docking point is configured in the code as:
\[
\mathbf{r}_{dock} = [0,\ -1,\ 0]^T
\]
The exact interpretation of the axis directions should be kept consistent throughout the implementation and plot labels.
6. Mathematical Model
The prototype uses the Clohessy-Wiltshire (CW) equations, also known as Hill's relative-motion equations. These are linearized equations for relative motion near a circular reference orbit.

The equations used in the simulation are:
\[
\ddot{x} = 3n^2x + 2n\dot{y} + a_x
\]
\[
\ddot{y} = -2n\dot{x} + a_y
\]
\[
\ddot{z} = -n^2z + a_z
\]
where:
Symbol	Meaning
\(x,y,z\)	Relative position components
\(\dot{x},\dot{y},\dot{z}\)	Relative velocity components
\(\ddot{x},\ddot{y},\ddot{z}\)	Relative acceleration components
\(n\)	Mean motion of the reference orbit
\(a_x,a_y,a_z\)	Control acceleration components


The current MATLAB prototype uses numerical time-step propagation. In the supplied implementation, velocity is updated before position at each time step (a semi-implicit Euler-style update).

Important modelling note: The current code sets n = 0.0011. This value must be checked against the intended reference orbit before interpreting the simulation as a GEO case. For an ideal circular GEO orbit, the mean motion is approximately \(7.2921 \times 10^{-5}\ \text{rad/s}\). The simulation should use a consistent unit system and a physically justified value.
7. Guidance and Control
A Proportional-Derivative (PD) controller is used to generate commanded acceleration based on position and velocity error.

The controller is implemented in the form:
\[
\mathbf{a}_{cmd} =
-K_p(\mathbf{r}-\mathbf{r}_{dock})-K_d\mathbf{v}
\]
where:
- \(K_p\) is the proportional gain.
- \(K_d\) is the derivative gain.
- \(\mathbf{r}\) is the current relative position.
- \(\mathbf{r}_{dock}\) is the desired docking position.
- \(\mathbf{v}\) is the relative velocity.
The proportional term drives the chaser toward the desired position. The derivative term provides damping based on relative velocity.

The current prototype uses:
Parameter	Current code value
Proportional gain, Kp	0.001
Derivative gain, Kd	0.15
Simulation time step, dt	1 s
Simulation duration, T	2500 s
Number of Monte Carlo trials	20


These are prototype parameters, not validated flight-control gains. Their suitability must be evaluated through tuning, stability analysis, and simulation testing.
8. Safety and Docking Logic
8.1 Keep-Out Zone (KOZ)
A keep-out zone is a protected region around the target. A chaser should not enter this region unless it is following an approved approach trajectory and satisfying the required safety constraints.

The current code draws a sphere to represent the KOZ in the 3D visualization. A visual sphere alone does not enforce collision avoidance. A separate, continuously evaluated KOZ constraint and a verified avoidance trajectory are required for a functional safety system.
8.2 Abort-threshold logic
The prototype checks whether the chaser is within a configured distance threshold while its relative speed exceeds an abort velocity limit. If the condition is met, an abort flag is set and a simple outward acceleration term is applied.

This is preliminary logic only. The outward acceleration in the current code is not a validated safe-retreat guidance law, and the implementation does not establish that the chaser reaches a safe stand-off point.
8.3 Docking condition
The code defines the following docking thresholds:
Parameter	Current code value
Docking distance threshold	0.2 m
Docking relative-speed threshold	0.02 m/s


A docking event is intended to be recorded when both the position and relative-speed conditions are satisfied.

Implementation detail to verify: The current code calculates distance from the coordinate origin, while the desired docking point is [0 -1 0]. For a correct docking-point error, calculate:
\[
d_{dock} = \|\mathbf{r}-\mathbf{r}_{dock}\|
\]
The state used for the docking check should also be calculated after the current integration step so that the check uses the updated state.
9. Monte Carlo Simulation
Monte Carlo simulation is used to evaluate the controller from multiple initial conditions rather than relying on one trajectory.

For each trial, the current code randomizes the initial relative position and velocity using normally distributed variations around configured starting values. The simulation then propagates the chaser state and records whether a docking or abort flag was triggered.

The current setup uses:
- 20 simulation trials.
- Randomized initial position in all three axes.
- Randomized initial velocity in all three axes.
- A maximum simulated duration of 2500 seconds per trial.
Monte Carlo testing can help reveal sensitivity to initial conditions. However, 20 trials are not enough to establish robust reliability, and the current implementation does not yet log a detailed reason for every unsuccessful trial.
10. Software Workflow
The current prototype is implemented in MATLAB and uses:
- MATLAB script execution for initialization and simulation.
- Numerical propagation of relative orbital dynamics.
- PD control calculations.
- Conditional checks for preliminary abort and docking events.
- 3D plotting for trajectory visualization.
- A loop to execute repeated Monte Carlo trials.
Suggested repository structure
Autonomous-RPOD-GEO/
├── README.md
├── LICENSE
├── .gitignore
├── src/
│   └── rpod_monte_carlo.m
├── docs/
│   ├── project_overview.md
│   └── model_and_assumptions.md
└── results/
    └── figures/
The filenames above are a suggested organization. Rename the script path in the instructions below to match the actual file you upload.
11. Running the Simulation
Requirements
- MATLAB (the project was developed as a MATLAB simulation).
- A desktop environment capable of displaying MATLAB figures.
No external toolbox dependency is specified by the current script.
Steps
1. Clone or download this repository.
2. Open MATLAB.
3. Set the MATLAB Current Folder to the repository directory.
4. Open the simulation script.
5. Run the script.
For example, if the script is stored at src/rpod_monte_carlo.m, run:
run('src/rpod_monte_carlo.m')
The script should open a 3D figure showing the simulated trajectories and print the Monte Carlo summary in the MATLAB Command Window.
12. Current Results
The baseline output reported for the current implementation is:
Metric	Result
Monte Carlo trials	20
Successful docking events	0
Recorded abort events	0
Docking success rate	0%

<img width="1600" height="900" alt="WhatsApp Image 2026-11-17 at 5 30 13 PM" src="https://github.com/user-attachments/assets/01762c7d-6587-4253-a991-fe359ad01e36" />
<img width="1600" height="900" alt="WhatsApp Image 2026-11-17 at 5 30 13 PM (1)" src="https://github.com/user-attachments/assets/3c50c3da-72d9-441a-a2c9-d6f0c0bcabbf" />
<img width="1600" height="900" alt="WhatsApp Image 2026-11-17 at 5 30 14 PM" src="https://github.com/user-attachments/assets/8e79b505-60f4-459b-a424-4c98f13a5ad1" />




These results mean that the current run did not demonstrate a successful docking event. Since no abort events were recorded either, the trial-level outcomes need further instrumentation to determine whether trials timed out, failed to converge, or encountered another condition.

Do not interpret the current output as proof that the approach is safe or that the controller is robust. It is a baseline result that identifies the need for further debugging and validation.
13. Current Limitations
The current prototype has several limitations that should be addressed before drawing engineering conclusions:
1. Reference-orbit parameter: The value of n in the code needs to be checked and corrected for the intended GEO reference orbit.
2. Docking-point distance: The current distance calculation is relative to the origin rather than the configured docking point.
3. KOZ enforcement: The KOZ is represented visually, but a complete geometric constraint and collision-avoidance response are not implemented.
4. Abort guidance: The current outward acceleration is a simplified response and is not a validated safe-retreat trajectory.
5. State-check timing: Distance and speed should be recalculated after state propagation before testing docking or abort conditions.
6. Trial outcome logging: Each trial should record success, abort, timeout, and any safety-constraint violation separately.
7. Controller validation: The PD gains have not been demonstrated to provide stable convergence under the full relative-motion dynamics.
8. Model fidelity: The current CW model does not include all relevant perturbations and operational effects, such as solar radiation pressure, Earth's non-spherical gravity effects, actuator limits, sensor noise, navigation uncertainty, or target rotation/tumbling.
9. Non-cooperative navigation: The project concept concerns an unprepared target, but a complete vision-based or sensor-fusion relative-navigation subsystem is not yet implemented.
14. Planned Improvements
Potential next development steps include:
- Correct and document the GEO mean-motion parameter and unit conventions.
- Calculate position error with respect to the actual docking point.
- Recompute relative distance and velocity after each integration step.
- Implement explicit KOZ entry detection and a geometrically defined safety boundary.
- Design and test a controlled retreat trajectory for abort cases.
- Add trial-level logs for docking, abort, timeout, KOZ entry, minimum separation, and maximum relative speed.
- Tune the controller and evaluate stability over a wider range of initial conditions.
- Increase the number of Monte Carlo trials and report statistical performance.
- Add navigation uncertainty, sensor noise, and actuator constraints.
- Investigate higher-fidelity orbital perturbations and target attitude/tumbling effects.
- Add plots for position error, relative speed, control acceleration, and distance to the KOZ boundary.
15. Applications
The broader RPOD problem is relevant to:
- On-orbit inspection of inactive or malfunctioning satellites.
- Satellite life-extension and servicing missions.
- Close-proximity inspection of non-cooperative spacecraft.
- Debris characterization and potential removal missions.
- Future autonomous space infrastructure.
These are potential application areas for the research problem; they are not capabilities demonstrated by the current prototype.
16. References and Further Reading
The following topics are useful starting points for understanding and extending the simulation:
1. Clohessy-Wiltshire / Hill equations for relative orbital motion.
2. Spacecraft rendezvous, proximity operations, and docking (RPOD).
3. Relative navigation for non-cooperative spacecraft.
4. Keep-out zones and safety corridors for proximity operations.
5. Guidance, navigation, and control (GNC) for autonomous spacecraft.
6. Monte Carlo methods for simulation-based performance evaluation.
Add the exact papers, textbooks, mission reports, and URLs consulted by the project team here. Do not list a source as a project reference unless it has actually been reviewed.
Project Disclaimer
This repository contains an academic simulation prototype for learning and analysis. It is not flight software and must not be used to control a real spacecraft. The current results do not demonstrate successful docking, collision avoidance, or flight readiness.
