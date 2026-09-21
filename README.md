# DroneSimulation

A MATLAB simulation of a quadrotor racing through a gated course. It plans a smooth path between waypoints, flies it with a real flight controller, and replays the run in an interactive 3D viewer.
<div align="center">
  <img width="416" height="276" alt="image" src="https://github.com/user-attachments/assets/96860bcf-c4eb-41ea-9f53-74b9c1a59ecc" />
</div>

## Quick start
1. Open `DroneSimulation.prj` in MATLAB, or just add the folder to your path.
2. Run:
   ```matlab
   main
   ```
3. Static plots pop up first, then the flight viewer. Hit Play and try the camera dropdown.

Needs MATLAB plus UAV Toolbox (for `uavScenario`, `uavPlatform`, `show3D`, `eul2quat`).

## Features

- Plans a cubic-polynomial path through any set of 3D waypoints, with continuous velocity across legs so the drone doesn't stop dead at each gate
- Flies the path with a cascaded position/attitude/thrust PD controller and full 12-state rigid-body dynamics, integrated with `ode45`
- Scores the actual flight against each waypoint as a circular racing gate, reporting PASS/MISS and miss distance per gate
- Interactive 3D replay with play/pause, a scrub slider, and five camera modes (isometric, top-down, side, front, chase)
- Gate rings in the viewer are colored by the real flight result (yellow for passed, red for missed), not the planned path

## How to run it locally

You need MATLAB with UAV Toolbox installed, nothing else. Open `DroneSimulation.prj`, or `cd` into the folder and run `main` directly. Waypoints and physics constants live at the top of `main.m`; see Adjustable parameters below.

## How it works

The controller is a cascade. An outer position-PD loop turns waypoint tracking error into a desired force vector. That force gets converted into a desired tilt angle using the small-angle relationship `F_x ≈ mass·g·θ`. An inner attitude-PD loop then turns the gap between desired and current tilt into motor torque. This is closer to how real flight controllers are built, position loop outer and slower, attitude loop inner and faster, than trying to command torque straight from position error, which is both numerically worse behaved and less physically honest.

Gate scoring runs on the actual flown path, not the planned one. `evaluateGatePasses.m` walks the `ode45`-integrated state, finds the sample closest to each gate's plane, and measures the in-plane miss distance there. That matters because the controller can overshoot or cut a corner the ideal cubic path never would.

## Files

| File | Purpose |
|---|---|
| `main.m` | Entry point. Defines waypoints and physical constants, builds the per-leg trajectories, runs the `ode45` simulation, plots results, and launches the gate evaluation and flight viewer. |
| `droneDynamics.m` | ODE right-hand side: the cascaded position/attitude/force controller and the 12-state equations of motion, called once per `ode45` step. |
| `cubicTrajectory.m` | Solves the 4 coefficients of a cubic polynomial between two points given boundary position/velocity constraints. |
| `evalCubic.m` | Evaluates a cubic's position, velocity, and acceleration at a given time. |
| `rotationalMatrix.m` | Builds the body-to-world rotation matrix (ZXY Euler convention) from roll/pitch/yaw. |
| `bodyZAxis.m` | Returns the world-frame direction the drone's thrust is currently pointing. |
| `evaluateGatePasses.m` | Post-flight check of whether the trajectory passed through each gate's disc, and by how much it missed if not. Prints a text report. |
| `uavFlightViewer.m` | Interactive 3D replay of the flight using UAV Toolbox, with playback controls, gate rings colored by pass/fail, and a procedural ground plane. |

## Adjustable parameters

All set in `main.m`:

- `waypoints`: the 3xN list of gate positions to fly through
- `mass`, `speed`: vehicle mass and nominal travel speed between waypoints
- `Kp`, `Kd`: outer-loop (position) proportional/derivative gains
- `Kr`, `Kw`: inner-loop (attitude) angle-error and spin-damping gains
- `I`: inertia tensor, currently placeholder diagonal values, swap in real vehicle numbers for accurate attitude dynamics
- `gate_radius`: used both for drawing gates and for the PASS/MISS check

Thank you for reading this!
