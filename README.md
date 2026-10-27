# Drone Simulation V2

A MATLAB simulation of a quadrotor flying through a gated racing course. It plans a path through waypoints, flies it with a cascaded PD controller, and replays the flight in an interactive 3D viewer.

<div align="center">
  <img width="416" height="276" alt="image" src="https://github.com/user-attachments/assets/96860bcf-c4eb-41ea-9f53-74b9c1a59ecc" />
</div>

## Features

- Cubic-polynomial path through any set of 3D waypoints, with continuous velocity between legs
- Cascaded position/attitude PD controller on a 12-state rigid-body model (`ode45`)
- PASS/MISS scoring and miss distance for each gate, based on the path actually flown
- 3D replay with play/pause, scrub slider, and five camera modes
- Gate rings colored by result (yellow = passed, red = missed)

## Control

An outer position-PD loop turns tracking error into a desired force. That force becomes a desired tilt via $F_x \approx m g \theta$, and an inner attitude-PD loop turns the tilt error into motor torque.

## Files

| File | Purpose |
|---|---|
| `main.m` | Entry point: parameters, trajectories, simulation, plots |
| `droneDynamics.m` | Controller and equations of motion |
| `cubicTrajectory.m` | Cubic coefficients from boundary conditions |
| `evalCubic.m` | Evaluates a cubic's position, velocity, acceleration |
| `rotationalMatrix.m` | Body-to-world rotation (ZXY Euler) |
| `bodyZAxis.m` | World-frame thrust direction |
| `evaluateGatePasses.m` | Post-flight gate check and report |
| `uavFlightViewer.m` | Interactive 3D replay |

## Parameters

Set at the top of `main.m`: `waypoints`, `mass`, `speed`, `Kp`, `Kd`, `Kr`, `Kw`, `I`, `gate_radius`.

## How to Run

Requires MATLAB with UAV Toolbox. Open `DroneSimulation.prj` or add the folder to your path, then run:

```matlab
main
```

Static plots appear first, then the flight viewer.
Thank you for reading!
