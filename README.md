# Self-Balancing Robot - Control Systems Project

A two-wheel self-balancing robot built from scratch — from first-principles dynamic
modelling and simulation to embedded control on hardware.

## Project Stages

-  **Stage 0** - Simulation of a single body pendulum: PID control, gain tuning, documented results
- **Stage 1** - Full coupled cart-pole simulation: explicit equations of motion, cascaded PID, pole placement analysis, state feedback control
-  **Stage 2** - Hardware design: Component selection, mechanical design, CAD, measurment of physical parameters
-  **Stage 3** - Embedded implementation: Microcontroller, IMU integration, sensor filtering, control on hardware
-  **Stage 4** - Testing and validation: Comparing hardware performance against simulation, tuning on real system

## Skills Demonstrated

- Dynamic modelling and equations of motion from first principles
- State-space modelling and full-state feedback control
- Pole placement design and closed-loop stability analysis
- PID control design with anti-windup and derivative on measurement
- Discrete-time simulation in Python
- Gain tuning and performance analysis
- Engineering documentation

## Repository Structure

```
analysis/     Pole placement and recovery-limit analysis scripts
simulation/   Plant model, controllers, and simulation entry points
hardware/     CAD source and mechanical design
embedded/     Arduino firmware
docs/         Derivations and stage-by-stage results
```

## Results - Stage 0

*Nominal tuning — stable convergence within 0.5 seconds*

<img src="docs/results/stage0/kp50_ki1_kd5_nominal.png" width="600"><br>

*Controller recovery limit — 5 N.m torque limit, all angles recovered but only up to 10° converge to zero*

<img src="docs/results/stage0/angle_sweep_5nm_limits.png" width="600">

[View full Stage 0 results](docs/results/stage0/README.md)

## Results - Stage 1

*Cascaded PID — oscillates, does not fully converge*

<img src="docs/results/stage1/cascaded_pid_before_tuning.png" width="600"><br>

*State feedback with pole placement — converges to zero*

<img src="docs/results/stage1/state_feedback_pole_placement.png" width="600">

[View full Stage 1 results](docs/results/stage1/README.md)


## Setup

Requires Python 3.13.

```bash
git clone https://github.com/AbdullahSaedan/Self-Balancing-Robot.git
cd Self-Balancing-Robot
conda create -n pid python=3.13
conda activate pid
pip install -r requirements.txt
```

Run a simulation:

```bash
python simulation/simulate.py
python simulation/simulate_state_feedback.py
```